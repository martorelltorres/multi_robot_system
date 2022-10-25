#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import Imu, MagneticField
from std_srvs.srv import Empty, EmptyResponse
import tf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import datetime
import numpy as np 
from numpy import *
from math import *
import PyKDL
import sys
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger, TriggerRequest
from cola2_msgs.msg import GoalDescriptor, NavSts, BodyVelocityReq
from cola2_msgs.srv import Goto, GotoRequest,GotoResponse
from cola2_xiroi.srv import SharePosition
from cola2_lib.utils.angles import wrap_angle
from cola2_lib.utils.ned import NED
from std_srvs.srv import Empty, EmptyRequest
from cola2_lib.rosutils import param_loader
from cola2_lib.rosutils.param_loader import get_ros_params
from multi_robot_system.msg import AvoidCollision
#import classes
from robot import robot
from area_partition import area_partition
from task_allocator import task_allocation


class CollisionAvoidance:
    
    def __init__(self, name):
        """ Init the class """
        # Initialize some parameters
        self.robot1_data = False
        self.robot2_data = False
        self.robot3_data = False

        # self.first_repulsion = False
        self.initialized = False
        self.radius= -1
        self.x =0
        self.repulsion_radius = rospy.get_param("repulsion_radius", default=10)
        self.anchor_radius = rospy.get_param("anchor_radius", default=20)
        self.depth_threshold = rospy.get_param("/xiroi/depth_threshold", default=3)
        self.number_of_robots = rospy.get_param('number_of_robots')

        self.r1_navigation_topic = rospy.get_param('/multi_robot_system_turbot1/navigation_topic',default='/turbot1/navigator/navigation') 
        self.r2_navigation_topic = rospy.get_param('/multi_robot_system_turbot2/navigation_topic',default='/turbot2/navigator/navigation') 
        self.r3_navigation_topic = rospy.get_param('/multi_robot_system_xiroi/navigation_topic',default='/xiroi/navigator/navigation') 

        self.robot1_name = rospy.get_param('~/robot1_name',default='turbot1') 
        self.robot2_name = rospy.get_param('~/robot2_name',default='turbot2')
        self.robot3_name = rospy.get_param('~/robot3_name',default='xiroi') 


        self.robot_handler = robot("robot")
        self.area_handler =  area_partition("area_partition")
        self.task_allocation_handler = task_allocation("task_allocation")

        # get general info from robots
        self.coverage_times = self.area_handler.get_estimated_polygons_coverage_time()
        self.goals,self.central_polygon = self.task_allocation_handler.task_allocation()
       
        # Get config parameters
        self.name = name
        self.ns = rospy.get_namespace()

        self.robot_names =[]
        self.robot_names.append(self.robot1_name)
        self.robot_names.append(self.robot2_name)
        self.robot_names.append(self.robot3_name)

        #Subscribers
        rospy.Subscriber(self.r1_navigation_topic,
                         NavSts,
                         self.update_nav_status,
                         1,#send a robot id to the callback function
                         queue_size=1)   

        rospy.Subscriber(self.r2_navigation_topic,
                         NavSts,
                         self.update_nav_status,
                         2,
                         queue_size=1)         

        rospy.Subscriber(self.r3_navigation_topic,
                         NavSts,
                         self.update_nav_status,
                         3,
                         queue_size=1)

        #Publishers
        self.markerPub_repulsion = rospy.Publisher('repulsionRadius',
                                                    Marker,
                                                    queue_size=1)

        self.markerPub_anchor = rospy.Publisher('anchorRadius',
                                                Marker,
                                                queue_size=1)

        self.collision_avoidance_pub = rospy.Publisher('collission_avoidance_info',
                                            AvoidCollision,
                                            queue_size=1)
  
    def update_nav_status(self,msg,robot):

        if(robot==0):
            if(self.robot1_data == False):
                robot_name = self.robot1_name
                self.set_services(robot_name)
            self.robot1_position_north = msg.position.north
            self.robot1_position_east = msg.position.east
            self.robot1_position_depth = msg.position.depth
            self.robot1_yaw = msg.orientation.yaw
            self.robot1_data = True
            self.anchor_marker(self.robot1_position_north,self.robot1_position_east,robot)
            self.repulsion_marker(self.robot1_position_north,self.robot1_position_east,robot)
            self.system_init()

        elif(robot==1):
            if(self.robot2_data == False):
                robot_name = self.robot2_name
                self.set_services(robot_name)
            self.robot2_position_north = msg.position.north
            self.robot2_position_east = msg.position.east
            self.robot2_position_depth = msg.position.depth
            self.robot2_yaw = msg.orientation.yaw
            self.robot2_data = True
            self.anchor_marker(self.robot2_position_north,self.robot2_position_east,robot)
            self.repulsion_marker(self.robot2_position_north,self.robot2_position_east,robot)
            self.system_init()

        elif(robot==2):
            if(self.robot3_data == False):
                robot_name = self.robot3_name
                self.set_services(robot_name)
            self.robot3_position_north = msg.position.north
            self.robot3_position_east = msg.position.east
            self.robot3_position_depth = msg.position.depth
            self.robot3_yaw = msg.orientation.yaw
            self.robot3_data = True
            self.anchor_marker(self.robot3_position_north,self.robot3_position_east,robot)
            self.repulsion_marker(self.robot3_position_north,self.robot3_position_east,robot)
            self.system_init()

    def get_robot_position(self,robot_id):

        if(robot_id == 1):
            return(self.robot1_position_north,self.robot1_position_east,self.robot1_position_depth,self.robot1_yaw)
        elif(robot_id == 2):
            return(self.robot2_position_north,self.robot2_position_east,self.robot2_position_depth,self.robot2_yaw)
        elif(robot_id == 3):
            return(self.robot2_position_north,self.robot2_position_east,self.robot2_position_depth,self.robot3_yaw)

    def set_services(self,robot_name):
        # enable thrusters service
        rospy.wait_for_service("/"+str(robot_name)+'/controller/enable_thrusters', 10)
        try:
            self.enable_thrusters_srv = rospy.ServiceProxy("/"+str(robot_name)+'/controller/enable_thrusters', Empty)
        except rospy.ServiceException, e:
            rospy.logwarn("%s: Service call failed: %s", self.name, e)

        # disable thrusters service
        rospy.wait_for_service("/"+str(robot_name)+'/controller/disable_thrusters', 10)
        try:
            self.disable_thrusters_srv = rospy.ServiceProxy("/"+str(robot_name)+'/controller/disable_thrusters', 
                        Empty)
        except rospy.ServiceException, e:
            rospy.logwarn("%s: Service call failed: %s", self.name, e)


    def system_init(self):
        if(self.robot1_data == self.robot2_data == self.robot3_data == True):
            self.initialized = True

        if(self.initialized == True):
            self.robot_master = self.assign_priorities()
            self.collision_avoidance()
            
    def assign_priorities(self):
        # In order to assign the priority we have been take into account the tasks assigned to each 
        # robot and the estimated time to finish the coverage of each assigned areas 
        robot_priorities=[]      
        for robot in range(self.number_of_robots):
            self.time_task_coverage = 0
            self.number_of_tasks = len(self.goals[robot][1])
            for task in range(self.number_of_tasks):
                robot_tasks_id = self.goals[robot][1]
                self.time_task_coverage = self.time_task_coverage + self.coverage_times[robot_tasks_id[task]]
            robot_priorities.append(self.time_task_coverage)
        # The higher time_task_coverage the higher priority
        # TODO: change the method for more than one robot_master
        max_priority = max(robot_priorities)
        self.robot_master_id = robot_priorities.index(max_priority)
        #remove 
        return(self.robot_master_id)

    def collision_avoidance(self):

        for robots in range(self.number_of_robots-1):
            
            if(self.robot_master==0):
                # check robot_1 with robot_2
                self.check_distance(self.robot1_position_north,self.robot1_position_east,self.robot2_position_north,self.robot2_position_east,1,2)
                # check robot_1 with robot_3
                self.check_distance(self.robot1_position_north,self.robot1_position_east,self.robot3_position_north,self.robot3_position_east,1,3)
                # check robot_2 with robot_3
                self.check_distance(self.robot2_position_north,self.robot2_position_east,self.robot3_position_north,self.robot3_position_east,2,3)
            
            elif(self.robot_master==1):
                # check robot_2 with robot_1
                self.check_distance(self.robot2_position_north,self.robot2_position_east,self.robot1_position_north,self.robot1_position_east,2,1)
                # check robot_2 with robot_3
                self.check_distance(self.robot2_position_north,self.robot2_position_east,self.robot3_position_north,self.robot3_position_east,2,3)
                # check robot_1 with robot_3
                self.check_distance(self.robot1_position_north,self.robot1_position_east,self.robot3_position_north,self.robot3_position_east,1,3)
            
            elif(self.robot_master==2):
                # check robot_3 with robot_1
                self.check_distance(self.robot3_position_north,self.robot3_position_east,self.robot1_position_north,self.robot1_position_east,3,1)
                # check robot_3 with robot_2
                self.check_distance(self.robot3_position_north,self.robot3_position_east,self.robot2_position_north,self.robot2_position_east,3,2)
                # check robot_1 with robot_2
                self.check_distance(self.robot1_position_north,self.robot1_position_east,self.robot2_position_north,self.robot2_position_east,1,2)
        
    def check_distance(self,position_north_1, position_east_1, position_north_2, position_east_2,robot_1,robot_2):

        self.x_distance = position_north_1-position_north_2
        self.y_distance = position_east_1-position_east_2
        self.radius = sqrt((self.x_distance)**2 + (self.y_distance)**2)
        self.initialized = True
        # rospy.loginfo(self.radius)

        if(self.radius > self.anchor_radius):
            self.coverage = True
            self.adrift = False
            self.repulsion = False
            self.send_collision_avoidance_info(robot_1,robot_2)

        if(self.repulsion_radius <= self.radius <= self.anchor_radius):
            self.adrift = True
            self.repulsion = False
            self.coverage = False
            self.send_collision_avoidance_info(robot_1,robot_2)
            
        elif(self.radius <= self.repulsion_radius and self.initialized):
            self.repulsion = True
            self.adrift = False
            self.coverage = False
            self.send_collision_avoidance_info(robot_1,robot_2)

    def send_collision_avoidance_info(self,robot_1,robot_2):
        collision_msg = AvoidCollision()
        collision_msg.header.frame_id = "multi_robot_system"
        collision_msg.header.stamp = rospy.Time.now()
        collision_msg.first_robot_id = robot_1
        collision_msg.second_robot_id = robot_2
        collision_msg.distance = self.radius
        collision_msg.robot_master_id = self.robot_master_id
        collision_msg.repulsion = self.repulsion
        collision_msg.adrift = self.adrift
        collision_msg.coverage = self.coverage

        collision_msg.robot_master = self.robot_names[self.robot_master]

        if(robot_1>robot_2):
            collision_msg.robot_slave = self.robot_names[robot_1-1]
        else:
            collision_msg.robot_slave = self.robot_names[robot_2-1]

        self.collision_avoidance_pub.publish(collision_msg)

    def extract_safety_position(self,current_robot,robot_master,robot_slave):
        print("The current robot is "+str(current_robot)+" the slave robot is: "+str(robot_slave)+" and the master_robot is: "+str(robot_master))
        rospy.sleep(1)
        current_north,current_east,current_depth, self.robot1_yaw = self.get_robot_position(current_robot)
        slave_north,slave_east,slave_depth, self.robot2_yaw = self.get_robot_position(robot_slave)
        master_north,master_east,master_depth, self.master_yaw = self.get_robot_position(robot_master)


        current_slave_vector = [(slave_north-current_north),(slave_east-current_east)]
        current_master_vector = [(master_north-current_north),(master_east-current_east)]
        sum_vector = [-(current_master_vector[0]+current_slave_vector[0]),-(current_master_vector[1]+current_slave_vector[1])]

        final_position = [(current_north+sum_vector[0]),(current_east+sum_vector[1])]
        
        self.x = final_position[0]
        self.y = final_position[1]

        initial_point = [current_north,current_east]
        final_point = [self.x, self.y]
        self.robot_handler.send_section_strategy(initial_point,final_point)
        # self.robot_handler.send_goto_strategy(self.x,self.y,1)


    def repulsion_strategy(self,current_robot,master_robot,robot_slave,robot_bvr_pub):
        self.extract_safety_position(current_robot,master_robot,robot_slave)

    

    def anchor_marker(self,position_north,position_east,robot ):
        self.anchor = Marker()
        self.anchor.header.frame_id = "world_ned"
        self.anchor.header.stamp = rospy.Time.now()
        self.anchor.ns = "robot"+str(robot)
        self.anchor.id = 0
        self.anchor.type = Marker.CYLINDER
        self.anchor.action = Marker.ADD
        self.anchor.pose.position.x = position_north
        self.anchor.pose.position.y = position_east
        self.anchor.pose.position.z = 0
        self.anchor.pose.orientation.x = 0
        self.anchor.pose.orientation.y = 0
        self.anchor.pose.orientation.z = 0
        self.anchor.pose.orientation.w = 1.0
        self.anchor.scale.x = self.anchor_radius*2
        self.anchor.scale.y = self.anchor_radius*2
        self.anchor.scale.z = 0.1
        self.anchor.color.r = 0.0
        self.anchor.color.g = 1.0
        self.anchor.color.b = 0.0
        self.anchor.color.a = 0.3
        self.markerPub_anchor.publish(self.anchor)

    def repulsion_marker(self,position_north,position_east,robot ):
        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = "world_ned"
        self.robotMarker.header.stamp = rospy.Time.now()
        self.robotMarker.ns = "robot"+str(robot)
        self.robotMarker.id = 0
        self.robotMarker.type = Marker.CYLINDER
        self.robotMarker.action = Marker.ADD
        self.robotMarker.pose.position.x = position_north
        self.robotMarker.pose.position.y = position_east
        self.robotMarker.pose.position.z = 0
        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = self.repulsion_radius*2
        self.robotMarker.scale.y = self.repulsion_radius*2
        self.robotMarker.scale.z = 0.1
        self.robotMarker.color.r = 1.0
        self.robotMarker.color.g = 0.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 0.3
        self.markerPub_repulsion.publish(self.robotMarker)


if __name__ == '__main__':
    try:
        rospy.init_node('collision_avoidance')
        collision_avoidance = CollisionAvoidance(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
