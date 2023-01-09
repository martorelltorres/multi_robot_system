#!/usr/bin/env python
import rospy
import math
import numpy as np
import random 
from math import *
import matplotlib
import actionlib         
import matplotlib.pyplot as plt
from cola2_msgs.msg import  NavSts,BodyVelocityReq
from std_srvs.srv import Trigger, TriggerRequest
from cola2_msgs.srv import Goto, GotoRequest
from shapely.geometry import Point
from multi_robot_system.msg import TaskMonitoring
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
#import classes
from area_partition import area_partition
from robot import Robot

 
class DustbinRobot:
  
    def __init__(self, name):
        """ Init the class """
        rospy.sleep(7)
        self.name = name
        # Get config parameters from the parameter server
        self.number_of_robots = self.get_param('number_of_robots')
        self.robot_ID = self.get_param('~robot_ID',0) 
        self.max_distance_to_waypoint = self.get_param('~max_distance_to_waypoint',750)
        self.tolerance = self.get_param('tolerance',2)
        self.surge_velocity = self.get_param('surge_velocity',0.5)
        self.section_action = self.get_param('section_action','/robot4/pilot/world_section_req') 
        self.section_result = self.get_param('section_result','/robot4/pilot/world_section_req/result') 
        self.pose = [0,0]
        self.repulsion_radius = self.get_param("repulsion_radius", default=10)
        self.adrift_radius = self.get_param("adrift_radius", default=20)
        self.tracking_radius = self.get_param("tracking_radius", default=50)

        self.is_section_actionlib_running = False
        self.start = False
        self.robot_at_center = False
        self.robots_id = np.array([])
        self.area_handler =  area_partition("area_partition")
        self.robot_handler = Robot("robot")
        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        # Initialize some variables
        self.system_init = False
        self.robot_data = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.robots_information = []
        self.robots = []
        self.robot_initialization = np.array([])

        # initialize the robots variables
        for robot in range(self.number_of_robots):
            self.robots_information.append(self.robot_data) #set the self.robots_information initialized to 0
            self.robot_initialization = np.append(self.robot_initialization,False) # self.robot_initialization = [False,False;False]
            self.robots.append(robot)  # self.robots = [0,1,2]
            self.robots_id = np.append(self.robots_id,robot)
            self.robots_id = self.robots_id.astype(int) #convert float to int type

        #Subscribers
        for robot_id in range(self.number_of_robots):
            rospy.Subscriber(
                '/robot'+str(robot_id+1)+'/navigator/navigation',
                NavSts,
                self.update_robots_position,
                robot_id,
                queue_size=1) 

        rospy.Subscriber('/robot'+str(self.robot_ID)+'/navigator/navigation',
                            NavSts,    
                            self.update_robot_position,
                            queue_size=1)
        #Publishers
        self.corrected_bvr = rospy.Publisher('/robot'+str(self.robot_ID)+'/controller/body_velocity_req',
                                                BodyVelocityReq,
                                                queue_size=1)

        # Services clients
        try:
            rospy.wait_for_service('/robot'+str(self.robot_ID)+'/captain/enable_goto', 20)
            self.goto_srv = rospy.ServiceProxy(
                        '/robot'+str(self.robot_ID)+'/captain/enable_goto', Goto)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to goto service',
                         self.name)
            rospy.signal_shutdown('Error creating client to goto service')
        
        try:
            rospy.wait_for_service('/robot'+str(self.robot_ID)+'/captain/disable_all_and_set_idle', 20)
            self.disable_all_and_set_idle_srv = rospy.ServiceProxy(
                        '/robot'+str(self.robot_ID)+'/captain/disable_all_and_set_idle', Trigger)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to disable_all_and_set_idle service',
                         self.name)
            rospy.signal_shutdown('Error creating client to disable_all_and_set_idle service')
    
        self.dustbin_strategy()

    def update_robot_position(self, msg):
        self.asv_north_position = msg.position.north
        self.asv_east_position = msg.position.east      
        self.asv_yaw = msg.orientation.yaw
 
    def update_robots_position(self, msg, robot_id):
        # fill the robots_information array with the robots information received from the NavSts
        self.robots_information[[robot_id][self.robot_data[0]]] = msg.position.north
        self.robots_information[[robot_id][self.robot_data[1]]] = msg.position.east
        self.robots_information[[robot_id][self.robot_data[2]]]= msg.position.depth
        self.robots_information[[robot_id][self.robot_data[3]]] = msg.altitude
        self.robots_information[[robot_id][self.robot_data[4]]] = msg.global_position.latitude
        self.robots_information[[robot_id][self.robot_data[5]]] = msg.global_position.longitude
        self.robots_information[[robot_id][self.robot_data[6]]] = msg.body_velocity.x
        self.robots_information[[robot_id][self.robot_data[7]]] = msg.body_velocity.y
        self.robots_information[[robot_id][self.robot_data[8]]] = msg.body_velocity.z
        self.robots_information[[robot_id][self.robot_data[9]]] = msg.orientation.roll
        self.robots_information[[robot_id][self.robot_data[10]]] = msg.orientation.pitch
        self.robots_information[[robot_id][self.robot_data[11]]] = msg.orientation.yaw

        # check the system initialization
        if(self.system_init == False):
            self.initialization(robot_id)       

    def dustbin_strategy(self):
        # move the robot to the central area
        if (self.robot_at_center == False):
            self.goto_central_area()

        # Init periodic timer 
        rospy.Timer(rospy.Duration(30), self.get_robot_information)

    def get_robot_information(self,event):
        if(self.system_init==True):
            self.robots_id = np.roll(self.robots_id, 2)
            self.robot_goal_id = self.robots_id[0]
            # Init periodic timer 
            rospy.Timer(rospy.Duration(0.1), self.tracking)
                  
    def goto_central_area(self):
        self.central_point = self.area_handler.get_main_polygon_centroid()
        self.pose = [self.central_point.x, self.central_point.y]
        self.transit_to(self.pose)
        self.robot_at_center = True

    def initialization(self,robot_id):
        # check if all the n robots are publishing their information
        if(self.robots_information[[robot_id][self.robot_data[0]]] != 0):
            self.robot_initialization[robot_id] = True
        
        if((self.robot_initialization == True).all()):
            self.system_init = True
                 
    def tracking(self,event):
        robot_id = self.robot_goal_id 
        self.auv_position_north = self.robots_information[[robot_id][self.robot_data[0]]]
        self.asv_position_north = self.asv_north_position
        self.auv_position_east = self.robots_information[[robot_id][self.robot_data[1]]]
        self.asv_position_east = self.asv_east_position

        self.x_distance = self.auv_position_north-self.asv_position_north
        self.y_distance = self.auv_position_east-self.asv_position_east
        self.radius = sqrt((self.x_distance)**2 + (self.y_distance)**2)
        self.initialized = True
        rospy.loginfo(self.radius)

        if(self.radius > self.adrift_radius):
            rospy.loginfo("----------------- TRACKING STRATEGY -----------------")
            self.tracking_strategy()

        elif(self.repulsion_radius <= self.radius <= self.adrift_radius):
            rospy.loginfo("----------------- ADRIFT STRATEGY -----------------")

        elif(self.radius <= self.repulsion_radius):
            rospy.loginfo("----------------- REPULSION STRATEGY -----------------")
            self.extract_safety_position()
            self.repulsion_strategy(self.x, self.y)
    
    def extract_safety_position(self):
        self.m =-(1/ (self.auv_position_east-self.asv_position_east)/(self.auv_position_north-self.asv_position_north))
        self.pointx = self.asv_position_north
        self.pointy = self.asv_position_east
        if (self.auv_position_north > self.asv_position_north):
            self.x = self.asv_position_north-1
        else:
            self.x = self.asv_position_north+1
        self.y = self.m*(self.x - self.pointy) + self.pointx 

    def repulsion_strategy(self, position_x, position_y):
        constant_linear_velocity = 4
        constant_angular_velocity = 2 
        linear_velocity = constant_linear_velocity
        alpha_ref = atan2(position_y,position_x)
        #obtain the minimum agle between both robots
        angle_error = atan2(sin(alpha_ref-self.asv_yaw), cos(alpha_ref-self.asv_yaw))
        self.angular_velocity = constant_angular_velocity * angle_error
        self.xr = linear_velocity*cos(angle_error)
        self.yr = linear_velocity*sin(angle_error)
        self.corrected_bvr_pusblisher(self.xr, self.yr,self.angular_velocity)


    def tracking_strategy(self):
        constant_linear_velocity = 7
        constant_angular_velocity = 2 

        if (self.radius<self.tracking_radius and self.radius>self.adrift_radius):
            self.velocity_adjustment = (self.radius-(self.adrift_radius))/(self.tracking_radius-(self.adrift_radius))
        else:
            self.velocity_adjustment = 1  
        linear_velocity = self.velocity_adjustment * constant_linear_velocity
        alpha_ref = atan2(self.y_distance,self.x_distance)
        angle_error = atan2(sin(alpha_ref-self.asv_yaw), cos(alpha_ref-self.asv_yaw))
        self.angular_velocity = constant_angular_velocity * angle_error
        self.xr = linear_velocity*cos(angle_error)
        self.yr = linear_velocity*sin(angle_error)
        if(self.xr < 0 ):
            self.xr = 0
        self.corrected_bvr_pusblisher(self.xr, self.yr,self.angular_velocity)
    
    def corrected_bvr_pusblisher(self,corrected_velocity_x,corrected_velocity_y,corrected_angular_z):
        # NEW PRIORITY DEFINITIONS
        # PRIORITY_TELEOPERATION_LOW = 0
        # PRIORITY_SAFETY_LOW = 5
        # PRIORITY_NORMAL = 10
        # PRIORITY_NORMAL_HIGH = 20
        # PRIORITY_TELEOPERATION = 40
        # PRIORITY_SAFETY = 45
        # PRIORITY_SAFETY_HIGH  = 50
        # PRIORITY_TELEOPERATION_HIGH = 60 
        bvr = BodyVelocityReq()
        bvr.header.frame_id    = '/robot'+str(self.robot_ID)+'/base_link'
        bvr.header.stamp       = rospy.Time.now()
        bvr.goal.requester     =  self.name
        bvr.disable_axis.x     = False
        bvr.disable_axis.y     = False
        bvr.disable_axis.z     = True
        bvr.disable_axis.roll  = True
        bvr.disable_axis.pitch = True
        bvr.disable_axis.yaw   = False
        bvr.twist.linear.x     = corrected_velocity_x
        bvr.twist.linear.y     = corrected_velocity_y
        bvr.twist.linear.z     = 0.0
        bvr.twist.angular.x    = 0.0
        bvr.twist.angular.y    = 0.0
        bvr.twist.angular.z    = corrected_angular_z
        bvr.goal.priority      = 20
        self.corrected_bvr.publish(bvr)
    
    def transit_to(self,pose):    
        self.disable_all_and_set_idle_srv()
        """Goto to position x, y, z, at velocity vel."""
        # // Define waypoint attributes
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = 1
        goto_req.position.x = pose[0]
        goto_req.position.y = pose[1]
        goto_req.position.z = 0.0
        goto_req.position_tolerance.x = 5
        goto_req.position_tolerance.y = 5
        goto_req.position_tolerance.z = 5
        goto_req.blocking = True
        goto_req.keep_position = False
        goto_req.disable_axis.x = False
        goto_req.disable_axis.y = True
        goto_req.disable_axis.z = False
        goto_req.disable_axis.roll = True
        goto_req.disable_axis.yaw = False
        goto_req.disable_axis.pitch = True
        goto_req.priority = 50
        goto_req.reference = 0 #REFERENCE_NED=0  REFERENCE_GLOBAL=1 REFERENCE_VEHICLE=2
        self.goto_srv(goto_req)

  
    def get_param(self, param_name, default = None):
        if rospy.has_param(param_name):
            param_value = rospy.get_param(param_name)
            return param_value
        elif default is not None:
            return default
        else:
            rospy.logfatal('[%s]: invalid parameters for %s in param server!', self.name, param_name)
            rospy.logfatal('[%s]: shutdown due to invalid config parameters!', self.name)
            exit(0)
   
if __name__ == '__main__':
    try:
        rospy.init_node('dustbin_robot')
        dustbin_robot = DustbinRobot(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

