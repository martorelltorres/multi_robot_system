#!/usr/bin/env python
import rospy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import datetime
from numpy import *
from shapely.geometry import Polygon,LineString,Point
import actionlib
from cola2_msgs.msg import WorldSectionActionResult
from std_srvs.srv import Empty
from geometry_msgs.msg import  PolygonStamped, Point32, Polygon
from cola2_msgs.msg import  NavSts
from multi_robot_system.msg import AvoidCollision


#import classes
from area_partition import area_partition
from task_allocator import task_allocation
from robot import robot
from collision_avoidance import CollisionAvoidance

class MultiRobotSystem:
    
    def __init__(self, name):
        """ Init the class """
        self.name = name
         # Get config parameters from the parameter server
        self.robot_ID = self.get_param('~robot_ID',0)   
        self.robot_name = self.get_param('~robot_name','turbot') 
        self.navigation_topic = self.get_param('~navigation_topic','/turbot/navigator/navigation') 
        self.section_result = self.get_param('~section_result','/turbot/pilot/world_section_req/result') 
        self.number_of_robots = self.get_param('number_of_robots')

        self.area_handler =  area_partition("area_partition")
        self.task_allocation_handler = task_allocation("task_allocation")
        self.robot_handler = robot("robot")
        self.collision_avoidance_handler = CollisionAvoidance("collision_avoidance_handler")

        self.system_initialization = True
        self.success_result = False
        self.data_gattered = False
        self.points = []

        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        #Subscribers
        rospy.Subscriber(self.section_result,
                         WorldSectionActionResult,    
                         self.update_section_result,
                         queue_size=1)

        rospy.Subscriber(self.navigation_topic ,
                         NavSts,    
                         self.update_robot_position,
                         queue_size=1)

        rospy.Subscriber('/collission_avoidance_info' ,
                         AvoidCollision,    
                         self.collision_avoidance,
                         queue_size=1)

        #Publishers
        self.polygon_pub = rospy.Publisher("voronoi_polygons",
                                        PolygonStamped,
                                        queue_size=1)

        self.polygon_offset_pub = rospy.Publisher("voronoi_offset_polygons",
                                        PolygonStamped,
                                        queue_size=1)

        # Init periodic timers
        rospy.Timer(rospy.Duration(1.0), self.print_polygon)
        rospy.Timer(rospy.Duration(1.0), self.print_offset_polygon)

        self.initialization()

    def update_section_result(self,msg):
        self.final_status = msg.result.final_status

    def update_robot_position(self,msg):
        self.robot_position_north = msg.position.north
        self.robot_position_east = msg.position.east
        self.robot_position_depth = msg.position.depth
        self.robot_orientation_yaw = msg.orientation.yaw 

    def collision_avoidance(self, msg):
        self.distance = msg.distance
        self.enable_section = msg.enable_section
        self.cancel_section = msg.cancel_section 
        self.stop_robot = msg.stop_robot
        self.robot_repulsion = msg.robot_repulsion

        # if(msg.enable_section == True):
        #     print("perform the coverage")  

        # elif(msg.cancel_section == True and msg.stop_robot==True):
        #     print("cancel section and stop the robot")
        #     self.robot_handler.cancel_section_strategy(self.robot_name)
        #     self.robot_handler.disable_thrusters(self.robot_name)

        # elif(msg.cancel_section == True and  msg.robot_repulsion == True):
        #     print("repulsion strategy")
        #     self.robot_handler.cancel_section_strategy(self.robot_name)
        #     self.collision_avoidance_handler.repulsion_strategy()
    
    
  
    def initialization(self): 
        # wait 4 seconds in order to initialize the different robot architectures
        rospy.sleep(4)
        if(self.system_initialization==True):
            self.system_initialization = False 
            self.goals,self.central_polygon = self.task_allocation_handler.task_allocation()
            # [['Robot_0', array([0, 1])], ['Robot_1', array([2, 3])]]
            self.goal_polygons = self.goals[self.robot_ID][1]
            print("The central polygon meeting point is the polygon: "+str(self.central_polygon))
            print("The robot_"+str(self.robot_ID)+" has the following goals: "+str(self.goal_polygons))
            times = self.area_handler.get_estimated_polygons_coverage_time()
            # print("--------------------------------------"+str(times)+"---------------------------------")
            self.goal_points = self.area_handler.define_path_coverage()
            self.robot_task_assignement()

    def robot_task_assignement(self):            
        for task in range(len(self.goal_polygons)):
            print("The robot_"+str(self.robot_ID)+" is covering the polygon: "+str(self.goal_polygons[task]))
            self.mrs_coverage(self.goal_polygons[task])

    def wait_until_section_reached(self):
        if(self.final_status==0):
            self.success_result = True    

    def mrs_coverage(self,goal):
        self.task_allocation_handler.update_task_status(self.robot_ID,goal,1,self.central_polygon)
        self.data_gattered = True
        section_points = self.goal_points[goal]
        print("-------------------------"+str(section_points)+"---------------------------------")
        self.section_id = goal

        for section in range(len(section_points)):
            self.task_allocation_handler.update_task_status(self.robot_ID,goal,2,self.central_polygon)
            current_section = section_points[section]
            if(self.section_id==goal):
                self.generate_initial_section(self.robot_position_north,self.robot_position_east,current_section)
                self.section_id = 100000 #TODO:find a better way     

            # Check the order of the initial and final points, set the initial point to the nearest point and the final to the furthest point 
            first_point = current_section[0]
            first_point_distance = self.robot_handler.get_robot_distance_to_point(self.robot_position_north,self.robot_position_east,first_point[0],first_point[1])
            second_point = current_section[1]
            second_point_distance = self.robot_handler.get_robot_distance_to_point(self.robot_position_north,self.robot_position_east,second_point[0],second_point[1])

            if(first_point_distance < second_point_distance):
                initial_point = first_point
                final_point = second_point
            else:
                initial_point = second_point
                final_point = first_point

            self.robot_handler.send_section_strategy(initial_point,final_point)

            self.wait_until_section_reached()

        self.task_allocation_handler.update_task_status(self.robot_ID,goal,3,self.central_polygon)
    
    def generate_initial_section(self,position_north,position_east,section_points):
        self.section_id = -1
        initial_point = Point(position_north,position_east)
        initial_point = list(initial_point.coords)[0]
        final_point = section_points[0]
        self.robot_handler.send_section_strategy(initial_point,final_point)
        self.wait_until_section_reached()

    def print_polygon(self,event):
        if(self.data_gattered==True):      
            points = []
            polygon_number = self.area_handler.get_polygon_number()
            polygon_number = polygon_number -1
            for polygon in range(polygon_number):
                polygon_coords_x,polygon_coords_y = self.area_handler.get_polygon_points(polygon)

                for coord in range(len(polygon_coords_x)):
                    polygon_points = Point32()
                    polygon_points.x = polygon_coords_x[coord]
                    polygon_points.y = polygon_coords_y[coord]
                    polygon_points.z = 0
                    points.append(polygon_points)

                    # create polygon message
                    polygon_msg = Polygon()
                    polygon_msg.points = points
                    
                    # create polygon_stamped message
                    polygon_stamped_msg = PolygonStamped()
                    polygon_stamped_msg.header.frame_id = "world_ned"
                    polygon_stamped_msg.header.stamp = rospy.Time.now()
                    polygon_stamped_msg.polygon = polygon_msg
                    self.polygon_pub.publish(polygon_stamped_msg)
        else:
            rospy.logwarn("Unable to print polygon, data not gattered yet")      

    def print_offset_polygon(self,event):
        if(self.data_gattered==True):      
            points = []
            polygon_number = self.area_handler.get_polygon_number()
            polygon_number = polygon_number -1
            for polygon in range(polygon_number):
                polygon_coords_x,polygon_coords_y = self.area_handler.get_offset_polygon_points(polygon)

                for coord in range(len(polygon_coords_x)):
                    polygon_points = Point32()
                    polygon_points.x = polygon_coords_x[coord]
                    polygon_points.y = polygon_coords_y[coord]
                    polygon_points.z = 0
                    points.append(polygon_points)

                    # create polygon message
                    polygon_msg = Polygon()
                    polygon_msg.points = points
                    
                    # create polygon_stamped message
                    polygon_stamped_msg = PolygonStamped()
                    polygon_stamped_msg.header.frame_id = "world_ned"
                    polygon_stamped_msg.header.stamp = rospy.Time.now()
                    polygon_stamped_msg.polygon = polygon_msg
                    self.polygon_offset_pub.publish(polygon_stamped_msg)
        else:
            rospy.logwarn("Unable to print polygon, data not gattered yet")  

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
        rospy.init_node('multi_robot_system')
        return_to_NED = MultiRobotSystem(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass