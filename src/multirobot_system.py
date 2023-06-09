#!/usr/bin/env python
import rospy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import datetime
from numpy import *
import numpy as np
import os
import subprocess
import pickle
from shapely.geometry import Polygon,LineString,Point
import actionlib
from cola2_msgs.msg import WorldSectionActionResult
from std_srvs.srv import Empty
from geometry_msgs.msg import  PolygonStamped, Point32, Polygon
from cola2_msgs.msg import  NavSts
from multi_robot_system.msg import AvoidCollision, CoverageStartTime, ExplorationUpdate
from std_msgs.msg import Int16, Float64

#import classes
from area_partition import area_partition
from task_allocator import task_allocation
from robot import Robot
class MultiRobotSystem:
    
    def __init__(self, name):
        """ Init the class """
        self.name = name
         # Get config parameters from the parameter server
        self.robot_ID = self.get_param('~robot_ID')   
        self.section_result = self.get_param('~section_result') 
        self.number_of_robots = self.get_param('number_of_robots')
        self.actual_sections = []
        self.actual_section = 0
        self.area_handler =  area_partition("area_partition")
        self.task_allocation_handler = task_allocation("task_allocation")
        self.robot_handler = Robot("robot")

        self.send_folowing_section = False
        self.points = []
        self.simulation_task_times = []
        self.task_monitoring = []
        self.section_cancelled = False
        self.final_status = 99999
        self.system_init = False
        self.robot_initialization = np.array([])
        self.start = True
        self.first_section_points = []
        self.robots_information =[]
        self.robot_data = [0,0]

         # initialize the robots variables
        for i in range(self.number_of_robots):
            self.robot_initialization = np.append(self.robot_initialization,False) # self.robot_initialization = [False,False;False]
            self.actual_sections.append([i,0])
            self.robots_information.append (self.robot_data)
        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        #Subscribers
        rospy.Subscriber(self.section_result,
                         WorldSectionActionResult,    
                         self.update_section_result,
                         queue_size=1)
        
        rospy.Subscriber(
            '/robot'+str(self.robot_ID)+'/navigator/navigation',
            NavSts,
            self.update_robot_position) 

        #Publishers
        self.polygon_pub = rospy.Publisher("voronoi_polygons",
                                        PolygonStamped,
                                        queue_size=1)

        self.polygon_offset_pub = rospy.Publisher("voronoi_offset_polygons",
                                        PolygonStamped,
                                        queue_size=1)

        self.exploration_update_pub = rospy.Publisher("exploration_area_update",
                                        ExplorationUpdate,
                                        queue_size=1)
        self.start_coverage_time = rospy.Publisher("robot"+str(self.robot_ID)+"_start_coverage_time",
                                        CoverageStartTime,
                                        queue_size=1)
        
        self.exploration_end_pub = rospy.Publisher("exploration_finished",
                                Int16,
                                queue_size=1)
        self.read_area_info()        
        rospy.Timer(rospy.Duration(1), self.print_polygon)

        # rospy.Timer(rospy.Duration(1.0), self.print_offset_polygon)

        self.initialization()
    
    def update_robot_position(self, msg):
        # fill the robots_information array with the robots information received from the NavSts 
        self.robot_position_north = msg.position.north
        self.robot_position_east = msg.position.east

    def update_section_result(self,msg):
        self.final_status = msg.result.final_status
    
    def read_area_info(self):
        # Open the pickle file in binary mode
        with open('/home/uib/area_partition_data.pickle', 'rb') as file:
            # Load the data from the file
            data = pickle.load(file)

        # Access different data from the loaded data
        self.cluster_centroids = data['array1']
        self.voronoi_polygons = data['array2']
        self.main_polygon = data['array3']
        self.main_polygon_centroid = data['array4']
        self.voronoi_offset_polygons = data['array5']

    def initialization(self): 
        # wait 7 seconds in order to initialize the different robot architectures
        rospy.sleep(7)
        if np.all(self.robot_initialization == False):
            for robot in range(self.number_of_robots):
                self.robot_initialization[robot] = self.robot_handler.is_robot_alive(robot)
        
        

        print("             *************************")
        print("                 ROBOT "+str(self.robot_ID)+ " INITIALIZED   ")
        print("             *************************")

        self.goals = self.task_allocation_handler.task_allocation()
        for task in range(len(self.goals)):
            self.task_monitoring.append(0)
        self.goal_polygons = self.goals[self.robot_ID][1]
        self.goal_points = self.area_handler.define_path_coverage()
        self.robot_sections = self.goal_points[self.robot_ID]
        # print( "The goals for robot"+str(self.robot_ID)+" are: "+str(self.goal_points[self.robot_ID]))
        self.coverage()
    
    def coverage(self):
        # get the closest point of the largest polygon side
        point1, point2 = self.area_handler.find_largest_side_distance(self.voronoi_polygons[self.robot_ID])
        dist_p1 = self.robot_handler.get_robot_distance_to_point(self.robot_position_north, self.robot_position_east,point1[0],point1[1])
        dist_p2 = self.robot_handler.get_robot_distance_to_point(self.robot_position_north, self.robot_position_east,point2[0],point2[1])
        
        if(dist_p1 > dist_p2):
            goal_point = point2
        else: 
            goal_point = point1
        # send the robot to start exploration point
        self.robot_handler.send_section_strategy((self.robot_position_north,self.robot_position_east),goal_point,self.robot_ID)
        self.wait_until_section_reached()
        # start the area exploration coverage
        print( "The robot"+str(self.robot_ID)+" started the exploration of area"+str(self.goals[self.robot_ID][1]))
        # advise the time when the robot starts the coverage
        msg = CoverageStartTime()
        msg.time = rospy.Time.now()
        msg.robot_id = self.robot_ID
        self.start_coverage_time.publish(msg)

        for section in range(len(self.robot_sections)):
            # get points from sections
            point_a = self.robot_sections[section][0]           
            point_a_0 = point_a[0]
            point_a_1 = point_a[1]
            point_b = self.robot_sections[section][1]
            point_b_0 = point_b[0]
            point_b_1 = point_b[1]
            p1_dist = self.robot_handler.get_robot_distance_to_point(self.robot_position_north, self.robot_position_east,point_a_0,point_a_1)
            p2_dist = self.robot_handler.get_robot_distance_to_point(self.robot_position_north, self.robot_position_east,point_b_0,point_b_1)
            
            if(p1_dist > p2_dist):
                initial_point = [point_b_0,point_b_1]
                final_point = [point_a_0,point_a_1]
            else: 
                initial_point = [point_a_0,point_a_1]
                final_point = [point_b_0,point_b_1]
           
            # send the robot to start the area exploration
            initial_task_time = rospy.Time.now()
            self.robot_handler.send_section_strategy(initial_point,final_point,self.robot_ID)
            self.wait_until_section_reached()

        final_task_time = rospy.Time.now()
        self.task_time = self.robot_handler.simulation_task_time(initial_task_time,final_task_time)

        # advise the robot_id of the robot that finishes ALL the tasks
        msg = Int16()
        msg.data = self.robot_ID 
        self.exploration_end_pub.publish(msg)

        # move the robot to the surface
        self.robot_handler.send_goto_strategy(self.robot_position_north,self.robot_position_east,True)
  
    def wait_until_section_reached(self):
        if(self.final_status==0):
            self.actual_sections[self.robot_ID][1] = self.actual_sections[self.robot_ID][1]+1
            self.actual_section = self.actual_sections[self.robot_ID][1]
            self.send_folowing_section = True
        else: 
            self.send_folowing_section = False
  
    def print_polygon(self,event):
        points = []
        for polygon in range(self.number_of_robots):
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

    def print_offset_polygon(self,event):
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