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

        self.success_result = False
        self.data_gattered = False
        self.points = []
        self.simulation_task_times = []
        self.task_monitoring = []
        self.section_cancelled = False
        self.final_status = 99999
        self.system_init = False
        self.robot_initialization = np.array([])
        self.start = True
        self.first_section_points = []

         # initialize the robots variables
        for i in range(self.number_of_robots):
            self.robot_initialization = np.append(self.robot_initialization,False) # self.robot_initialization = [False,False;False]
            self.actual_sections.append([i,0])
        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        #Subscribers
        rospy.Subscriber(self.section_result,
                         WorldSectionActionResult,    
                         self.update_section_result,
                         queue_size=1)

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
        
        # Init periodic timers
        rospy.Timer(rospy.Duration(1.0), self.print_polygon)
        rospy.Timer(rospy.Duration(1.0), self.print_offset_polygon)

        self.initialization()

    def update_section_result(self,msg):
        self.final_status = msg.result.final_status


    def initialization(self): 
        # wait 7 seconds in order to initialize the different robot architectures
        rospy.sleep(7)      
        if np.all(self.robot_initialization == False):
            for robot in range(self.number_of_robots):
                self.robot_initialization[robot] = self.robot_handler.is_robot_alive(robot)

        print("             *************************")
        print("                 ROBOT "+str(self.robot_ID)+ " INITIALIZED   ")
        print("             *************************")

        self.goals,self.central_polygon = self.task_allocation_handler.task_allocation()
        # init the task_time variable
        for task in range(len(self.goals)):
            self.task_monitoring.append(0)

        self.goal_polygons = self.goals[self.robot_ID][1]
        # print("The central polygon meeting point is the polygon: "+str(self.central_polygon))
        print("The robot_"+str(self.robot_ID)+" has the following goals: "+str(self.goal_polygons))
        # times = self.area_handler.get_estimated_polygons_coverage_time()
        self.goal_points = self.area_handler.define_path_coverage()
        self.coverage()

    def coverage(self):          
        for task in range(len(self.goal_polygons)):
            print("The robot_"+str(self.robot_ID)+" is covering the polygon: "+str(self.goal_polygons[task]))
            # get the points of the largest polygon side
            voronoi_polygons = self.area_handler.get_voronoi_polygons()
            self.point1, self.point2 = self.area_handler.find_largest_side_distance(voronoi_polygons[self.goal_polygons[task]])
            self.mrs_coverage(self.goal_polygons[task])

            # task_time = self.robot_handler.simulation_task_time(initial_task_time,final_task_time)
            # print(".......................................")
            # print("The spended time is "+ str(task_time)+ " seconds")
            # self.simulation_task_times[task] = self.task_time
            # self.task_monitoring[task]= True

            # advise the robot_id of the robot that finishes the task
            msg = ExplorationUpdate()
            msg.header.frame_id = "exploration_area"
            msg.header.stamp = rospy.Time.now()
            msg.robot_id = self.robot_ID 
            msg.explored_sub_area = self.goal_polygons[task]
            self.exploration_update_pub.publish(msg)
        
        # advise the robot_id of the robot that finishes ALL the tasks
        msg = Int16()
        msg.data = self.robot_ID 
        self.exploration_end_pub.publish(msg)

        # move the robot to the surface
        position_north,position_east,position_depth,orientation_yaw = self.robot_handler.get_robot_position(self.robot_ID)
        self.robot_handler.send_goto_strategy(position_north,position_east,True)
  
    def wait_until_section_reached(self):
        if(self.final_status==0):
            self.actual_sections[self.robot_ID][1] = self.actual_sections[self.robot_ID][1]+1
            self.actual_section = self.actual_sections[self.robot_ID][1]
            self.success_result = True    
    
    def mrs_coverage(self,goal):
        self.task_allocation_handler.update_task_status(self.robot_ID,goal,1,self.central_polygon,self.goal_polygons,self.actual_section)
        self.data_gattered = True
        section_points = self.goal_points[goal]
        

        for section in range(len(section_points)):
            self.task_allocation_handler.update_task_status(self.robot_ID,goal,2,self.central_polygon,self.goal_polygons,self.actual_section)
            self.robot_position_north,self.robot_position_east,self.robot_position_depth,self.robot_orientation_yaw = self.robot_handler.get_robot_position(self.robot_ID)
            self.current_section = section_points[section]
            # print("************current_section********************* ")
            # print(self.current_section)
            
            if(self.start == True):
                # send the first section to move the robot from the origin to the start exploration point
                # print("************** initial section **********************")
                # self.generate_initial_section(self.robot_position_north,self.robot_position_east,self.current_section)
                self.start = False

                
                # send the robot to start the area exploration
                self.robot_handler.send_section_strategy((0,0),self.point1,self.robot_ID)
                self.wait_until_section_reached()
                # advise the time when the robot starts the coverage
                msg = CoverageStartTime()
                msg.time = rospy.Time.now()
                msg.robot_id = self.robot_ID
                self.start_coverage_time.publish(msg)
                # send the first section over the polygon side in order to cover the hole exploration area
                self.robot_handler.send_section_strategy(self.point1,self.point2,self.robot_ID)
                self.wait_until_section_reached()


            # update_current_section = self.robot_handler.set_current_section(self,self.actual_section)
            # print("--------------------")
            # print(update_current_section)

            if(section >= 1 and self.current_section):
                self.current_section = section_points[section]
                # Check the order of the initial and final points, set the initial point to the nearest point and the final to the furthest point 
                first_point = self.current_section[0]
                first_point_distance = self.robot_handler.get_robot_distance_to_point(self.robot_position_north,self.robot_position_east,first_point[0],first_point[1])
                second_point = self.current_section[1]
                second_point_distance = self.robot_handler.get_robot_distance_to_point(self.robot_position_north,self.robot_position_east,second_point[0],second_point[1])

                if(first_point_distance < second_point_distance):
                    initial_point = first_point
                    final_point = second_point
                else:
                    initial_point = second_point
                    final_point = first_point
                

                initial_task_time = rospy.Time.now()
                self.robot_handler.send_section_strategy(initial_point,final_point,self.robot_ID)

                self.wait_until_section_reached()
            
        final_task_time = rospy.Time.now()
        self.task_time = self.robot_handler.simulation_task_time(initial_task_time,final_task_time)
        # print(".......................................")
        # print("The spended time is "+ str(self.task_time)+ " seconds")
        # self.simulation_task_times[task] = task_time
        # self.task_monitoring[task]= True

        self.task_allocation_handler.update_task_status(self.robot_ID,goal,3,self.central_polygon,self.goal_polygons,self.actual_section)
    
    def generate_initial_section(self,position_north,position_east,section_points):
        initial_point = Point(position_north,position_east)
        initial_point = list(initial_point.coords)[0]
        final_point = section_points[0]
        self.robot_handler.send_section_strategy(initial_point,final_point,self.robot_ID)
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
        # else:
        #     rospy.logwarn("Unable to print polygon, data not gattered yet")  

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