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
import actionlib
from cola2_lib.utils.ned import NED
from shapely.geometry import Polygon,Point
from cola2_msgs.msg import WorldSectionActionResult
from std_srvs.srv import Empty
from geometry_msgs.msg import  PolygonStamped, Point32, Polygon
from cola2_msgs.msg import  NavSts
from multi_robot_system.msg import AvoidCollision, CoverageStartTime, ExplorationUpdate
from std_msgs.msg import Int16, Bool,Float32MultiArray,Float32,Int16MultiArray
from sea_cucumber_detection.msg import holo_detections

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
        self.ned_origin_lat = self.get_param('ned_origin_lat')
        self.ned_origin_lon = self.get_param('ned_origin_lon')
        self.ned = NED(self.ned_origin_lat, self.ned_origin_lon, 0.0)
        self.actual_sections = []
        self.actual_section = 0
        self.coverage_started=False
        self.avoid = False
        self.area_handler =  area_partition("area_partition")
        self.task_allocation_handler = task_allocation("task_allocation")
        self.robot_handler = Robot("robot")
        self.executing_dense_mission = False
        self.send_folowing_section = False
        self.points = []
        self.goal_section_point = [0,0]
        self.simulation_task_times = []
        self.task_monitoring = []
        self.first_detection=True
        self.section_cancelled = False
        self.final_status = 99999
        self.system_init = False
        self.robot_initialization = np.array([])
        self.explored_objects_index = np.array([])
        self.start = True
        self.first_section_points = []
        self.robots_information =[]
        self.robot_data = [0,0]
        self.object_exploration = False

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

        rospy.Subscriber(
            '/robot'+str(self.robot_ID)+'/holo_detections',
            holo_detections,
            self.holo_detected) 
        
        rospy.Subscriber(
            '/mrs/coverage_init',
            Bool,
            self.object_exploration_flag) 
        
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

        self.initialization()
    
    def holo_detected(self, msg):
        self.image = msg.image_rect_color
        self.holo_lat = msg.lat
        self.holo_lon = msg.lon
        self.num_detections = msg.num_detections
        self.holo_north, self.holo_east, depth = self.ned.geodetic2ned([self.holo_lat, self.holo_lon, 0.0])
        if(self.coverage_started==True):
            self.object_detection()
    
    def update_objects(self,msg):
        print("Updating object_points array")
        self.random_points = msg.data
    
    def object_exploration_flag(self,msg):
        self.object_exploration = msg.data
    
    def object_detection(self):
        self.holo_point = Point(self.holo_north,self.holo_east)

        if(self.first_detection==False):
            self.avoid_double_detections(self.holo_point, self.old_holo_point)

        # check if the object is in the AUV assigned sub-area
        if(self.voronoi_polygons[self.robot_ID].contains(self.holo_point) and self.avoid==False):
            print("Robot "+str(self.robot_ID)+ " has been detecting an holoturia at: "+str(self.holo_point))
            self.old_holo_point = self.holo_point
            self.first_detection=False
    
    def avoid_double_detections(self,point_a,point_b):
        distance = point_a.distance(point_b)
        if(distance<=1.2):
            self.avoid=True
            print("removed!")
        else:
            self.avoid=False

    def update_robot_position(self, msg):
        self.robot_position_north = msg.position.north
        self.robot_position_east = msg.position.east

    def execute_dense_mission(self,x,y,robot_id):
        from shapely.geometry import Polygon
        width = 10
        length = 10

        # Calcular las coordenadas de los vertices del rectangulo
        half_width = width / 2
        half_length = length / 2

        # Coordenadas de los vertices del rectangulo
        vertices = [(x - half_width, y - half_length),
                    (x + half_width, y - half_length),
                    (x + half_width, y + half_length),
                    (x - half_width, y + half_length)]

        # Crear el poligono del area a cubrir
        area_polygon = Polygon(vertices)

        # Pasos para las pasadas paralelas (2.5 m de distancia entre ellas)
        step_size = 2.5

        # Obtener limites de la coordenada x para las pasadas
        x_min, x_max = area_polygon.bounds[0], area_polygon.bounds[2]

        # Variable para alternar entre las direcciones de las pasadas
        self.reverse = False

        # Planificar pasadas horizontales paralelas
        y_current = area_polygon.bounds[1] if not self.reverse else area_polygon.bounds[3]
        while area_polygon.bounds[1] <= y_current <= area_polygon.bounds[3]:
            # Obtener limites de la coordenada y para la pasada actual
            x_start, x_end = (x_max, x_min) if self.reverse else (x_min, x_max)

            # Planificar pasada horizontal
            point_a = [x_start,y_current]
            point_b = [x_end,y_current]

            # Actualizar coordenada y para la proxima pasada
            y_current += step_size

            # Cambiar la direccion de las pasadas
            self.reverse = not self.reverse
            self.robot_handler.send_slow_section_strategy(point_a,point_b,robot_id)

    def update_section_result(self,msg):
        if (self.executing_dense_mission == False):
            self.final_status = msg.result.final_status
        else:
            self.final_status = 8888
    
    def return_to_exploration_path(self):
        print(str(self.robot_ID)+ ": is returning to exploration path!!!!!")
        point_a1 = [self.robot_position_north,self.robot_position_east]
        point_b1 = self.goal_section_point
        self.robot_handler.send_slow_section_strategy(point_a1,point_b1,self.robot_ID)
    
    def read_area_info(self):
        # Open the pickle file in binary mode
        with open('/home/tintin/area_partition_data.pickle', 'rb') as file:
            # Load the data from the file
            data = pickle.load(file)

        # Access different data from the loaded data
        self.cluster_centroids = data['array1']
        self.voronoi_polygons = data['array2']
        self.main_polygon = data['array3']
        self.main_polygon_centroid = data['array4']
        self.voronoi_offset_polygons = data['array5']
        self.random_points = data['array6']

    def initialization(self): 
        # wait 7 seconds in order to initialize the different robot architectures
        rospy.sleep(5)
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
            final_point = point2
        else: 
            final_point = point1

        self.goal_section_point = final_point
        self.robot_handler.send_section_strategy((self.robot_position_north,self.robot_position_east),final_point,self.robot_ID)
        self.wait_until_section_reached()
        # start the area exploration coverage
        print( "The robot"+str(self.robot_ID)+" started the exploration of area"+str(self.goals[self.robot_ID][1]))
        # advise the time when the robot starts the coverage
        self.coverage_started=True
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
            
            self.goal_section_point = final_point  
           
            # send the robot to start the area exploration
            initial_task_time = rospy.Time.now()
            self.restart_exploration_point = final_point
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

        if(self.final_status==0 and self.executing_dense_mission==False):
            self.actual_sections[self.robot_ID][1] = self.actual_sections[self.robot_ID][1]+1
            self.actual_section = self.actual_sections[self.robot_ID][1]
            self.send_folowing_section = True

        elif(self.final_status!=0 and self.executing_dense_mission==True): 
            self.send_folowing_section = False
            self.return_to_exploration_path()
            self.executing_dense_mission = False
  
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