#!/usr/bin/env python
import rospy
import math
import numpy as np
import random 
import csv
import time
from math import *
import matplotlib
import pickle
import actionlib       
import matplotlib.pyplot as plt
from std_msgs.msg import Int16, Bool, Int16MultiArray,Float32MultiArray,Float32
from geometry_msgs.msg  import PointStamped
from cola2_msgs.msg import  NavSts,BodyVelocityReq
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from cola2_msgs.srv import Goto, GotoRequest
from multi_robot_system.msg import CoverageStartTime,AcousticData,TravelledDistance,BufferedData,ExplorationUpdate,TransmittedData,CommunicationLatency,ObjectInformation,Communication,Distances, Data
import os
import sys
import subprocess
#import classes
from area_partition import area_partition
 
class DustbinRobot:
  
    def __init__(self, name):
        """ Init the class """
        rospy.sleep(7)
        self.name = name
        node_name = rospy.get_name()

        # Get config parameters from the parameter server
        self.number_of_robots = self.get_param('number_of_robots')
        self.robot_ID = self.get_param('~robot_ID',0) 
        self.tolerance = self.get_param('tolerance',2)
        self.surge_velocity = self.get_param('surge_velocity',0.5)
        self.section_action = self.get_param('section_action','/robot4/pilot/world_section_req') 
        self.section_result = self.get_param('section_result','/robot4/pilot/world_section_req/result') 
        self.repulsion_radius = self.get_param("repulsion_radius",20)
        self.adrift_radius = self.get_param("adrift_radius",30)
        self.tracking_radius = self.get_param("tracking_radius",50)
        self.dutsbin_timer = self.get_param("dutsbin_timer",1)

        self.alpha = self.get_param("alpha",1)
        self.beta = self.get_param("beta",1)
        self.gamma = self.get_param("gamma",1)
        self.n = self.get_param("n",1)
        self.optimization_model = self.get_param("optimization_model",1)

        self.w1 = self.get_param("w1",1)
        self.w2 = self.get_param("w2",1)
        self.w3 = self.get_param("w3",1)
        self.w4 = self.get_param("w4",1)

        self.optimization_strategy = self.get_param("optimization_strategy",1)
        self.area_handler =  area_partition("area_partition")

        # Initialize some variables
        self.qlearning_init = False
        self.pose = [0,0]
        self.a=True
        self.data_transmited = []
        self.get_information = False
        self.start_to_publish = False
        self.robot_at_center = False
        self.set_transmission_init_time=False
        self.robots_id = np.array([])
        self.OWA_inputs= np.array([])
        self.elapsed_time = []
        self.system_init = False
        self.robot_data = [0,0]
        self.auvs_info = [[],[],[],[],[],[]]
        self.robots = []
        self.robot_initialization = np.array([])
        self.enable_tracking = False
        self.set_end_time = False
        self.start_dustbin_strategy =np.array([False,False,False,False,False,False])
        self.exploration_tasks_update = np.array([])
        self.battery_charge= []
        self.distance = []
        self.transmission_init_time = []
        self.time_init = []
        self.storage_disk =[]
        self.max_value = 1
        self.min_value = 0
        self.comm_signal = []
        self.stimulus = np.array([])
        self.robots_sense = np.array([])
        self.scaled_senses = []
        self.explorer_robots = []
        self.senses = []
        self.number_of_stimulus = 4
        self.active_robots = self.number_of_robots
        self.robot_to_remove = 999
        self.removed_robots= []
        # self.communication=True
        self.transmission_time = []
        self.communication_latency = []
        self.first_time = True
        self.travelled_distance = 0
        self.data_gather_time = []
        self.start_recording_time = []
        self.start_data_gathering = True

        # Set the number of stimulus depending of the optimization strategy
        if(self.optimization_model==1):
            self.number_of_stimulus=3
        else:
            self.number_of_stimulus=4

        self.read_area_info()
        # intialize the variables
        for robot in range(self.number_of_stimulus):
            self.senses.append(0)

        for i in range(self.active_robots):
            self.scaled_senses.append(self.senses)
            self.transmission_time.append(0)

        # initialize the robots variables
        for robot_ in range(self.number_of_robots):
            self.exploration_tasks_update = np.append(self.exploration_tasks_update,False)
            self.robot_initialization = np.append(self.robot_initialization,False) 
            self.robots.append(robot_)  
            self.robots_id = np.append(self.robots_id,robot_)
            self.robots_id = self.robots_id.astype(int) #convert float to int type
            self.comm_signal.append(0)
            self.storage_disk.append(0)
            self.data_transmited.append(0)
            self.time_init.append(0)
            self.distance.append(0)
            self.start_recording_time.append(0)
            self.data_gather_time.append(0)
            self.elapsed_time.append(0)
            self.transmission_init_time.append(0)    
            self.battery_charge.append(0)
            self.stimulus = np.append(self.stimulus,0)
            self.communication_latency.append(0)
        
        for stimulus in range(self.number_of_stimulus):
            self.robots_sense = np.append(self.robots_sense,0)
    
        self.stimulus_variables= np.vstack((self.robots_sense,self.robots_sense,self.robots_sense,self.robots_sense,self.robots_sense,self.robots_sense))

        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        for auv_agent in range(self.number_of_robots):
            rospy.Subscriber('/robot'+str(auv_agent)+'/navigator/navigation',
                            NavSts,
                            self.update_auv_info,
                            auv_agent,
                            queue_size=1)
        
        for robot_id in range(self.number_of_robots):
            rospy.Subscriber("/mrs/robot"+str(robot_id)+"_object_info",
                                ObjectInformation,
                                self.update_object_information,
                                robot_id,
                                queue_size=1)

        rospy.Subscriber('/robot'+str(self.robot_ID)+'/navigator/navigation',
                            NavSts,    
                            self.update_asv_position,
                            queue_size=1)
        
        rospy.Subscriber('/mrs/exploration_area_update',
                            ExplorationUpdate,    
                            self.kill_the_process,
                            queue_size=1)
               
        rospy.Subscriber('/mrs/exploration_finished',
                            Int16,    
                            self.remove_robot_from_dustbin_goals,
                            queue_size=1)
            
        for element in range(self.number_of_robots):
            rospy.Subscriber(
            '/mrs/robot'+str(element)+'_start_coverage_time',
            CoverageStartTime,
            self.set_coverage_start_time,
            element,
            queue_size=1)      

        #Publishers
        self.corrected_bvr = rospy.Publisher('/robot6/controller/body_velocity_req',
                                                BodyVelocityReq,
                                                queue_size=1)
        
        self.pub_object = rospy.Publisher('object_point', PointStamped, queue_size=2)

        self.markerPub_repulsion = rospy.Publisher('repulsion_radius',
                                                    Marker,
                                                    queue_size=1)

        self.markerPub_adrift = rospy.Publisher('adrift_radius',
                                                Marker,
                                                queue_size=1)

        self.markerPub_tracking = rospy.Publisher('tracking_radius',
                                                Marker,
                                                queue_size=1)
        
        self.markerPub_object = rospy.Publisher('object',
                                                Marker,
                                                queue_size=1)
        
        self.coverage_init_pub = rospy.Publisher('coverage_init',
                                            Bool,
                                            queue_size=1)
               
        self.reset_storage_pub = rospy.Publisher('reset_storage_disk',
                                        Int16,
                                        queue_size=1)
      
        # ---------------------------------------------------------------------------
        self.goal_id_pub = rospy.Publisher('goal_id',
                                        Data,
                                        queue_size=1)
        
        self.buffered_data_pub = rospy.Publisher('data_buffered',
                                BufferedData,
                                queue_size=1)
        
        self.data_transmited_pub = rospy.Publisher('data_transmited',
                                TransmittedData,
                                queue_size=1)
               
        self.communication_latency_pub = rospy.Publisher('communication_latency',
                                CommunicationLatency,
                                queue_size=2)
        
        self.travelled_distance_pub = rospy.Publisher('asv_travelled_distance',
                                TravelledDistance,
                                queue_size=2)
        
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
        
        # Init periodic timers self.distance
        rospy.Timer(rospy.Duration(1.0), self.update_travelled_distance)
        
    def update_auv_info(self,msg, auv_agent):
        self.auvs_info[auv_agent] = [msg.position.north, msg.position.east, msg.position.depth, msg.orientation.yaw]
        # check the system initialization
        if(self.system_init == False):
            self.initialization(auv_agent)

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
        self.random_points = data['array6']
    
    def update_object_information(self,msg,robot_id):
        # See https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10244660 for more details.
        self.storage_disk[robot_id] = self.storage_disk[robot_id] + 70
        print("The robot "+str(robot_id)+ " storage disk is: "+str(self.storage_disk[robot_id]))
        # set the time when the AUV detects an object
        self.data_gather_time[robot_id]= rospy.Time.now().secs
        # Publish the stored data
        msg = BufferedData()
        msg.header.stamp = rospy.Time.now()
        msg.storage = self.storage_disk
        self.buffered_data_pub.publish(msg)
        # Start the data gathering when the first robot detects an object
        if(self.start_data_gathering == True):
            self.start_data_gathering = False
            self.get_goal_id()
    
    def update_travelled_distance(self,event):
        if (self.first_time == True):
            self.x_old_position = 0
            self.y_old_position = 0
            self.x_current_position = self.asv_north_position
            self.y_current_position = self.asv_east_position
            self.update_distance()
            self.first_time = False
        else:
            self.x_old_position = self.x_current_position
            self.y_old_position = self.y_current_position
            self.x_current_position = self.asv_north_position
            self.y_current_position = self.asv_east_position
            self.update_distance()
        
        # publish the data
        msg = TravelledDistance()
        msg.header.stamp = rospy.Time.now()
        msg.travelled_distance = self.travelled_distance 
        self.travelled_distance_pub.publish(msg)
    
    def update_distance(self):
        x_diff =  self.x_current_position - self.x_old_position
        y_diff =  self.y_current_position - self.y_old_position 
        distance =  sqrt(x_diff**2 + y_diff**2)
        self.travelled_distance = self.travelled_distance + distance
 
    def update_asv_position(self, msg):
        self.asv_north_position = msg.position.north
        self.asv_east_position = msg.position.east      
        self.asv_yaw = msg.orientation.yaw
        self.asv_init = True

        if (self.robot_at_center == False and self.asv_init == True):
            self.transit_to(self.main_polygon_centroid)
            self.robot_at_center = True

        if(self.enable_tracking == True):
            self.tracking()

    def set_coverage_start_time(self,msg,element):
        print("Setting coverage start time for robot"+str(element))
        self.set_elapsed_time(element)
        self.start_dustbin_strategy[element]= True
        self.t_start = msg.time.secs
        self.time_robot_id = msg.robot_id
        self.time_init[element] = msg.time.secs
        self.start_recording_time[self.time_robot_id] = self.t_start 
        if (np.all(self.start_dustbin_strategy) == True):
            msg = Bool()
            msg.data = True 
            self.coverage_init_pub.publish(msg)
        else:
            msg = Bool()
            msg.data = False 
            self.coverage_init_pub.publish(msg)
                      
    def set_elapsed_time(self,robot_id):
        self.start_recording_time[robot_id] = rospy.Time.now().secs
                 
    def get_distance(self, robot_id):
        x_diff =  self.asv_north_position - self.auvs_info[robot_id][0]
        y_diff =  self.asv_east_position - self.auvs_info[robot_id][1] 
        distance =  sqrt(x_diff**2 + y_diff**2)
        return(distance)
   
    def get_elapsed_time(self,robot_id):
        if(self.start_dustbin_strategy[robot_id]== False):
            time = 0
            self.elapsed_time[robot_id]= time
        else:
            time = rospy.Time.now().secs - self.start_recording_time[robot_id]
            self.elapsed_time[robot_id]= time
        return(time)

    def scale_value(self, value):
        scaled_value =(value- self.min_value)/(self.max_value-self.min_value)
        return(scaled_value)
          
    def get_stimulus(self):
        for robot in range(self.active_robots):
            # get the time elapsed between AUV visits
            self.robots_sense[0] = self.get_elapsed_time(self.robots_id[robot])
            # get the distance between ASV-AUV's
            distance = self.get_distance(self.robots_id[robot])
            self.robots_sense[1] = distance
            # obtain the amount of data to be transferred
            self.robots_sense[2] = self.storage_disk[self.robots_id[robot]]
            #get the communication signal based on RSSI
            if(self.optimization_model==2):
                self.robots_sense[3] = self.comm_signal[self.robots_id[robot]]
            
            print("**** Robot "+str(self.robots_id[robot])+"stimulus ****")
            print("Elapsed time:" +str(self.robots_sense[0]))
            print("Distance: "+str(self.robots_sense[1]))
            print("Storage disk: "+str(self.robots_sense[2]))
            print("Communication signal: "+str(self.comm_signal[self.robots_id[robot]]))

            self.stimulus_variables[self.robots_id[robot]] = self.robots_sense

        # print(".................. STIMULUS VARIABLES ..................")
        # print(self.stimulus_variables)
        
        # normalize the values
        self.min_max_scaled = self.min_max_scale(self.stimulus_variables)
        # print(".................. SCALED STIMULUS VARIABLES ..................")
        # print(self.min_max_scaled)
           
        # remove the robots that have completed their work
        for element in range(len(self.removed_robots)):
            self.stimulus[self.removed_robots[element]] = 0

        return(self.stimulus)

    def min_max_scale(self,values):
        # get the minimum and maximum values
        self.min_value = np.min(values)
        self.max_value = np.max(values)
        for robot in range(self.active_robots):
            scaled_values = np.array([])
            for value in range(self.number_of_stimulus):
                if(value==1): #distance stimulus
                    calc =(values[robot][value]- self.min_value)/(self.max_value-self.min_value)
                    dist = abs(calc-1)
                    scaled_values = np.append(scaled_values,dist)
                else:
                    calc =(values[robot][value]- self.min_value)/(self.max_value-self.min_value)
                    scaled_values = np.append(scaled_values,calc)
            self.scaled_senses[robot] = scaled_values
        return(self.scaled_senses)

    def get_goal_id(self):
        self.get_stimulus()
        self.ARTM()


        print("The resulting AUV goal ID is: "+str(self.robot_goal_id))
        # this flag launches the tracking strategy
        self.enable_tracking = True

        # publish the goal_id
        msg = Data()
        msg.header.stamp = rospy.Time.now()
        msg.data = self.robot_goal_id
        self.goal_id_pub.publish(msg)
        # set the flag to 
        self.set_transmission_init_time=True
       
        
    def ARTM(self):
        for robot in range(self.active_robots):
            scaled_values = self.min_max_scaled[robot]
            s = self.alpha*abs(scaled_values[0])+ self.beta*abs(scaled_values[1])+ self.gamma* abs(scaled_values[2])
            self.stimulus[robot] = s**self.n/(s**self.n + self.comm_signal[robot]**self.n)
        # extract the goal robot ID
        self.robot_goal_id = self.stimulus.argmax()  
        # print(".................. ARTM PROBABILITY ..................")
        # print(self.stimulus)

    def round_robin(self):
        self.robots_id = np.roll(self.robots_id,1)
        self.robot_goal_id = self.robots_id[0]  
    
    def normalize(self,value, min_val, max_val, new_min, new_max):
        normalized_value = new_min + (value - min_val) * (new_max - new_min) / (max_val - min_val)
        return normalized_value
    
    def update_acoustic_info(self, msg, robot_agent):
        distance = self.get_distance(robot_agent)
        # set RSSI communication signal 
        rssi = -47.537 -(0.368*distance) + (0.00132*distance**2) - (0.0000016*distance**3)
        normalized_value = self.normalize(rssi, -85, -40, 0, 1)
        self.comm_signal[robot_agent] = abs(normalized_value -1) # lower threshold values fixes lower Pf
 
    def kill_the_process(self,msg):
        # update area explored
        self.exploration_tasks_update[msg.explored_sub_area] = True
        # print("__________TASK UPDATE__________")
        # print(self.exploration_tasks_update)
        # check if the area is fully explored 
        if all(self.exploration_tasks_update):
            print("____________________The target area is totally explored____________________")
            list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
            list_output = list_cmd.stdout.read()
            retcode = list_cmd.wait()
            assert retcode == 0, "List command returned %d" % retcode
            for str in list_output.split("\n"):
                if (str.startswith('/record_')):
                    os.system("rosnode kill " + str)

    def remove_robot_from_dustbin_goals(self,msg):
        # remove the robot from the dustbin goals
        robot_id = msg.data
        self.robots_id = np.delete(self.robots_id, np.where(self.robots_id == robot_id))
        self.robot_to_remove = robot_id
        self.remove_robot=True
        self.removed_robots.append(robot_id)
        self.active_robots = self.active_robots -1
        # print("Robot "+str(robot_id)+" removed from the team")
        # set at minimum value the robots that have completed their work 
        if(self.robot_to_remove!=999 and self.remove_robot==True):
            for element in range(len(self.removed_robots)):
                if(self.optimization_model==1):
                    self.stimulus_variables[self.removed_robots[element]] = [0,0,0]
                    self.min_max_scaled[self.removed_robots[element]] = [0,0,0]
                elif(self.optimization_model==2):
                    self.stimulus_variables[self.removed_robots[element]] = [0,0,0,0]
                    self.min_max_scaled[self.removed_robots[element]] = [0,0,0,0]
            self.remove_robot=False

        self.check_dustbin_robot()
    
    def check_dustbin_robot(self):
        if(np.size(self.robots_id)==0):
            self.enable_tracking = False
            if(self.asv_init == True):
                self.goto_central_area()
                  
    def goto_central_area(self):
        self.transit_to(self.main_polygon_centroid)
        self.robot_at_center = True

    def initialization(self,robot_id):
        if(self.auvs_info[robot_id][0] != 0):
            self.robot_initialization[robot_id] = True
        
        if((self.robot_initialization == True).all()):
            self.system_init = True
            self.print_random_points()
    
    def print_random_points(self):
        while not rospy.is_shutdown():
            for element in range(len(self.random_points)):
                object = PointStamped()
                object.header.stamp = rospy.Time.now()
                object.header.frame_id = "world_ned"
                object.point.x = self.random_points[element].x
                object.point.y = self.random_points[element].y
                object.point.z = 0
                # Publish
                self.pub_object.publish(object)

    def get_lateral_position(self):
        yaw_rad = math.radians(self.auv_yaw)

        # Obtain lateral positiona at predefined distance
        lateral_pos_1_x = self.auv_position_north + self.adrift_radius * math.cos(yaw_rad + math.pi / 2)
        lateral_pos_1_y = self.auv_position_east + self.adrift_radius * math.sin(yaw_rad + math.pi / 2)
        lateral_pos_2_x = self.auv_position_north + self.adrift_radius * math.cos(yaw_rad - math.pi / 2)
        lateral_pos_2_y = self.auv_position_east + self.adrift_radius * math.sin(yaw_rad - math.pi / 2)

        # find the closest position
        dist_1 = self.get_euclidean_distance(self.asv_position_north,self.asv_position_east,lateral_pos_1_x,lateral_pos_1_y)
        dist_2 = self.get_euclidean_distance(self.asv_position_north,self.asv_position_east,lateral_pos_2_x,lateral_pos_2_y)
        
        if dist_1 < dist_2:
            return lateral_pos_1_x, lateral_pos_1_y
        else:
            return lateral_pos_2_x, lateral_pos_2_y

    def get_euclidean_distance(self,p1_x,p1_y,p2_x,p2_y):
        x_distance = p1_x - p2_x
        y_distance = p1_y - p2_y
        distance = np.sqrt(x_distance**2 + y_distance**2)
        return(distance)
    
    def tracking(self):
        self.auv_position_north = self.auvs_info[self.robot_goal_id][0]
        self.asv_position_north = self.asv_north_position
        self.auv_position_east = self.auvs_info[self.robot_goal_id][1]
        self.asv_position_east = self.asv_east_position
        self.auv_yaw = self.auvs_info[self.robot_goal_id][3]

        if(self.a==True):
            print(self.auv_position_north)
            print(self.auv_position_east)
            self.a=False

        self.tracking_marker()
        self.adrift_marker()
        self.repulsion_marker()

        # distance ASV-AUV
        self.x_distance = self.auv_position_north-self.asv_position_north
        self.y_distance = self.auv_position_east-self.asv_position_east

        # distance ASV - 90 lateral AUV distance
        position_x, position_y = self.get_lateral_position()
        self.x_lateral_distance = position_x-self.asv_position_north
        self.y_lateral_distance = position_y-self.asv_position_east

        self.radius = sqrt((self.x_distance)**2 + (self.y_distance)**2)
        self.initialized = True

        # Tracking area
        if(self.radius > self.adrift_radius):
            self.tracking_strategy()

        # Communication area
        if(self.radius < (self.adrift_radius+5)):
            if(self.set_transmission_init_time==True):
                self.transmission_init_time [self.robot_goal_id] = rospy.Time.now().secs
                self.set_transmission_init_time=False

            # print("Time: "+str(rospy.Time.now().secs-self.transmission_init_time[self.robot_goal_id])+ " waiting time: "+str(self.storage_disk[self.robot_goal_id]))
            if(self.storage_disk[self.robot_goal_id]==0):
                self.get_goal_id()

            elif((rospy.Time.now().secs-self.transmission_init_time[self.robot_goal_id]) > self.storage_disk[self.robot_goal_id]):
                # publish the amount of transmited data
                msg = TransmittedData()
                msg.header.stamp = rospy.Time.now()
                self.data_transmited[self.robot_goal_id] = self.data_transmited[self.robot_goal_id] + self.storage_disk[self.robot_goal_id]
                msg.transmitted_data = self.data_transmited
                self.data_transmited_pub.publish(msg)
                self.communicate()

        # Repulsion area
        if(self.radius <= self.repulsion_radius):
            self.extract_safety_position()
            self.repulsion_strategy(self.x_lateral_distance, self.y_lateral_distance)

    def communicate(self):
        # reset the storage values
        self.storage_disk[self.robot_goal_id] = 0
        reset_id = self.robot_goal_id
        msg = Int16()
        msg.data = reset_id 
        self.reset_storage_pub.publish(msg)

        # publish the communication latency
        self.communication=True
        if(self.data_gather_time[self.robot_goal_id]==0):
            self.communication_latency[self.robot_goal_id] = 0
        else:
            self.communication_latency[self.robot_goal_id] = (rospy.Time.now().secs - self.data_gather_time[self.robot_goal_id])

        msg = CommunicationLatency()
        msg.header.stamp = rospy.Time.now()
        msg.comm_delay = self.communication_latency 
        self.communication_latency_pub.publish(msg)

        # When the data transmission ends reset the elapsed_time
        self.set_elapsed_time(self.robot_goal_id)
        self.get_goal_id()
        
        # self.dustbin_strategy()
    
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
        constant_linear_velocity = 3
        constant_angular_velocity = 2 

        if (self.radius<self.tracking_radius and self.radius>self.adrift_radius):
            self.velocity_adjustment = (self.radius-(self.adrift_radius))/(self.tracking_radius-(self.adrift_radius))
        else:
            self.velocity_adjustment = 1  

        linear_velocity = self.velocity_adjustment * constant_linear_velocity
        alpha_ref = atan2(self.y_lateral_distance,self.x_lateral_distance)
        angle_error = atan2(sin(alpha_ref-self.asv_yaw), cos(alpha_ref-self.asv_yaw))
        self.angular_velocity = constant_angular_velocity * angle_error
        self.xr = linear_velocity*cos(angle_error)
        self.yr = linear_velocity*sin(angle_error)
        if(self.xr < 0 ):
            self.xr = 0
        self.corrected_bvr_pusblisher(self.xr, self.yr,self.angular_velocity)
    
    def corrected_bvr_pusblisher(self,corrected_velocity_x,corrected_velocity_y,corrected_angular_z):
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
        bvr.goal.priority      = 60
        self.corrected_bvr.publish(bvr)
    
    def transit_to(self,pose):    
        self.disable_all_and_set_idle_srv()
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = 1
        goto_req.position.x = pose.x
        goto_req.position.y = pose.y
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
        goto_req.priority = 20
        goto_req.reference = 0 
        self.goto_srv(goto_req)

    def adrift_marker(self):
        self.anchor = Marker()
        self.anchor.header.frame_id = "world_ned"
        self.anchor.header.stamp = rospy.Time.now()
        self.anchor.ns = "mrs"
        self.anchor.id = 0
        self.anchor.type = Marker.CYLINDER
        self.anchor.action = Marker.ADD
        self.anchor.pose.position.x = self.auv_position_north
        self.anchor.pose.position.y = self.auv_position_east
        self.anchor.pose.position.z = -0.2
        self.anchor.pose.orientation.x = 0
        self.anchor.pose.orientation.y = 0
        self.anchor.pose.orientation.z = 0
        self.anchor.pose.orientation.w = 1.0
        self.anchor.scale.x = self.adrift_radius*2
        self.anchor.scale.y = self.adrift_radius*2
        self.anchor.scale.z = 0.1
        self.anchor.color.r = 0.0
        self.anchor.color.g = 1.0
        self.anchor.color.b = 0.0
        self.anchor.color.a = 1
        self.markerPub_adrift.publish(self.anchor)

    def repulsion_marker(self):
        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = "world_ned"
        self.robotMarker.header.stamp = rospy.Time.now()
        self.robotMarker.ns = "mrs"
        self.robotMarker.id = 0
        self.robotMarker.type = Marker.CYLINDER
        self.robotMarker.action = Marker.ADD
        self.robotMarker.pose.position.x = self.auv_position_north
        self.robotMarker.pose.position.y = self.auv_position_east
        self.robotMarker.pose.position.z = -0.3
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
        self.robotMarker.color.a = 1
        self.markerPub_repulsion.publish(self.robotMarker)

    def tracking_marker(self):
        self.follow = Marker()
        self.follow.header.frame_id = "world_ned"
        self.follow.header.stamp = rospy.Time.now()
        self.follow.ns = "mrs"
        self.follow.id = 0
        self.follow.type = Marker.CYLINDER
        self.follow.action = Marker.ADD
        self.follow.pose.position.x = self.auv_position_north
        self.follow.pose.position.y = self.auv_position_east
        self.follow.pose.position.z = -0.1
        self.follow.pose.orientation.x = 0
        self.follow.pose.orientation.y = 0
        self.follow.pose.orientation.z = 0
        self.follow.pose.orientation.w = 1.0
        self.follow.scale.x = self.tracking_radius*2
        self.follow.scale.y = self.tracking_radius*2
        self.follow.scale.z = 0.1
        self.follow.color.r = 0.0
        self.follow.color.g = 0.0
        self.follow.color.b = 1.0
        self.follow.color.a = 1
        self.markerPub_tracking.publish(self.follow)

    def object_point(self,x,y):
        self.object = Marker()
        self.object.header.frame_id = "world_ned"
        self.object.header.stamp = rospy.Time.now()
        self.object.ns = "mrs"
        self.object.id = 0
        self.object.type = Marker.CYLINDER
        self.object.action = Marker.ADD
        self.object.pose.position.x = x
        self.object.pose.position.y = y
        self.object.pose.position.z = -20
        self.object.pose.orientation.x = 0
        self.object.pose.orientation.y = 0
        self.object.pose.orientation.z = 0
        self.object.pose.orientation.w = 1.0
        self.object.scale.x = 2
        self.object.scale.y = 2
        self.object.scale.z = 0.1
        self.object.color.r = 0.0
        self.object.color.g = 0.0
        self.object.color.b = 1.0
        self.object.color.a = 1
        self.markerPub_object.publish(self.object)

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