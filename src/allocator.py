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
from std_msgs.msg import Int16, Int8, Bool
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
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
import pickle


class allocator:
    local_points=[]
    local_coords=[]
    polygon_coords_x =0
    polygon_coords_y= 0
    voronoi_polygons= []
    first_time = False
    centroid_points = []
    cluster_centroids = []
    voronoi_polygons = []

    def __init__(self, name):
        self.name = name
        # Get config parameters from the parameter server
        self.number_of_robots = self.get_param('number_of_robots')
        self.robot_ID = self.get_param('~robot_ID',0) 
        self.asv_ID = self.get_param('~asv_ID',0)
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
        # Initialize some variables
        self.qlearning_init = False
        self.pose = [0,0]
        self.data_transmited = []
        self.get_information = False
        self.start_to_publish = False
        self.robot_at_center = False
        self.robots_id = np.array([])
        self.OWA_inputs= np.array([])
        self.elapsed_time = []
        self.system_init = False
        self.robot_data = [0,0]
        self.robots_information = [[],[],[],[],[],[]]
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
        self.number_of_asvs= 2
        self.asvs_positions= [[],[]]

        # initialize the robots variables
        for robot_ in range(self.number_of_robots):
            self.exploration_tasks_update = np.append(self.exploration_tasks_update,False)
            self.robot_initialization = np.append(self.robot_initialization,False) # self.robot_initialization = [False,False;False]
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


        # subscribers
        rospy.Subscriber('/mrs/exploration_finished',
                    Int16,    
                    self.remove_robot_from_dustbin_goals,
                    queue_size=1)
        
        for robot_id in range(self.number_of_robots):
            rospy.Subscriber("/mrs/robot"+str(robot_id)+"_object_info",
                                ObjectInformation,
                                self.update_object_information,
                                robot_id,
                                queue_size=1)


    def remove_robot_from_dustbin_goals(self,msg):
        # remove the robot from the dustbin goals
        robot_id = msg.data
        self.robots_id = np.delete(self.robots_id, np.where(self.robots_id == robot_id))
        self.robot_to_remove = robot_id
        self.remove_robot=True
        self.removed_robots.append(robot_id)
        self.active_robots = self.active_robots -1
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

    def update_object_information(self,msg,robot_id):
        # See https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10244660 for more details.
        self.storage_disk[robot_id] = self.storage_disk[robot_id] + 70
        # print("The robot "+str(robot_id)+ " storage disk is: "+str(self.storage_disk[robot_id]))
        # set the time when the AUV detects an object
        self.data_gather_time[robot_id]= rospy.Time.now().secs
        # Publish the stored data
        msg = BufferedData()
        msg.header.stamp = rospy.Time.now()
        msg.storage = self.storage_disk
        self.buffered_data_pub.publish(msg)
        # # Start the data gathering when the first robot detects an object
        # if(self.start_data_gathering == True):
        #     self.start_data_gathering = False
        #     self.get_goal_id()

    def get_goal_id(self):
        self.get_stimulus()
        if(self.asv_ID==0):
            if(self.optimization_model==1):
                self.ARTM()

            elif(self.optimization_model==2):
                self.OWA()
                
            elif(self.optimization_model==3):
                self.use_max_stimulus()
                
            elif(self.optimization_model==4):
                self.round_robin()

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

            self.stimulus_variables[self.robots_id[robot]] = self.robots_sense

        # print(".................. STIMULUS VARIABLES ..................")
        # print(self.stimulus_variables)
        
        self.min_max_scaled = self.min_max_scale(self.stimulus_variables)
        # print(".................. SCALED STIMULUS VARIABLES ..................")
        # print(self.min_max_scaled)
        # print("Communication signal: "+str(self.comm_signal))
           
        # remove the robots that have completed their work
        for element in range(len(self.removed_robots)):
            self.stimulus[self.removed_robots[element]] = 0
        return(self.stimulus)
    
    def get_distance(self, robot_id):
        x_diff =  self.asvs_positions[self.asv_ID][0] - self.robots_information[robot_id][0]
        y_diff =  self.asvs_positions[self.asv_ID][1] - self.robots_information[robot_id][1] 
        depth = self.robots_information[robot_id][2]
        distance =  sqrt(x_diff**2 + y_diff**2 + depth**2)
        return(distance)
    
    def min_max_scale(self,values):
        # get the minimum and maximum values
        self.min_value = np.min(values)
        self.max_value = np.max(values)
        for robot in range(self.active_robots):
            scaled_values = np.array([])
            for value in range(self.number_of_stimulus):
                calc =(values[robot][value]- self.min_value)/(self.max_value-self.min_value)
                scaled_values = np.append(scaled_values,calc)
            self.scaled_senses[robot] = scaled_values
        return(self.scaled_senses)
    
    def get_euclidean_distance(self,p1_x,p1_y,p2_x,p2_y):
        x_distance = p1_x - p2_x
        y_distance = p1_y - p2_y
        distance = np.sqrt(x_distance**2 + y_distance**2)
        return(distance)
    
    def ARTM(self):
        for robot in range(self.active_robots):
            scaled_values = self.min_max_scaled[robot]
            s = self.alpha*abs(scaled_values[0])+ self.beta*abs(scaled_values[1])+ self.gamma* abs(scaled_values[2])
            self.stimulus[robot] = s**self.n/(s**self.n + self.comm_signal[robot]**self.n)
        
        if(self.asv_ID==0):
            print("*************EXTRACTING GOAL ID*************")
            # extract the goal robot ID
            sorted_stimulus_index = np.argsort(-self.stimulus) #- in order to sort in decending order
            selected_auv = sorted_stimulus_index[0]
        
            distance1 = self.get_euclidean_distance(self.asvs_positions[0][0],self.asvs_positions[0][1],self.robots_information[selected_auv][0],self.robots_information[selected_auv][1])
            distance2 = self.get_euclidean_distance(self.asvs_positions[1][0],self.asvs_positions[1][1],self.robots_information[selected_auv][0],self.robots_information[selected_auv][1])
            
            if(distance1<distance2):
                self.robot_goal_id = selected_auv
                # send the decission to the other ASV
                msg = Int8()
                msg.data = sorted_stimulus_index[1]
                self.decision_pub.publish(msg)
            else:
                self.robot_goal_id = selected_auv
                # send the decission to the other ASV
                msg = Int8()
                msg.data = sorted_stimulus_index[1]
                self.decision_pub.publish(msg)
   
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

    def set_elapsed_time(self,robot_id):
        self.start_recording_time[robot_id] = rospy.Time.now().secs

if __name__ == '__main__':
    try:
        rospy.init_node('allocator')
        allocator = allocator(rospy.get_name())
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass