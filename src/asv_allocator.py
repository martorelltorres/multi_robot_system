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
import message_filters       
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray, Bool, Int16
from geometry_msgs.msg  import PointStamped
from cola2_msgs.msg import  NavSts,BodyVelocityReq
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from cola2_msgs.srv import Goto, GotoRequest
from multi_robot_system.msg import CoverageStartTime,AcousticData,AggregationModelInfo,TravelledDistance,BufferedData,ExplorationUpdate,TransmittedData,CommunicationLatency,Communication,Distances, Data
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Quaternion,PointStamped
import tf     
import os
import sys
import subprocess
 
class ASVAllocator:
    def __init__(self, name):
        """ Init the class """
        rospy.sleep(2)
        self.name = name
        node_name = rospy.get_name()

        # Get config parameters from the parameter server
        self.number_of_robots = self.get_param('number_of_robots')
        self.number_of_auvs = self.get_param('number_of_auvs')
        self.number_of_asvs = self.get_param('number_of_asvs')
        self.robot_ID = self.get_param('~robot_ID',0) 
        self.asv_ID = self.get_param('~asv_ID',0)
        self.tolerance = self.get_param('tolerance',2)
        self.surge_velocity = self.get_param('surge_velocity',0.5)
        self.section_action = self.get_param('section_action','/robot6/pilot/world_section_req') 
        self.section_result = self.get_param('section_result','/robot6/pilot/world_section_req/result') 
        self.pickle_path = self.get_param('pickle_path','/home/uib/MRS_ws/src/multi_robot_system/config/mission.pickle')
        self.repulsion_radius = self.get_param("repulsion_radius",20)
        self.adrift_radius = self.get_param("adrift_radius",30)
        self.tracking_radius = self.get_param("tracking_radius",50)
        # self.gamma = self.get_param("gamma",1)
        # self.test = self.get_param("test",0)
        self.n = self.get_param("n",1)
        self.aggregation_model = self.get_param("aggregation_model",1)
        self.w1 = self.get_param("w1",1)
        self.w2 = self.get_param("w2",1)
        self.w3 = self.get_param("w3",1)
        self.w4 = self.get_param("w4",1)
        self.alpha = self.get_param("alpha",1)
        self.beta = self.get_param("beta",1)
     
        
        # Initialize some variables
        self.acoustic_time = []
        self.current_time = []
        self.acquired_data = []
        self.first_acoustic_reception = True
        self.qlearning_init = False
        self.first=True
        self.pose = [0,0]
        self.data_transmited = []
        self.get_information = False
        self.start_to_publish = False
        self.robot_at_center = False
        self.robots_id = np.array([])
        self.OWA_inputs= np.array([])
        self.penalty = np.array([])
        self.system_init = False
        self.robot_data = [0,0]
        self.robots_information = []
        self.compare = []
        self.robots = []
        self.robot_initialization = np.array([])
        self.set_end_time = False
        self.start_dustbin_strategy =np.array([])
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
        self.number_of_stimulus = 2
        self.active_robots = self.number_of_auvs
        self.robot_to_remove = 999
        self.removed_robots= []
        self.bussy_auvs = np.array([999,999])
        # self.communication=True
        self.transmission_time = []
        self.communication_latency = []
        self.first_time = True
        self.travelled_distance = 0
        self.data_gather_time = []
        self.start_recording_time = []
        self.start_data_gathering = True
        self.asvs_positions= []
        self.asvs_init = np.array([])
        self.init=False
        self.elapsed_time =np.array([])
        self.asv_id = 100
        self.latency_data = np.zeros((self.number_of_asvs, self.number_of_auvs), dtype=int)
        self.transmited_data = np.zeros((self.number_of_asvs, self.number_of_auvs), dtype=int)
        self.owa=[]
        self.latency_init_data=[]

        self.zero_stimulus_variables = np.tile(np.zeros(0), (self.number_of_robots, 1))
        # -----------------------------------------------------------------

        for asv in range(self.number_of_asvs):
            self.asvs_init = np.append(self.asvs_init,False)
            self.asvs_positions.append([0,0,0])
        # Set the number of stimulus depending of the optimization strategy
        if(self.aggregation_model==1):
            self.number_of_stimulus=2
        else:
            self.number_of_stimulus=3

        self.read_area_info()

        # intialize the variables
        for robot in range(self.number_of_stimulus):
            self.senses.append(0)

        for i in range(self.active_robots):
            self.scaled_senses.append(self.senses)
            self.transmission_time.append(0)
        

        # initialize the robots variables
        for robot_ in range(self.number_of_auvs):
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
            self.elapsed_time = np.append(self.elapsed_time,0)
            self.transmission_init_time.append(0)    
            self.battery_charge.append(0)
            self.stimulus = np.append(self.stimulus,0)
            self.penalty = np.append(self.penalty,0)
            self.communication_latency.append(0)
            self.compare.append([0,0]) 
            self.robots_information.append([])
            self.owa.append(0)
            self.start_dustbin_strategy = np.append(self.start_dustbin_strategy,False)
            self.acoustic_time.append(0)
            self.current_time.append(0)
            self.acquired_data.append(0)
        
       
        robots_sense = np.zeros(2)
        self.stimulus_variables = np.tile(robots_sense, (self.number_of_auvs, 1))

        self.times = np.vstack((self.elapsed_time,self.elapsed_time))
        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        #Subscribers       
        for robot_agent in range(self.number_of_auvs):
            rospy.Subscriber('/robot'+str(robot_agent)+'/acoustic_communication',
                            PoseWithCovarianceStamped,
                            self.update_acoustic_info,
                            robot_agent,
                            queue_size=1)
        
        rospy.Subscriber('/robot4/navigator/navigation',
                        NavSts,    
                        self.update_asv_position,
                        self.asv_ID,
                        queue_size=1)
        
        for asv in range(self.number_of_asvs):
            rospy.Subscriber('/mrs/asv'+str(asv)+'_elapsed_time',
                            Float64MultiArray,    
                            self.update_elapsed_time,
                            asv,
                            queue_size=1)
            
            rospy.Subscriber('/mrs/asv'+str(asv)+'_data_buffered',
                        BufferedData,    
                        self.update_acquired_data,
                        asv,
                        queue_size=1)
            
            rospy.Subscriber('/mrs/asv'+str(asv)+'_communication_latency',
                        CommunicationLatency,    
                        self.update_communication_latency,
                        asv,
                        queue_size=1)
            
            rospy.Subscriber('/mrs/asv'+str(asv)+'_data_transmited',
                        TransmittedData,    
                        self.update_transmitted_data,
                        asv,
                        queue_size=1)

        #Publishers
        self.pub_priority_object = rospy.Publisher('priority_object_point', PointStamped, queue_size=2)

        self.pub_regular_object = rospy.Publisher('regular_object_point', PointStamped, queue_size=2)
        
        self.coverage_init_pub = rospy.Publisher('coverage_init',
                                            Bool,
                                            queue_size=1)
        
        self.buffered_data_pub = rospy.Publisher('allocator_data_buffered',
                                BufferedData,
                                queue_size=1)
        
        self.pub_tracking_control_asv0 = rospy.Publisher('/asv0/tracking',
                                        Bool,
                                        queue_size=2)
        
        self.pub_tracking_control_asv1 = rospy.Publisher('/asv1/tracking',
                                        Bool,
                                        queue_size=2)
        
        self.pub_elapsed_time = rospy.Publisher('allocator_elapsed_time',
                                                 Float64MultiArray,
                                                queue_size=2)
        
        self.communication_latency_pub = rospy.Publisher('allocator_communication_latency',
                                CommunicationLatency,
                                queue_size=2)
        
        self.data_transmited_pub = rospy.Publisher('allocator_data_transmited',
                                TransmittedData,
                                queue_size=1)

        self.aggregation_model_info_pub = rospy.Publisher('aggregation_model_info',
                        AggregationModelInfo,
                        queue_size=2)

        # Publish AggregationModelInfo 
        msg = AggregationModelInfo()
        msg.header.stamp = rospy.Time.now()
        msg.aggregation_model = self.aggregation_model
        msg.alpha = self.alpha
        msg.beta = self.beta
        msg.w1 = self.w1
        msg.w2 = self.w2
        msg.w3 = self.w3
        self.aggregation_model_info_pub.publish(msg)    
            
        
        # Init periodic timers self.distance
        rospy.Timer(rospy.Duration(0.1), self.plot_priority_objects)
        rospy.Timer(rospy.Duration(0.1), self.plot_regular_objects)

    def read_area_info(self):
        # Open the pickle file in binary mode
        with open(self.pickle_path, 'rb') as file:
            # Load the data from the file
            data = pickle.load(file)

        # Access different data from the loaded data
        self.cluster_centroids = data['array1']
        self.voronoi_polygons = data['array2']
        self.main_polygon = data['array3']
        self.main_polygon_centroid = data['array4']
        self.voronoi_offset_polygons = data['array5']
        self.regular_objects = data['array6']
        self.priority_objects = data['array7']
   
    #  --------------------- ELAPSED TIME---------------------------------
    def update_elapsed_time(self,msg,asv):
        self.times[asv] = msg.data
        # OJO self.sync_elapsed_time = np.minimum(self.times[0],self.times[1])
        self.sync_elapsed_time = self.times[asv]

        # Publish information
        msg = Float64MultiArray()
        msg.data = self.sync_elapsed_time
        self.pub_elapsed_time.publish(msg)

    #  --------------------- ACQUIRED DATA ---------------------------------
    def update_acquired_data(self, msg, asv_id):
        self.acquired_data = msg.data_stimulus
        regular = msg.buffered_regular_objects
        priority = msg.buffered_priority_objects
        storage = msg.storage
        # Publish information
        msg = BufferedData()
        msg.header.stamp = rospy.Time.now()
        msg.storage = storage
        msg.data_stimulus = self.acquired_data
        msg.buffered_regular_objects = regular
        msg.buffered_priority_objects = priority
        self.buffered_data_pub.publish(msg)
        self.update_stimulus_matrix()
        
        # enable tracking
        msg = Bool()
        msg.data = True
        self.pub_tracking_control_asv0.publish(msg)
      
    
    def update_transmitted_data(self, msg, asv_id):
        self.transmited_data[asv_id]= msg.transmitted_data
        # OJO data = np.maximum(self.transmited_data[0],self.transmited_data[1])
        data = self.transmited_data[0]
        regular= msg.transmitted_regular_objects
        priority=msg.transmitted_priority_objects

        # Publish information
        msg = TransmittedData()
        msg.header.stamp = rospy.Time.now()
        msg.transmitted_data = data
        msg.transmitted_regular_objects = regular
        msg.transmitted_priority_objects = priority
        self.data_transmited_pub.publish(msg)
    
    def update_communication_latency(self, msg, asv_id):
        self.latency_data[asv_id]= msg.comm_latency
        # OJO latency = np.minimum(self.latency_data[0],self.latency_data[1])
        latency = self.latency_data[0]

        # Publish information
        msg = CommunicationLatency()
        msg.header.stamp = rospy.Time.now()
        msg.comm_latency = latency
        self.communication_latency_pub.publish(msg)
    
    #  ----------------------- POSITION & COMMUNICATION SIGNAL -----------------------------------
    def update_acoustic_info(self, msg, robot_agent):
        self.current_time[robot_agent] = rospy.Time.now().nsecs

        if self.acoustic_time[robot_agent] != 0:
            time_diff = self.current_time[robot_agent] - self.acoustic_time[robot_agent]
            self.time_diff_normalized = self.normalize(time_diff, 0, 10, 1, 0)

        self.acoustic_time[robot_agent] = self.current_time[robot_agent]

        # print("Time diff robot" +str(robot_agent)+": "+str(self.time_diff_normalized))
        
        # tranform from quaternion to euler angles
        rpy = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # normalize the position covariance
        normalized_covariance = self.normalize(msg.pose.covariance[0],0,4.5,1,0)
        # ASVAllocator.penalty[robot_agent]= normalized_covariance*time_diff_normalized
        self.penalty[robot_agent]= normalized_covariance
        # fill the robots_information array with the robots information received from the PoseWithCovariance 
        self.robots_information[robot_agent] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, rpy[2], normalized_covariance]
        # check the system initialization
        if(self.system_init == False):
            self.initialization(robot_agent) 
    
    def get_communication_signal(self,asv_id,auv_id):
        distance = self.get_distance(asv_id,auv_id)
        # get RSSI communication signal  
        rssi = -47.537 -(0.368*distance) + (0.00132*distance**2) - (0.0000016*distance**3)
        normalized_value = (rssi +45) / (-85 + 45) 
        self.comm_signal[auv_id] = normalized_value
        return(normalized_value)
        
    def normalize(self,value, min_val, max_val, new_min, new_max):
        normalized_value = new_min + (value - min_val) * (new_max - new_min) / (max_val - min_val)
        return normalized_value
    
    def get_position(self, robot_id):
        return(self.robots_information[robot_id])

    def get_comm_signal(self, robot_id):
        return(self.comm_signal[robot_id])
    
    def get_distance(self, asv_id, auv_id):
        x_diff =  self.asvs_positions[asv_id][0] - self.robots_information[auv_id][0]
        y_diff =  self.asvs_positions[asv_id][1] - self.robots_information[auv_id][1] 
        distance =  sqrt(x_diff**2 + y_diff**2 )
        return(distance)

    def update_asv_position(self, msg, asv):
   
        north = float(msg.position.north)
        east = float(msg.position.east)
        yaw = float(msg.orientation.yaw)

        if asv not in self.asvs_positions:
            self.asvs_positions[asv] = [0.0, 0.0, 0.0]  # Initialize if not already present
        
        self.asvs_positions[asv] = [north, east, yaw]
        self.asvs_init[asv] = True

        if(np.all(self.asvs_init) == True):
            self.init=True

    def update_stimulus_matrix(self):
        # get the information from the AUVs and create the matrix stimulus
        for asv in range(self.number_of_asvs):
            for auv in range(self.active_robots):
                # obtain the amount of data to be transferred
                self.stimulus_variables[auv][0] = self.acquired_data[auv]
                
                # get the distance
                distance = self.get_distance(asv,auv)
                self.stimulus_variables[auv][1] = distance

                # set the stimulus to 0 if there are no data to transmit
                if(self.stimulus_variables[auv][0] == 0):
                    self.stimulus_variables[auv]=[0,0]

            # check if there are data to transmit, if not stop the tracking
            if np.all(self.stimulus_variables == 0):
                # send the order to stop the tracking process
                msg = Bool()
                msg.data = False
                self.pub_tracking_control_asv0.publish(msg)
            else:
                # normalize the stimulus values
                normalized_values = self.min_max_scale(self.stimulus_variables)

                print("NORMALIZED VALUES")
                print(normalized_values)

            if(self.aggregation_model==1):
                self.ARTM(normalized_values)
            elif(self.aggregation_model==2):
                self.OWA()

    def get_goal_AUV(self):
        return(self.robot_goal_id)
           
    def ARTM(self,normalized_values):
        for robot in range(self.active_robots):
            self.get_communication_signal(0,robot)
            scaled_values = normalized_values[robot]
            s = self.alpha*scaled_values[0]+self.beta*scaled_values[1]
            self.stimulus[robot] = s**self.n/(s**self.n + self.comm_signal[robot]**self.n)
        
        # extract the sorted goal robot IDs in descending order
        sorted_goal_ids = np.array([])
        sorted_goal_ids = np.argsort(self.stimulus)[::-1] 
        self.robot_goal_id = sorted_goal_ids[0]
       
    # --------------------------------------------------------------------------------------
    def min_max_scale(self,values):

        self.normalized_values = values
        self.min_value = np.min(values)
        self.max_value = np.max(values)

        data_column = self.normalized_values[:, 0]
        self.normalized_values[:, 0] = (data_column - self.min_value) / (self.max_value - self.min_value)
        self.normalized_values[:, 0] = np.where(data_column == 0, 0, self.normalized_values[:, 0])

        distance_column = self.normalized_values[:, 1]
        self.normalized_values[:, 1] = (self.max_value - distance_column) / (self.max_value - self.min_value)
        self.normalized_values[:, 1] = np.where(data_column == 0, 0, self.normalized_values[:, 1])

        return(self.normalized_values)
    
    def set_dustbin_robots(self,msg):
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
                if(self.aggregation_model==1):
                    self.stimulus_variables[self.removed_robots[element]] = [0,0] #number of stimulus
                    self.normalized_values[self.removed_robots[element]] = [0,0] 
                elif(self.aggregation_model==2):
                    self.stimulus_variables[self.removed_robots[element]] = [0,0]
                    self.normalized_values[self.removed_robots[element]] = [0,0]
            self.remove_robot=False  
        
    def initialization(self,robot_id):
        if(self.robots_information[robot_id][0] != 0):
            self.robot_initialization[robot_id] = True
        
        if((self.robot_initialization == True).all()):
            self.system_init = True
            
    def plot_regular_objects(self,event):
            for element in range(len(self.regular_objects)):
                regular_object = PointStamped()
                regular_object.header.stamp = rospy.Time.now()
                regular_object.header.frame_id = "world_ned"
                regular_object.point.x = self.regular_objects[element].x
                regular_object.point.y = self.regular_objects[element].y
                regular_object.point.z = 30
                # Publish
                self.pub_regular_object.publish(regular_object)

    def plot_priority_objects(self,event):
            for element in range(len(self.priority_objects)):
                priority_object = PointStamped()
                priority_object.header.stamp = rospy.Time.now()
                priority_object.header.frame_id = "world_ned"
                priority_object.point.x = self.priority_objects[element].x
                priority_object.point.y = self.priority_objects[element].y
                priority_object.point.z = 30
                # Publish
                self.pub_priority_object.publish(priority_object)
    
    def OWA(self):
        self.number_of_stimulus = 3 #data distance and RSSI 
        OWA_inputs = self.min_max_scale(self.stimulus_variables)
        comm = np.array([])
        OWA_inputs_out = np.array([])

        #Iteram a traves de la sortida de min_max_scale per afegir l'estimul de RSSI
        for element in range(self.number_of_auvs):
            signal = self.get_communication_signal(0,element)
            comm=np.append(comm,signal)

        OWA_inputs_out = np.column_stack((OWA_inputs,comm))

        for i in range(len(OWA_inputs)):
            if np.all(OWA_inputs[i] == 0):
                OWA_inputs_out[i, 2] = 0

        # get the weights and set the values for w1,w2,w3,w4 where w1>=w2>=w3>=w4
        OWA_weights =np.array([self.w1,self.w2,self.w3])
        OWA_weights_sorted = np.sort(OWA_weights)
        self.w1 = OWA_weights_sorted[2]
        self.w2 = OWA_weights_sorted[1]
        self.w3 = OWA_weights_sorted[0]

        for robot in range(self.number_of_auvs):
            sorted_values = np.sort(OWA_inputs_out[robot]) 
            self.max_element = sorted_values[2]
            self.middle1 = sorted_values[1]
            self.min_element= sorted_values[0]
            self.owa[robot]= self.max_element*self.w1 + self.middle1*self.w2 + self.min_element*self.w3

        # print("The owas are: "+str(self.owa))
        max_owa = max(self.owa)
        self.robot_goal_id = self.owa.index(max_owa)
        # print("The maximum owa:"+str(max_owa)+" belongs to robot"+str(self.robot_goal_id))

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
        rospy.init_node('asv_allocator')
        asv_allocator = ASVAllocator(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass    