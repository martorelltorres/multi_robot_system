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
from std_msgs.msg import Int16MultiArray, Bool, Int16
from geometry_msgs.msg  import PointStamped
from cola2_msgs.msg import  NavSts,BodyVelocityReq
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from cola2_msgs.srv import Goto, GotoRequest
from multi_robot_system.msg import CoverageStartTime,AcousticData,TravelledDistance,BufferedData,ExplorationUpdate,TransmittedData,CommunicationLatency,ObjectInformation,Communication,Distances, Data
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Quaternion,PointStamped
import tf     
import os
import sys
import subprocess
 
class ASVAllocator:
  
    def __init__(self, name):
        """ Init the class """
        rospy.sleep(5)
        self.name = name
        node_name = rospy.get_name()

        # Get config parameters from the parameter server
        self.number_of_robots = self.get_param('number_of_robots')
        self.robot_ID = self.get_param('~robot_ID',0) 
        self.asv_ID = self.get_param('~asv_ID',0)
        self.tolerance = self.get_param('tolerance',2)
        self.surge_velocity = self.get_param('surge_velocity',0.5)
        self.section_action = self.get_param('section_action','/robot6/pilot/world_section_req') 
        self.section_result = self.get_param('section_result','/robot6/pilot/world_section_req/result') 
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

        # Initialize some variables
        self.qlearning_init = False
        self.first=True
        self.pose = [0,0]
        self.data_transmited = []
        self.get_information = False
        self.start_to_publish = False
        self.robot_at_center = False
        self.acquired_data = [0,0,0,0,0,0]
        self.robots_id = np.array([])
        self.OWA_inputs= np.array([])
        self.bussy_auvs = [None,None]
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
        self.asvs_positions= np.array([[0,0,0],[0,0,0]])
        self.asvs_init = np.array([])
        self.init=False
        self.elapsed_time =np.array([])
        self.auv_goal_ids = np.array([[0,0,0,0,0,0],[0,0,0,0,0,0]])
        self.asv_id = 100
        self.data = np.array([[0,0,0,0,0,0],[0,0,0,0,0,0]])
        self.latency_data = np.array([[0,0,0,0,0,0],[0,0,0,0,0,0]])
        self.transmited_data = np.array([[0,0,0,0,0,0],[0,0,0,0,0,0]])
        # -----------------------------------------------------------------

        for asv in range(self.number_of_asvs):
            self.asvs_init = np.append(self.asvs_init,False)
 
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
            self.communication_latency.append(0)
        
        for stimulus in range(4):
            self.robots_sense = np.append(self.robots_sense,0)

        self.stimulus_variables= np.vstack((self.robots_sense,self.robots_sense,self.robots_sense,self.robots_sense,self.robots_sense,self.robots_sense))
        self.times = np.vstack((self.elapsed_time,self.elapsed_time))
        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        #Subscribers       
        for robot_agent in range(self.number_of_robots):
            rospy.Subscriber('/robot'+str(robot_agent)+'/acoustic_communication',
                            PoseWithCovarianceStamped,
                            self.update_acoustic_info,
                            robot_agent,
                            queue_size=1)
        
        rospy.Subscriber('/robot6/navigator/navigation',
                        NavSts,    
                        self.update_asv_position,
                        0,
                        queue_size=1)
        
        rospy.Subscriber('/robot7/navigator/navigation',
                        NavSts,    
                        self.update_asv_position,
                        1,
                        queue_size=1)
            
        for asv in range(self.number_of_asvs):

            rospy.Subscriber('/mrs/asv'+str(asv)+'_elapsed_time',
                            Int16MultiArray,    
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
        self.pub_object = rospy.Publisher('object_point', PointStamped, queue_size=2)

        
        self.coverage_init_pub = rospy.Publisher('coverage_init',
                                            Bool,
                                            queue_size=1)
        
        self.buffered_data_pub = rospy.Publisher('allocator_data_buffered',
                                BufferedData,
                                queue_size=1)
        
        self.pub_elapsed_time = rospy.Publisher('allocator_elapsed_time',
                                                 Int16MultiArray,
                                                queue_size=2)
        
        self.communication_latency_pub = rospy.Publisher('allocator_communication_latency',
                                CommunicationLatency,
                                queue_size=2)
        
        self.data_transmited_pub = rospy.Publisher('allocator_data_transmited',
                                TransmittedData,
                                queue_size=1)

    def read_area_info(self):
        # Open the pickle file in binary mode
        with open('/home/uib/MRS_ws/src/MRS_stack/multi_robot_system/config/area_partition_data.pickle', 'rb') as file:
            # Load the data from the file
            data = pickle.load(file)

        # Access different data from the loaded data
        self.cluster_centroids = data['array1']
        self.voronoi_polygons = data['array2']
        self.main_polygon = data['array3']
        self.main_polygon_centroid = data['array4']
        self.voronoi_offset_polygons = data['array5']
        self.random_points = data['array6']
    
    #  --------------------- ELAPSED TIME---------------------------------
    def update_elapsed_time(self,msg,asv):
        self.times[asv] = msg.data
        self.sync_elapsed_time = np.minimum(self.times[0],self.times[1])

        # Publish information
        msg = Int16MultiArray()
        msg.data = self.sync_elapsed_time
        self.pub_elapsed_time.publish(msg)

    #  --------------------- ACQUIRED DATA ---------------------------------
    def update_acquired_data(self, msg, asv_id):
        self.data[asv_id] = msg.storage
        self.acquired_data = np.minimum(self.data[0],self.data[1])

        # Publish information
        msg = BufferedData()
        msg.header.stamp = rospy.Time.now()
        msg.storage = self.acquired_data
        self.buffered_data_pub.publish(msg)

        self.update_stimulus_matrix()
    
    def update_transmitted_data(self, msg, asv_id):
        self.transmited_data[asv_id]= msg.transmitted_data
        data = np.maximum(self.transmited_data[0],self.transmited_data[1])

        # Publish information
        msg = TransmittedData()
        msg.header.stamp = rospy.Time.now()
        msg.transmitted_data = data
        self.data_transmited_pub.publish(msg)
    
    def update_communication_latency(self, msg, asv_id):
        self.latency_data[asv_id]= msg.comm_delay
        latency = np.minimum(self.latency_data[0],self.latency_data[1])

        # Publish information
        msg = CommunicationLatency()
        msg.header.stamp = rospy.Time.now()
        msg.comm_delay = latency
        self.communication_latency_pub.publish(msg)
    
    #  ----------------------- POSITION & COMMUNICATION SIGNAL -----------------------------------
    def update_acoustic_info(self, msg, robot_agent):
        # tranform from quaternion to euler angles
        rpy = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # fill the robots_information array with the robots information received from the PoseWithCovariance 
        self.robots_information[robot_agent] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, rpy[2]]
        # check the system initialization
        if(self.system_init == False):
            self.initialization(robot_agent) 
    
    def get_communication_signal(self,asv_id,auv_id):
        distance = self.get_distance(asv_id,auv_id)
        # set RSSI communication signal 
        rssi = -47.537 -(0.368*distance) + (0.00132*distance**2) - (0.0000016*distance**3)
        normalized_value = self.normalize(rssi, -85, -40, 0, 1)
        self.comm_signal[auv_id] = abs(normalized_value-1)
        
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
        self.asvs_init[asv]=True
        self.asvs_positions[asv] = [msg.position.north,msg.position.east,msg.orientation.yaw]

        if(np.all(self.asvs_init) == True):
            self.init=True

    def update_stimulus_matrix(self):
        # get the information from the AUVs and create the matrix stimulus
        for asv in range(self.number_of_asvs):
            for robot in range(self.active_robots):
                # get the time elapsed between ASV visits
                self.robots_sense[0] = self.sync_elapsed_time[robot]
                # get the distance between ASV-AUV's
                distance = self.get_distance(asv,self.robots_id[robot])
                self.robots_sense[1] = distance
                # obtain the amount of data to be transferred
                self.robots_sense[2] = self.acquired_data[self.robots_id[robot]]
                #get the communication signal based on RSSI
                self.get_communication_signal(asv,self.robots_id[robot])
                self.robots_sense[3] = self.comm_signal[self.robots_id[robot]]

                self.stimulus_variables[self.robots_id[robot]] = self.robots_sense

            print("------- VALUES FOR ASV"+str(asv)+"--------")
            print(self.stimulus_variables)

            # normalize the stimulus values
            normalized_values = self.min_max_scale(self.stimulus_variables)
            # obtain the sorted goal id's for each ASV using ARTM
            sorted_ids = self.ARTM(normalized_values)
            self.auv_goal_ids[asv] = sorted_ids
        

        self.asv0_goals = self.auv_goal_ids[0]
        self.asv1_goals = self.auv_goal_ids[1]
        print("ASV0 goals are: "+str(self.asv0_goals))
        print("ASV1 goals are: "+str(self.asv1_goals))

        # if both ASV has as first goal the same goal AUV 
        if(self.asv0_goals[0] == self.asv1_goals[0] and self.first==True):
            # set the first AUV to the first ASV and the second AUV to the second ASV
            self.set_goal_id(0,self.asv0_goals[0])
            self.set_goal_id(1,self.asv1_goals[1])
            self.first=False

        # if ASVs has different goal AUV
        elif(self.asv0_goals[0] != self.asv1_goals[0] and self.first==True):
            self.set_goal_id(0,self.asv0_goals[0])
            self.set_goal_id(1,self.asv0_goals[0])
            self.first=False

        # if goal auv are equal
        elif(self.asv0_goals[0] == self.asv1_goals[0] and (self.asv0_goals[0] not in self.bussy_auvs) and (self.asv1_goals[1] not in self.bussy_auvs and self.first_time==False)):
            self.set_goal_id(0,self.asv0_goals[0])
            self.set_goal_id(1,self.asv1_goals[1])

        # if goal auv are different
        elif(self.asv0_goals[0] != self.asv1_goals[0] and(self.asv0_goals[0] not in self.bussy_auvs) and (self.asv1_goals[0] not in self.bussy_auvs and self.first_time==False)):
            self.set_goal_id(0,self.asv0_goals[0])
            self.set_goal_id(1,self.asv0_goals[0])
    
    def set_bussy_auvs(self,auv_id, asv_id):
        # This method handles the assigneds AUVs in order to avoid duplicities in the assignation process. 
        # The same AUV can't be assigned to both ASVs
        self.bussy_auvs[asv_id] = auv_id

    def set_goal_id(self,asv_id,id_data):
        if(asv_id==0):
            self.asv0_goal_id = id_data
            print("ASV0 goal auv: "+str(self.asv0_goal_id))
        elif(asv_id==1):
            self.asv1_goal_id = id_data
            print("ASV1 goal auv: "+str(self.asv1_goal_id))
        

    def get_asv_goal_id(self,asv_id):
        if(asv_id==0):
            return(self.asv0_goal_id)
        elif(asv_id==1):
            return(self.asv1_goal_id)

    def ARTM(self,normalized_values):
        for robot in range(self.active_robots):
            scaled_values = normalized_values[robot]
            s = self.alpha*abs(scaled_values[0])+ self.beta*abs(scaled_values[1])+ self.gamma* abs(scaled_values[2])
            self.stimulus[robot] = s**self.n/(s**self.n + self.comm_signal[robot]**self.n)
        # extract the sorted goal robot IDs
        sorted_goal_ids = np.array([])
        sorted_goal_ids = np.argsort(self.stimulus)  
        return(sorted_goal_ids)
    # --------------------------------------------------------------------------------------
    def min_max_scale(self,values):
        # get the minimum and maximum values
        self.min_value = np.min(values)
        self.max_value = np.max(values)
        for robot in range(self.active_robots):
            scaled_values = np.array([])
            for value in range(self.number_of_stimulus):
                if(value==1): #distance stimulus: the minor the distance the greater the stimulus value
                    calc =(values[robot][value]- self.min_value)/(self.max_value-self.min_value)
                    dist = abs(calc-1)
                    scaled_values = np.append(scaled_values,dist)
                else:
                    calc =(values[robot][value]- self.min_value)/(self.max_value-self.min_value)
                    scaled_values = np.append(scaled_values,calc)
            self.scaled_senses[robot] = scaled_values
        return(self.scaled_senses)
    
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
                if(self.optimization_model==1):
                    self.stimulus_variables[self.removed_robots[element]] = [0,0,0]
                    self.min_max_scaled[self.removed_robots[element]] = [0,0,0]
                elif(self.optimization_model==2):
                    self.stimulus_variables[self.removed_robots[element]] = [0,0,0,0]
                    self.min_max_scaled[self.removed_robots[element]] = [0,0,0,0]
            self.remove_robot=False  
        
    def initialization(self,robot_id):
        if(self.robots_information[robot_id][0] != 0):
            self.robot_initialization[robot_id] = True
        
        if((self.robot_initialization == True).all()):
            self.system_init = True
            self.plot_random_points()

    def plot_random_points(self):
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
    
    def OWA(self):
        self.number_of_stimulus = 4 #Set to 4 in order to take into account the comm signal RSSI 
        OWA_inputs = self.min_max_scale(self.stimulus_variables)
        self.owa=[0,0,0,0,0,0]
        print("________________________________")
        print(OWA_inputs)

        # get the weights and set the values for w1,w2,w3,w4 where w1>=w2>=w3>=w4
        OWA_weights =np.array([self.w1,self.w2,self.w3,self.w4])
        OWA_weights_sorted = np.sort(OWA_weights)
        self.w1 = OWA_weights_sorted[3]
        self.w2 = OWA_weights_sorted[2]
        self.w3 = OWA_weights_sorted[1]
        self.w4 = OWA_weights_sorted[0]

        for robot in range(self.number_of_robots):
            OWA_input = np.sort(OWA_inputs[robot]) 
            self.max_element = OWA_input[3]
            self.middle1 = OWA_input[2]
            self.middle2 = OWA_input[1]
            self.min_element= OWA_input[0]
            self.owa[robot]= self.max_element*self.w1 + self.middle1*self.w2 + self.middle2*self.w3 + self.min_element*self.w4

        print("The owas are: "+str(self.owa))
        max_owa = max(self.owa)
        self.robot_goal_id = self.owa.index(max_owa)
        print("The maximum owa:"+str(max_owa)+" belongs to robot"+str(self.robot_goal_id))

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