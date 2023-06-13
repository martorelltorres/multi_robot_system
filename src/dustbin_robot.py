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
from std_msgs.msg import Int16,Header
from cola2_msgs.msg import  NavSts,BodyVelocityReq
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from cola2_msgs.srv import Goto, GotoRequest
from multi_robot_system.msg import CoverageStartTime,TravelledDistance,CommunicationDelay,ExplorationUpdate,Communication,Distances, Data
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
        self.repulsion_radius = self.get_param("repulsion_radius",2)
        self.adrift_radius = self.get_param("adrift_radius",5)
        self.tracking_radius = self.get_param("tracking_radius",20)
        self.area_handler =  area_partition("area_partition")

        # Initialize some variables
        self.pose = [0,0]
        self.robot_at_center = False
        self.robots_id = np.array([])
        self.communication_times_delay = [0,0,0,0,0,0]
        self.start_recording_time = [0,0,0,0,0,0]
        self.communication_times_end = [0,0,0,0,0,0]
        self.system_init = False
        self.robot_data = [0,0]
        self.reset_flag=True
        self.robots_information = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
        self.robots = []
        self.robot_initialization = np.array([])
        self.enable_tracking = False
        self.set_end_time = True
        self.start_dustbin_strategy =np.array([False,False,False,False,False,False])
        self.first_dustbin_time = True
        self.trigger = False
        self.exploration_tasks_update = np.array([])
        self.battery_charge= []
        self.s_norm = []
        self.distance = []
        self.time_init = []
        self.storage_disk =[]
        self.max_value = 500
        self.min_value = 1
        self.comm_signal = []
        self.stimulus = np.array([])
        self.robots_sense = np.array([])
        tasks = self.area_handler.get_polygon_number()
        self.max_stimulus=[]
        self.min_stimulus = []
        self.time_threshold=[]
        self.scaled_senses = []
        self.senses = []
        self.number_of_stimulus = 3
        self.active_robots = self.number_of_robots
        self.robot_to_remove = 999
        self.removed_robots= []
        self.communication=True
        self.asv_init=False
        self.transferred_data = 0
        self.flag=True
        self.communication_latency = []
        self.first_time = True
        self.travelled_distance = 0
        # intialize the variables
        for robot in range(self.number_of_stimulus):
            self.senses.append(0)

        for i in range(self.active_robots):
            self.scaled_senses.append(self.senses)
            self.max_stimulus.append(0)
            self.min_stimulus.append(0)
       
        for task in range(tasks):
            self.exploration_tasks_update = np.append(self.exploration_tasks_update,False)

        # initialize the robots variables
        for robot_ in range(self.number_of_robots):
            self.robot_initialization = np.append(self.robot_initialization,False) # self.robot_initialization = [False,False;False]
            self.robots.append(robot_)  
            self.robots_id = np.append(self.robots_id,robot_)
            self.robots_id = self.robots_id.astype(int) #convert float to int type
            self.comm_signal.append(0)
            self.storage_disk.append(0)
            self.time_init.append(0)
            self.distance.append(0)
            self.battery_charge.append(0)
            self.stimulus = np.append(self.stimulus,0)
            self.robots_sense = np.append(self.robots_sense,0)
            self.s_norm.append(0)
            self.time_threshold.append(0)
            self.communication_latency.append(0)
        
        self.stimulus_variables= np.vstack((self.robots_sense,self.robots_sense,self.robots_sense,self.robots_sense,self.robots_sense,self.robots_sense))

        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        #Subscribers
        for robot_id in range(self.number_of_robots):
            rospy.Subscriber(
                '/robot'+str(robot_id)+'/navigator/navigation',
                NavSts,
                self.update_robots_position,
                robot_id,
                queue_size=1) 

        rospy.Subscriber('/robot'+str(self.robot_ID)+'/navigator/navigation',
                            NavSts,    
                            self.update_robot_position,
                            queue_size=1)

        rospy.Subscriber('/mrs/exploration_area_update',
                            ExplorationUpdate,    
                            self.kill_the_process,
                            queue_size=1)
        
        rospy.Subscriber('/mrs/exploration_finished',
                            Int16,    
                            self.remove_robot_from_dustbin_goals,
                            queue_size=1)
        
        if(self.asv_init==True):
            for robot_id in range(self.number_of_robots):
                rospy.Subscriber("/mrs/communications_sim/robot"+str(robot_id)+"_communication",
                            Communication,    
                            self.update_communication_state,
                            queue_size=1)
            
        
        for robot in range(self.number_of_robots):
            rospy.Subscriber(
            '/mrs/robot'+str(robot)+'_start_coverage_time',
            CoverageStartTime,
            self.set_coverage_start_time,
            robot,
            queue_size=1)      

        #Publishers
        self.corrected_bvr = rospy.Publisher('/robot6/controller/body_velocity_req',
                                                BodyVelocityReq,
                                                queue_size=1)

        self.markerPub_repulsion = rospy.Publisher('repulsion_radius',
                                                    Marker,
                                                    queue_size=1)

        self.markerPub_adrift = rospy.Publisher('adrift_radius',
                                                Marker,
                                                queue_size=1)

        self.markerPub_tracking = rospy.Publisher('tracking_radius',
                                                Marker,
                                                queue_size=1)
        
        self.communication_delay_time = rospy.Publisher("communication_time_delay",
                                        CommunicationDelay,
                                        queue_size=1)
        
        self.reset_storage_pub = rospy.Publisher('reset_storage_disk',
                                        Int16,
                                        queue_size=1)
        
        self.robot_distances_pub = rospy.Publisher("robot_distances",
                                        Distances,
                                        queue_size=1)
        # ---------------------------------------------------------------------------
        self.goal_id_pub = rospy.Publisher('goal_id',
                                        Data,
                                        queue_size=1)
        
        self.transferred_data_pub = rospy.Publisher('transferred_data',
                                Data,
                                queue_size=1)
               
        self.communication_latency_pub = rospy.Publisher('communication_latency',
                                CommunicationDelay,
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
        
        self.read_area_info()

        
        # Init periodic timers self.distance
        rospy.Timer(rospy.Duration(0.1), self.dustbin_trigger)
        rospy.Timer(rospy.Duration(1.0), self.update_travelled_distance)
    
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

    def dustbin_trigger(self, event):
        # start the dustbin_strategy if all the has started the coverage
        if (np.all(self.start_dustbin_strategy) and self.first_dustbin_time == True):
            self.dustbin_strategy()
            self.first_dustbin_time = False
   
    def update_robot_position(self, msg):
        self.asv_north_position = msg.position.north
        self.asv_east_position = msg.position.east      
        self.asv_yaw = msg.orientation.yaw
        self.asv_init=True
        # move the ASV to the central area
        if (self.robot_at_center == False):
            self.transit_to(self.main_polygon_centroid)
            self.robot_at_center = True

        if(self.enable_tracking == True):
            self.tracking()

    def update_communication_state(self,msg):
        robot = msg.auv_id
        comm_freq = msg.communication_freq
        self.storage_disk[robot] = msg.storage_disk
        self.battery_charge[robot] = msg.battery_charge
        self.comm_signal[robot] = comm_freq* (self.max_value*(self.number_of_robots+2))

        # get the distance
        distance = self.get_distance(robot)
        self.distance[robot] = distance

        # publish the distance to the AUV
        msg = Distances()
        msg.header.stamp = rospy.Time.now()
        msg.auv_id = robot 
        msg.distance = distance
        self.robot_distances_pub.publish(msg)
    
    def set_coverage_start_time(self,msg,robot_id):
        self.start_dustbin_strategy[robot_id]= True
        # print(self.start_dustbin_strategy)
        self.t_start = msg.time.secs
        self.time_robot_id = msg.robot_id
        self.time_init[robot_id] = msg.time.secs
        self.start_recording_time[self.time_robot_id] = self.t_start 

        # reset the storage values
        self.storage_disk[robot_id] = 0
        reset_id = robot_id
        msg = Int16()
        msg.data = reset_id 
        self.reset_storage_pub.publish(msg)
           
    def set_comm_start_time(self,time):
        self.t_start = time.secs
        self.start_recording_time[self.robot_goal_id] = self.t_start
        
    def set_comm_end_time(self,time):
        self.t_end = time.secs
        self.communication_times_end[self.robot_goal_id] = self.t_end
        self.get_comm_time_delay()
    
    def get_comm_time_delay(self):
        end_time = self.communication_times_end[self.robot_goal_id]
        start_time = self.start_recording_time[self.robot_goal_id]
        comm_delay = end_time - start_time
        self.communication_times_delay[self.robot_goal_id] =  comm_delay
        self.set_comm_start_time(rospy.Time.now())

        # check if there are zeros in the array
        if(self.communication_times_delay.count(0) == 0):
            self.trigger = True
       
        # publish the communication_delay_time
        msg = CommunicationDelay()
        msg.header.stamp = rospy.Time.now()
        msg.comm_delay = self.communication_times_delay 
        self.communication_delay_time.publish(msg)
          
    def get_distance(self, robot_id):
        x_diff =  self.asv_north_position - self.robots_information[robot_id][0]
        y_diff =  self.asv_east_position - self.robots_information[robot_id][1] 
        distance =  sqrt(x_diff**2 + y_diff**2)
        return(distance)
   
    def get_time_threshold(self,robot_id):
        time = rospy.Time.now().secs - self.time_init[robot_id]
        self.time_threshold[robot_id]= time
        return(time)

    def scale_value(self, value):
        scaled_value =(value- self.min_value)/(self.max_value-self.min_value)
        return(scaled_value)
       
    def min_max_scale(self,values):
        for robot in range(self.active_robots):
            scaled_values = np.array([])
            for value in range(self.number_of_stimulus):
                calc =((values[robot][value]- self.min_value)*self.max_value)/(np.max(values)-self.min_value)
                scaled_values = np.append(scaled_values,calc)
            self.scaled_senses[robot] = scaled_values
        return(self.scaled_senses)

    def get_stimulus(self):
        for robot in range(self.active_robots):
            # The self.robots_id variable stores the active AUV explorer robots
            # get time delay
            self.robots_sense[0] = self.get_time_threshold(self.robots_id[robot])

            # get the distance between ASV-AUV's
            distance = self.distance[self.robots_id[robot]]
            self.robots_sense[1] = distance

            # get the hard disk storage capacity
            self.robots_sense[2] = self.storage_disk[self.robots_id[robot]]
            self.stimulus_variables[self.robots_id[robot]] = self.robots_sense
        
           
        print(".................. STIMULUS VARIABLES ..................")
        print(self.stimulus_variables)
        
        self.min_max_scaled = self.min_max_scale(self.stimulus_variables)

        # set at minimum value the robots that have completed their work 
        if(self.robot_to_remove!=999 and self.remove_robot==True):
            for element in range(len(self.removed_robots)):
                self.stimulus_variables[self.removed_robots[element]] = [0,0,0,0,0,0]
                self.min_max_scaled[self.removed_robots[element]] = [0,0,0,0,0,0]
            self.remove_robot=False
    

        print(".................. SCALED STIMULUS VARIABLES ..................")
        print(self.min_max_scaled)
   
        # obtain the stimulus value using a weighted sum
        self.alpha = 3
        self.beta = 2
        self.gamma = 5
        self.n = 4

        for robot in range(self.active_robots):
            scaled_values = self.min_max_scaled[robot]
            s = self.alpha*abs(scaled_values[0])+ self.beta*abs(scaled_values[1])+ self.gamma* abs(scaled_values[2])
            self.stimulus[robot] = s**self.n/(s**self.n + self.comm_signal[robot]**self.n)
            
        # remove the robots that have completed their work
        for element in range(len(self.removed_robots)):
            self.stimulus[self.removed_robots[element]] = 0
        return(self.stimulus)

    def time_trigger(self):
        # this function set the robot goal id for the dustbin_strategy
        # The trigger flag becomes True only when there are no zeros in the self.communication_times_delay array 
        if (self.trigger==True):
            self.set_comm_start_time(rospy.Time.now())

        self.get_stimulus()

        # Choose the optimization strategy
        self.use_max_prob()
        # self.use_random_prob()
        # self.use_max_stimulus()
        # self.max_min_stimulus()
        # self.round_robin()

        print("The resulting AUV goal ID is: "+str(self.robot_goal_id))

        self.enable_tracking = True

        # publish the goal_id
        msg = Data()
        msg.header.stamp = rospy.Time.now()
        msg.data = self.robot_goal_id
        self.goal_id_pub.publish(msg)

    
    def max_min_stimulus(self):
        minimum_values = []
        max_value = 0
        for element in range(len(self.robots_id)):
            minimum_values.append(min(self.min_max_scaled[element])) 
        print("The minimum values are: "+str(minimum_values))
        max_value = max(minimum_values)
        print("The max min value is: "+str(max_value))
        self.robot_goal_id = minimum_values.index(max_value)
    

    def use_max_stimulus(self):
        # remove the max_stimulus element if needed
        for element in range(len(self.removed_robots)):
            self.max_stimulus[self.removed_robots[element]]= 0
        
        for element in range(self.active_robots):
            self.max_stimulus[element] = max(self.min_max_scaled[element])

        print("The maximum stimulus values are: "+str(self.max_stimulus))
        
        maximum_value = max(self.max_stimulus)
        print("The maximum value is: "+str(maximum_value))
        self.robot_goal_id = self.max_stimulus.index(maximum_value)
        
    def use_max_prob(self):
        print("Probability function: "+str(self.stimulus))
        # extract the goal robot ID
        self.robot_goal_id = self.stimulus.argmax()  

    def round_robin(self):
        self.robots_id = np.roll(self.robots_id,1)
        self.robot_goal_id = self.robots_id[0]  
        
    def update_robots_position(self, msg, robot_id):
        # fill the robots_information array with the robots information received from the NavSts 
        self.robots_information[robot_id][0] = msg.position.north
        self.robots_information[robot_id][1] = msg.position.east

        # check the system initialization
        if(self.system_init == False):
            self.initialization(robot_id) 

    def dustbin_strategy(self):
        if(self.system_init==True and self.communication==True):
            self.time_trigger()
    
    def kill_the_process(self,msg):
        # update area explored
        self.exploration_tasks_update[msg.explored_sub_area] = True
        print("__________TASK UPDATE__________")
        print(self.exploration_tasks_update)
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
        self.check_dustbin_robot()
    
    def check_dustbin_robot(self):
        if(np.size(self.robots_id)==0):
            self.enable_tracking = False
            self.transit_to(self.main_polygon_centroid)
                  
    def initialization(self,robot_id):
        # check if all the n robots are publishing their information
        if(self.robots_information[robot_id][0] != 0):
            self.robot_initialization[robot_id] = True
        
        if((self.robot_initialization == True).all()):
            self.system_init = True
    
    def communicate(self):
        print("_____COMMUNICATE_____")
        time = self.storage_disk[self.robot_goal_id]/300
        communication_init = rospy.Time.now().secs

        while(rospy.Time.now().secs-communication_init < time):
            self.communication=False
        
        self.get_time_threshold(self.robot_goal_id)
        self.transferred_data = self.transferred_data + self.storage_disk[self.robot_goal_id]

        # publish the total transferred data
        msg = Data()
        msg.header.stamp = rospy.Time.now()
        msg.data = self.transferred_data
        self.transferred_data_pub.publish(msg)

        # reset the storage values
        self.storage_disk[self.robot_goal_id] = 0
        reset_id = self.robot_goal_id
        msg = Int16()
        msg.data = reset_id 
        self.reset_storage_pub.publish(msg)

        self.communication_latency[self.robot_goal_id] = rospy.Time.now().secs - self.time_init[self.robot_goal_id]
        msg = CommunicationDelay()
        msg.header.stamp = rospy.Time.now()
        msg.comm_delay = self.communication_latency 
        self.communication_latency_pub.publish(msg)
        self.time_init[self.robot_goal_id] = rospy.Time.now().secs

        print("______ COMMUNICATION FINISHED ______")
        self.communication=True
        self.dustbin_strategy()

    def tracking(self):
        self.auv_position_north = self.robots_information[self.robot_goal_id][0]
        self.asv_position_north = self.asv_north_position
        self.auv_position_east = self.robots_information[self.robot_goal_id][1]
        self.asv_position_east = self.asv_east_position

        self.tracking_marker()
        self.adrift_marker()
        self.repulsion_marker()

        self.x_distance = self.auv_position_north-self.asv_position_north
        self.y_distance = self.auv_position_east-self.asv_position_east
        self.radius = sqrt((self.x_distance)**2 + (self.y_distance)**2)
        self.initialized = True

        # Communication area
        if(self.radius > self.adrift_radius):
            self.tracking_strategy()

        if(self.radius < (self.adrift_radius+3) and self.set_end_time == True):
            self.set_comm_end_time(rospy.Time.now())
            self.set_end_time = False
            self.communicate()

        # Repulsion area
        if(self.radius <= self.repulsion_radius):
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
        constant_linear_velocity = 3
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
        bvr.goal.priority      = 60
        self.corrected_bvr.publish(bvr)
    
    def transit_to(self,pose):    
        self.disable_all_and_set_idle_srv()
        """Goto to position x, y, z, at velocity vel."""
        # // Define waypoint attributes
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
        goto_req.reference = 0 #REFERENCE_NED=0  REFERENCE_GLOBAL=1 REFERENCE_VEHICLE=2
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
        self.anchor.pose.position.z = 0
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
        self.follow.pose.position.z = 0
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
