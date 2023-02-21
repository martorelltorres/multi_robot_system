#!/usr/bin/env python

import rospy
from cola2_msgs.msg import  NavSts
import numpy as np
from multi_robot_system.msg import Communication
import random
from functools import partial
from itertools import combinations

class communications:

    def __init__(self, name):
        self.name = name
        self.number_of_robots = self.get_param('number_of_robots')

        # communication noise frequency 
        self.low_noise = self.get_param('~low_noise','1') 
        self.medium_noise = self.get_param('~medium_noise','0.5') 
        self.high_noise = self.get_param('~high_noise','0.1') 

        # distance range
        self.low_distance = self.get_param('~low_distance','10') 
        self.medium_distance = self.get_param('~medium_distance','40') 
        self.large_distance = self.get_param('~large_distance','80') 

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
        #Publishers
        node_name = rospy.get_name()
        self.communication_pub = rospy.Publisher(node_name +"/communication_info",
                                        Communication,
                                        queue_size=1)
        #Subscribers
        
        rospy.Subscriber(node_name +"/communication_info",
                         Communication,    
                         self.monitoring_communications,
                         queue_size=1)
    
        for robot in range(self.number_of_robots):
            rospy.Subscriber(
                '/robot'+str(robot+1)+'/navigator/navigation',
                NavSts,
                self.update_robot_position,
                robot,
                queue_size=1)

    def update_robot_position(self, msg, robot_id):

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
        else:
            self.communication_noise()

    def initialization(self,robot_id):
        # check if all the n robots are publishing their information
        if(self.robots_information[[robot_id][self.robot_data[0]]] != 0): 
            self.robot_initialization[robot_id] = True
        
        if((self.robot_initialization == True).all()):
            self.system_init = True

    def distance_between_robots(self,first_robot,second_robot):
        first_robot_north = self.robots_information[[first_robot][self.robot_data[0]]]
        first_robot_east = self.robots_information[[first_robot][self.robot_data[1]]]
        second_robot_north = self.robots_information[[second_robot][self.robot_data[0]]]
        second_robot_east = self.robots_information[[second_robot][self.robot_data[1]]]
        x_distance = first_robot_north - second_robot_north
        y_distance = first_robot_east - second_robot_east
        self.distance = np.sqrt(x_distance**2 + y_distance**2)
        self.distance = round(self.distance, 2)   

    def communication_noise(self):
        for robot in range(self.number_of_robots):
            # find the different combinations between the n robots
            extracted_combinations = combinations(self.robots, 2)
            robot_combinations = np.array(list(extracted_combinations)[robot])
            self.distance_between_robots(robot_combinations[0], robot_combinations[1])

            if(self.distance < self.low_distance):
                self.noise = self.low_noise
                self.distance_range = "low_distance"
                self.communication_freq = 1
                self.rate = rospy.Rate(self.communication_freq)
                self.robot1_id = robot_combinations[0]
                self.robot2_id = robot_combinations[1]
                self.communication()

            elif(self.low_distance <= self.distance <= self.medium_distance):
                self.distance_range = "medium_distance"
                self.noise = self.medium_noise
                self.communication_freq = 0.5
                self.robot1_id = robot_combinations[0]
                self.robot2_id = robot_combinations[1]
                self.rate = rospy.Rate(self.communication_freq)
                self.communication()
            else:
                self.noise = self.high_noise
                self.communication_freq = 0.1  
                self.distance_range = "large_distance"
                self.robot1_id = robot_combinations[0]
                self.robot2_id = robot_combinations[1]
                self.rate = rospy.Rate(self.communication_freq)
                self.communication()        

    def random_interference(self):
        interference = random.randint(0,2)
        rospy.sleep(interference)

    def communication(self):
        random_number = random.randint(0,100)
        if(random_number%2 == 0):
            self.random_interference()
        else:
            communication_msg = Communication()
            communication_msg.header.frame_id = "multi_robot_system"
            communication_msg.header.stamp = rospy.Time.now()
            communication_msg.distance = self.distance
            communication_msg.distance_range = self.distance_range
            communication_msg.noise_level = self.noise
            communication_msg.robot1_id = self.robot1_id
            communication_msg.robot2_id = self.robot2_id
            communication_msg.communication_freq = self.communication_freq
            self.communication_pub.publish(communication_msg)

    def monitoring_communications(self,msg):
        distance = msg.distance
       
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
        rospy.init_node('communications')
        communications = communications(rospy.get_name())
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass


