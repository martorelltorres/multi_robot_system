#!/usr/bin/env python

import rospy
from cola2_msgs.msg import  NavSts
import numpy as np
from multi_robot_system.msg import Communication
import random

class communications:

    def __init__(self, name):
        self.name = name
        self.number_of_robots = self.get_param('number_of_robots')
        # navigation topics
        self.r0_navigation_topic = self.get_param('~r0_navigation_topic','/turbot/navigator/navigation') 
        self.r1_navigation_topic = self.get_param('~r1_navigation_topic','/xiroi/navigator/navigation') 
        # communication noise frequency 
        self.low_noise = self.get_param('~low_noise','1') 
        self.medium_noise = self.get_param('~medium_noise','0.5') 
        self.high_noise = self.get_param('~high_noise','0.1') 
        # distance range
        self.low_distance = self.get_param('~low_distance','10') 
        self.medium_distance = self.get_param('~medium_distance','40') 
        self.large_distance = self.get_param('~large_distance','80') 

        self.r1_init = False
        self.r0_init = False
        self.system_init = False

        # for robot in range(self.number_of_robots):
        #     rospy.Subscriber("/robot"+str(robot)+"/navigator/navigation",
        #                  NavSts,    
        #                  self.update_robots_position,
        #                  queue_size=1)


        #Subscribers
        rospy.Subscriber(self.r0_navigation_topic,
                         NavSts,    
                         self.update_r0_position,
                         queue_size=1)

        rospy.Subscriber(self.r1_navigation_topic,
                        NavSts,    
                        self.update_r1_position,
                        queue_size=1)
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

    def update_r0_position(self, msg):
        self.r0_position_north = msg.position.north
        self.r0_position_east = msg.position.east
        self.r0_position_depth = msg.position.depth
        self.r0_yaw = msg.orientation.yaw
        self.r0_init = True
        self.initialization()
    
    # def update_robots_position(self, msg):
    #     self.position_north = msg.position.north
    #     self.position_east = msg.position.east
    #     self.position_depth = msg.position.depth
    #     self.yaw = msg.orientation.yaw
    #     self.init = True
    #     self.initialization()

    def update_r1_position(self, msg):
        self.r1_position_north = msg.position.north
        self.r1_position_east = msg.position.east
        self.r1_position_depth = msg.position.depth
        self.r1_yaw = msg.orientation.yaw
        self.r1_init = True
        self.initialization()
    
    def initialization(self):
        if(self.r1_init==True and self.r0_init==True):
            self.system_init = True
            self.communication_noise()  
        else:
            self.system_init = False

    def distance_between_robots(self):
        x_distance = self.r1_position_north - self.r0_position_north
        y_distance = self.r1_position_east - self.r0_position_east
        self.distance = np.sqrt(x_distance**2 + y_distance**2)
        self.distance = round(self.distance, 2)
    
    def distance_between_robots(self):
        x_distance = self.r1_position_north - self.r0_position_north
        y_distance = self.r1_position_east - self.r0_position_east
        self.distance = np.sqrt(x_distance**2 + y_distance**2)
        self.distance = round(self.distance, 2)

    def communication_noise(self):
        self.distance_between_robots()
        if(self.distance < self.low_distance):
            self.noise = self.low_noise
            self.distance_range = "low_distance"
            self.communication_freq = 1
            self.rate = rospy.Rate(self.communication_freq)
            self.communication()

        elif(self.low_distance <= self.distance <= self.medium_distance):
            self.distance_range = "medium_distance"
            self.noise = self.medium_noise
            self.communication_freq = 0.5
            self.rate = rospy.Rate(self.communication_freq)
            self.communication()
        else:
            self.noise = self.high_noise
            self.communication_freq = 0.1  
            self.distance_range = "large_distance"
            self.rate = rospy.Rate(self.communication_freq)
            self.communication()        

    def random_interference(self):
        interference = random.randint(0,10)
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
            communication_msg.communication_freq = self.communication_freq
            self.communication_pub.publish(communication_msg)
            self.rate.sleep()

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


