#!/usr/bin/env python

import rospy
from cola2_msgs.msg import  NavSts
import numpy as np

class communications:

    def __init__(self, name):
        self.name = name
        # navigation
        self.r0_navigation_topic = self.get_param('~r0_navigation_topic','/turbot/navigator/navigation') 
        self.r1_navigation_topic = self.get_param('~r1_navigation_topic','/xiroi/navigator/navigation') 
        # communication noise 
        self.low_noise = self.get_param('~low_noise','0.9') 
        self.medium_noise = self.get_param('~medium','0.6') 
        self.high_noise = self.get_param('~high_noise','0.3') 
        # distance
        self.low_distance = self.get_param('~low_distance','10') 
        self.medium_distance = self.get_param('~medium_distance','40') 
        self.large_distance = self.get_param('~large_distance','80') 

        self.communication_freq = 0
        self.r1_init = False
        self.r0_init = False

        #Subscribers
        rospy.Subscriber(self.r0_navigation_topic,
                         NavSts,    
                         self.update_r0_position,
                         queue_size=1)

        rospy.Subscriber(self.r1_navigation_topic,
                        NavSts,    
                        self.update_r1_position,
                        queue_size=1)

        self.communication_noise()                


    def update_r0_position(self, msg):
        self.r0_position_north = msg.position.north
        self.r0_position_east = msg.position.east
        self.r0_position_depth = msg.position.depth
        self.r0_yaw = msg.orientation.yaw
        self.r0_init = True

    def update_r1_position(self, msg):
        self.r1_position_north = msg.position.north
        self.r1_position_east = msg.position.east
        self.r1_position_depth = msg.position.depth
        self.r1_yaw = msg.orientation.yaw
        self.r1_init = True

    def distance_between_robots(self):
        x_distance = self.r1_position_north - self.r0_position_north
        y_distance = self.r1_position_east - self.r0_position_east
        self.distance = np.sqrt(x_distance**2 + y_distance**2)
        print("The distance between robots is: "+ str(self.distance))


    def communication_noise(self):

        if(self.r0_init==True and self.r1_init==True):
            self.distance_between_robots()

            if(self.distance < self.low_distance):
                noise = self.low_noise
                self.communication_freq = 1

            elif(self.low_distance < self.distance < self.medium_distance):
                noise = self.medium_noise
                self.communication_freq = 3

            else:
                noise = self.high_noise
                self.communication_freq = 6
            
            # Init periodic timers
            rospy.Timer(rospy.Duration(self.communication_freq), self.comunicate)
            
        else:
            rospy.logwarn("Communication module is not init yet...")

    def comunicate(self,event):
        print("INNNNN")
    
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


