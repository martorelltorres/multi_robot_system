#!/usr/bin/env python

import rospy
from cola2_msgs.msg import  NavSts
import numpy as np
from std_msgs.msg import Empty
from multi_robot_system.msg import AcousticData,BufferedData,NED
from std_msgs.msg import Int16
from sensor_msgs.msg import BatteryState
import random
from functools import partial
from itertools import combinations
import matplotlib.pyplot as plt

class acoustic_communication:

    def __init__(self, name):
        self.name = name
        self.stored_data = 0
        self.asv_init = False
        node_name = rospy.get_name()
        self.robot_ID = self.get_param('~robot_ID',0) 
        self.ASV_ID = self.get_param("~ASV_ID",0)
          
        #Publishers
        self.acoustic_communication_pub = rospy.Publisher('/robot'+str(self.robot_ID)+'/acoustic_communication',
                                        AcousticData,
                                        queue_size=1)
   
        #Subscribers         
        rospy.Subscriber('/robot'+str(self.robot_ID)+'/navigator/navigation',
                        NavSts,
                        self.update_robot_position,
                        queue_size=1) 
        
        rospy.Subscriber('/robot'+str(self.ASV_ID)+'/navigator/navigation',
                NavSts,
                self.update_asv_position,
                queue_size=1) 
        
        rospy.Subscriber('/mrs/data_buffered',
                BufferedData,
                self.update_buffered_data,
                queue_size=1) 

    def update_buffered_data(self,msg):
        self.stored_data = msg.storage[self.robot_ID]
        
    def update_asv_position(self,msg):
        self.asv_north = msg.position.north
        self.asv_east = msg.position.east
        self.asv_init = True
    
    def update_robot_position(self, msg):
        self.auv_north = msg.position.north
        self.auv_east = msg.position.east
        self.auv_depth = msg.position.depth
        self.auv_yaw = msg.orientation.yaw
        # send data acoustically
        if( self.asv_init == True):
            self.communication_process()    

    def get_comm_freq(self):
        distance = np.sqrt((self.auv_north - self.asv_north)**2 + (self.auv_east - self.asv_east)**2 + (self.auv_depth)**2)
        # y = -0.0000001 + 0.0000776*x^1 + -0.0141229*x^2 + 0.8483710*x^3
        communication_freq = 0.848371 - 0.01412292*distance + 0.00007763495*distance**2
        # Add noise
        noise = np.random.normal(0, 0.3)
        freq_with_noise = communication_freq + noise
        # Create a rate
        self.rate = rospy.Rate(freq_with_noise)
        return(freq_with_noise)

    def communication_process(self):
        self.get_comm_freq()
        acoustic_data_msg = AcousticData()
        acoustic_data_msg.header.stamp = rospy.Time.now()
        acoustic_data_msg.position.north = self.auv_north
        acoustic_data_msg.position.east = self.auv_east
        acoustic_data_msg.position.depth = self.auv_depth
        acoustic_data_msg.orientation.yaw = self.auv_yaw
        acoustic_data_msg.stored_data = self.stored_data
        self.acoustic_communication_pub.publish(acoustic_data_msg)
        self.rate.sleep()
      
    def reset_values(self,msg):
        self.storage_disk[msg.data] = 1

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
        rospy.init_node('acoustic_communication')
        acoustic_communication = acoustic_communication(rospy.get_name())
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass


