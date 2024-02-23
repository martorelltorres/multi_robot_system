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
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Quaternion,PointStamped
import tf       

class acoustic_communication:

    def __init__(self, name):
        self.name = name
        self.stored_data = 0
        self.asv_init = False
        node_name = rospy.get_name()
        self.robot_ID = self.get_param('~robot_ID',0) 
        self.ASV_ID = self.get_param("~ASV_ID",0)
          
        #Publishers      
        self.pose_covariance_pub = rospy.Publisher('/robot'+str(self.robot_ID)+'/acoustic_communication',
                                PoseWithCovarianceStamped,
                                queue_size=2)
   
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
        self.auv_roll = msg.orientation.roll
        self.auv_pitch = msg.orientation.pitch
        self.auv_yaw = msg.orientation.yaw
        # send data acoustically
        if( self.asv_init == True):
            self.communication_process()    

    def get_comm_freq(self):
        self.distance = np.sqrt((self.auv_north - self.asv_north)**2 + (self.auv_east - self.asv_east)**2 + (self.auv_depth)**2)
        communication_freq = 0.848371 - 0.01412292*self.distance + 0.00007763495*self.distance**2
        # Add noise
        noise = np.random.normal(0, 0.1)
        freq_with_noise = communication_freq + noise
        # Create a rate
        self.rate = rospy.Rate(freq_with_noise)
        return(freq_with_noise)

    
    def communication_process(self):
        self.get_comm_freq()
        # obtain the accuracy from the experimental characterization curve
        accuracy = (+1.9400*10**-5*self.distance**3)+(0.009959*self.distance**2)+(0.896506*self.distance)+16.90
        # create PoseWithCovarianceStamped message
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/robot'+str(self.robot_ID)+'/base_link'
        msg.pose.pose.position.x = self.auv_north
        msg.pose.pose.position.y = self.auv_east
        msg.pose.pose.position.z = self.auv_depth
        # convert Euler angles to Quaternion
        quaternion_from_euler = tf.transformations.quaternion_from_euler(self.auv_roll, self.auv_pitch, self.auv_yaw)
        quaternion = Quaternion(*quaternion_from_euler)
        msg.pose.pose.orientation = quaternion
        msg.pose.covariance[0] = accuracy
        msg.pose.covariance[8] = accuracy
        msg.pose.covariance[16] = accuracy
        self.pose_covariance_pub.publish(msg)
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


