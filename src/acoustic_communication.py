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
        self.asv_init = False
        node_name = rospy.get_name()
        self.number_of_robots = self.get_param('number_of_robots')
        self.number_of_asvs= 1
        self.auv_position = []
        self.asv_position = [[],[]]
        self.auv_flag = np.array([])
        self.asv_auv_distance = 10000
        self.closest_asv_indices = np.zeros(self.number_of_robots, dtype=int)
        self.closest_asv = None
        self.min_distance = np.inf

        for robot_ in range(self.number_of_robots):
          self.auv_flag = np.append(self.auv_flag,False)
          self.auv_position.append([])
        #Publishers      
        self.auv0_pose_covariance_pub = rospy.Publisher('/robot0/acoustic_communication',
                                PoseWithCovarianceStamped,
                                queue_size=2)
        
        self.auv1_pose_covariance_pub = rospy.Publisher('/robot1/acoustic_communication',
                                PoseWithCovarianceStamped,
                                queue_size=2)
        
        self.auv2_pose_covariance_pub = rospy.Publisher('/robot2/acoustic_communication',
                                PoseWithCovarianceStamped,
                                queue_size=2)
        
        self.auv3_pose_covariance_pub = rospy.Publisher('/robot3/acoustic_communication',
                                PoseWithCovarianceStamped,
                                queue_size=2)
        
  
        #Subscribers 
        for auv in range(self.number_of_robots):        
            rospy.Subscriber('/robot'+str(auv)+'/navigator/navigation',
                            NavSts,
                            self.update_auv_position,
                            auv,
                            queue_size=1) 
        
        for asv in range(self.number_of_asvs):
            rospy.Subscriber('/robot'+str(asv)+'/navigator/navigation',
                    NavSts,
                    self.update_asv_position,
                    asv,
                    queue_size=1) 
                
    def update_asv_position(self,msg,asv_id):
        self.asv_position[asv_id]=[msg.position.north,msg.position.east]
        self.asv_init = True
    
    def update_auv_position(self, msg,robot_id):
        self.auv_position[robot_id]= [msg.position.north,msg.position.east,msg.position.depth,msg.orientation.roll,msg.orientation.pitch,msg.orientation.yaw]
        self.auv_flag[robot_id] = True

        # get the closest ASV to each AUV
        if( np.all(self.auv_flag) == True):
            self.get_closest_ASV
            self.communication_process(self.closest_asv_indices[robot_id],robot_id)    
            
    def get_closest_ASV(self):
        for auv in range(self.number_of_robots):
            min_distance = np.inf
            for asv in range(self.number_of_asvs):
                dist = self.get_distance(self.auv_position[auv][0],self.auv_position[auv][1],self.auv_position[auv][2],self.asv_position[asv][0],self.asv_position[asv][1])
                
                if(dist< min_distance):
                    min_distance = dist
                    self.closest_asv = asv

            self.closest_asv_indices[auv] = self.closest_asv

    def get_distance(self,auv_x,auv_y,auv_z,asv_x,asv_y):
        distance = np.sqrt((auv_x - asv_x)**2 + (auv_y - asv_y)**2 + (auv_z)**2)
        return(distance)

    def get_comm_freq(self,asv,auv):
        self.distance = self.get_distance(self.auv_position[auv][0],self.auv_position[auv][1],self.auv_position[auv][2],self.asv_position[asv][0],self.asv_position[asv][1])
        communication_freq = 0.848371 - 0.01412292*self.distance + 0.00007763495*self.distance**2
        # Add noise
        noise = np.random.normal(0, 0.2)
        freq_with_noise = communication_freq + noise
        # Create a rate
        self.rate = rospy.Rate(freq_with_noise)
        return(freq_with_noise)

    
    def communication_process(self,asv,auv):
        self.get_comm_freq(asv,auv)
        self.distance = self.get_distance(self.auv_position[auv][0],self.auv_position[auv][1],self.auv_position[auv][2],self.asv_position[asv][0],self.asv_position[asv][1])
        # obtain the accuracy from the experimental characterization curve
        accuracy = (8.9157*10**-6*self.distance**2)+(0.0061121*self.distance)-0.06835
        # create PoseWithCovarianceStamped message
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'robot'+str(auv)+'/odom'
        msg.pose.pose.position.x = self.auv_position[auv][0]
        msg.pose.pose.position.y = self.auv_position[auv][1]
        msg.pose.pose.position.z = self.auv_position[auv][2]
        # convert Euler angles to Quaternion
        quaternion_from_euler = tf.transformations.quaternion_from_euler(self.auv_position[auv][3], self.auv_position[auv][4], self.auv_position[auv][5])
        quaternion = Quaternion(*quaternion_from_euler)
        msg.pose.pose.orientation = quaternion
        msg.pose.covariance[0] = accuracy
        msg.pose.covariance[8] = accuracy
        msg.pose.covariance[16] = accuracy

        if(auv==0):
            self.auv0_pose_covariance_pub.publish(msg)
        elif(auv==1):
            self.auv1_pose_covariance_pub.publish(msg)
        elif(auv==2):
            self.auv2_pose_covariance_pub.publish(msg)
        elif(auv==3):
            self.auv3_pose_covariance_pub.publish(msg)

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


