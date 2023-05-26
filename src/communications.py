#!/usr/bin/env python

import rospy
from cola2_msgs.msg import  NavSts
import numpy as np
from std_msgs.msg import Empty
from multi_robot_system.msg import Communication,Distances
from std_msgs.msg import Int16
from sensor_msgs.msg import BatteryState
import random
from functools import partial
from itertools import combinations

class communications:

    def __init__(self, name):
        self.name = name
        node_name = rospy.get_name()
        self.number_of_robots = self.get_param('number_of_robots')
        self.robot_ID = self.get_param('~robot_ID',0) 

        self.system_init = False
        # self.robots_information = [[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0]]
        self.robots_information = []
        self.data = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.robots = []
        self.robot_initialization = np.array([])
        self.storage_disk =[]
        self.communication_pub = []
        self.distances = []
        self.battery_charge = []
        self.communication_freq = 1


       # initialize the robots variables
        for robot in range(self.number_of_robots+1):# add one in order to get info from the ASV too
            self.robot_initialization = np.append(self.robot_initialization,False) # self.robot_initialization = [False,False;False]
            self.robots.append(robot)  # self.robots = [0,1,2]
            self.robots_information.append(self.data)
        
        for robot_ in range(self.number_of_robots):
            self.storage_disk.append(0)
            self.battery_charge.append(0)
            


        #Publishers

        self.robot0_comm_pub = rospy.Publisher(node_name +"/robot0_communication",
                                        Communication,
                                        queue_size=1)
        
        self.robot1_comm_pub = rospy.Publisher(node_name +"/robot1_communication",
                                Communication,
                                queue_size=1)
        
        self.robot2_comm_pub = rospy.Publisher(node_name +"/robot2_communication",
                                Communication,
                                queue_size=1)

        #Subscribers
        for robot in range(self.number_of_robots):
            rospy.Subscriber(
                '/robot'+str(robot)+'/batteries/status',
                BatteryState,
                self.update_battery_state,
                robot,
                queue_size=1) 
            
        for robot in range(self.number_of_robots+1):# add one in order to get info from the ASV too
            rospy.Subscriber(
                '/robot'+str(robot)+'/navigator/navigation',
                NavSts,
                self.update_robot_position,
                robot,
                queue_size=1) 
                 
        rospy.Subscriber("reset_storage_disk",
                         Int16,    
                         self.reset_values,
                         queue_size=1)
        
        rospy.Subscriber("robot_distances",
                    Distances,    
                    self.update_distance,
                    queue_size=1)
            
        self.round_robots = np.array([],dtype=np.uint32)

        for robot in range(self.number_of_robots):
            self.round_robots = np.append(self.round_robots,robot)
            self.distances.append(0)
    
    def update_distance(self,msg):
        self.distances[msg.auv_id] = msg.distance      

    def update_battery_state(self,msg,robot_id): 
        self.battery_charge[robot_id]= msg.charge
    
    def update_robot_position(self, msg, robot_id):
        # fill the robots_information array with the robots information received from the NavSts 
        self.robots_information[robot_id][0] = msg.position.north
        self.robots_information[robot_id][1] = msg.position.east
        self.robots_information[robot_id][2] = msg.position.depth
        self.robots_information[robot_id][3] = msg.altitude
        self.robots_information[robot_id][4] = msg.global_position.latitude
        self.robots_information[robot_id][5] = msg.global_position.longitude
        self.robots_information[robot_id][6] = msg.body_velocity.x
        self.robots_information[robot_id][7] = msg.body_velocity.y
        self.robots_information[robot_id][8] = msg.body_velocity.z
        self.robots_information[robot_id][9] = msg.orientation.roll
        self.robots_information[robot_id][10] = msg.orientation.pitch
        self.robots_information[robot_id][11] = msg.orientation.yaw

        # check the system initialization
        if(self.system_init == False):
            self.initialization(robot_id)
        else:
            self.communication_process()

    def initialization(self,robot_id):
        # check if all the n robots are publishing their information
        if(self.robots_information[[robot_id][0]] != 0): 
            self.robot_initialization[robot_id] = True
        
        if((self.robot_initialization == True).all()):
            self.system_init = True
   
    def get_storage_disk(self,robot_id):
        occupied_memory = random.randint(1,20)
        new_value = self.storage_disk[robot_id]+abs(occupied_memory*(robot_id+1))
        self.storage_disk[robot_id] = new_value
        return(new_value)
    
    def get_comm_freq(self,distance):
        # y = -0.0000001 + 0.0000776*x^1 + -0.0141229*x^2 + 0.8483710*x^3
        communication_freq = 0.848371 - 0.01412292*distance + 0.00007763495*distance**2
        # Create a rate
        self.rate = rospy.Rate(communication_freq)
        return(communication_freq)

    def communication_process(self):       
        self.round_robots = np.roll(self.round_robots,1)
        target = self.round_robots[0]
        self.auv_id = target
        self.asv_id = 3
        distance = self.distances[target]
        self.communication_freq = self.get_comm_freq(distance)
        communication_msg = Communication()
        communication_msg.header.frame_id = "multi_robot_system"
        communication_msg.header.stamp = rospy.Time.now()
        communication_msg.auv_north = self.robots_information[self.auv_id][0]
        communication_msg.auv_east = self.robots_information[self.auv_id][1]
        communication_msg.battery_charge = self.battery_charge[self.auv_id]
        communication_msg.asv_id = self.asv_id
        communication_msg.auv_id = self.auv_id
        communication_msg.communication_freq = self.communication_freq
        storage = self.get_storage_disk(self.auv_id)
        communication_msg.storage_disk = storage

        if(self.auv_id==0):
            self.robot0_comm_pub.publish(communication_msg)
        elif(self.auv_id==1):
            self.robot1_comm_pub.publish(communication_msg)
        else:
            self.robot2_comm_pub.publish(communication_msg)
        
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
        rospy.init_node('communications')
        communications = communications(rospy.get_name())
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass


