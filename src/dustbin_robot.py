#!/usr/bin/env python
import rospy
import math
import numpy as np
import random 
import actionlib         
import matplotlib.pyplot as plt
from cola2_msgs.msg import  NavSts
from shapely.geometry import Point
from multi_robot_system.msg import TaskMonitoring
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
#import classes
from area_partition import area_partition
 
class DustbinRobot:
  
    def __init__(self, name):
        """ Init the class """

        self.name = name
        # Get config parameters from the parameter server
        self.number_of_robots = self.get_param('number_of_robots')
        self.robot_ID = self.get_param('~robot_ID',0) 
        self.tolerance = self.get_param('tolerance',2)
        self.surge_velocity = self.get_param('surge_velocity',0.5)
        self.section_action = self.get_param('section_action','/robot4/pilot/world_section_req') 
        self.section_result = self.get_param('section_result','/robot4/pilot/world_section_req/result') 
        self.is_section_actionlib_running = False
        self.robot_at_center = False
        self.area_handler =  area_partition("area_partition")
        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        # Initialize some variables
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

        #Subscribers
        for robot_id in range(self.number_of_robots):
            rospy.Subscriber(
                '/robot'+str(robot_id+1)+'/navigator/navigation',
                NavSts,
                self.update_robots_position,
                robot_id,
                queue_size=1) 

        rospy.Subscriber('/robot'+str(self.robot_ID)+'/navigator/navigation',
                            NavSts,    
                            self.update_robot_position,
                            queue_size=1)

        rospy.Subscriber(self.section_result,
                         WorldSectionActionResult,    
                         self.update_section_result,
                         queue_size=1)
        #Actionlib section client
        self.section_strategy = actionlib.SimpleActionClient(self.section_action, WorldSectionAction)
        self.section_strategy.wait_for_server() 

    def update_robot_position(self, msg):
        self.north_position = msg.position.north
        self.east_position = msg.position.east      
        self.yaw = msg.orientation.yaw

    def update_section_result(self,msg):
        self.final_status = msg.result.final_status
 
    def update_robots_position(self, msg, robot_id):
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
            # go to the central area
            if(self.robot_at_center == False):
                self.goto_central_point()
            else:
                self.dustbin_strategy()
 
    def initialization(self,robot_id):
        # check if all the n robots are publishing their information
        if(self.robots_information[[robot_id][self.robot_data[0]]] != 0):
            self.robot_initialization[robot_id] = True
        
        if((self.robot_initialization == True).all()):
            self.system_init = True
  
    def dustbin_strategy(self):
        print("do a lot of TODO's")

    
    def goto_central_point(self):
        self.central_point = self.area_handler.get_main_polygon_centroid()
        initial_point = [self.north_position,self.east_position]
        final_point = [self.central_point.x,self.central_point.y]
        self.send_section_strategy(initial_point,final_point,self.robot_ID)
        self.wait_until_section_reached()
        self.robot_at_center = True
    
    def wait_until_section_reached(self):
        if(self.final_status==0):
            self.success_result = True 


    def send_section_strategy(self,initial_point,final_point,robot_id):
        initial_position_x = initial_point[0]
        final_position_x = final_point[0]
        initial_position_y = initial_point[1]
        final_position_y = final_point[1]

        section_req = WorldSectionGoal()
        section_req.initial_position.x = initial_position_x
        section_req.initial_position.y = initial_position_y
        section_req.initial_position.z = 0.0
        section_req.initial_yaw = self.yaw #yaw
        section_req.final_position.x = final_position_x
        section_req.final_position.y = final_position_y
        section_req.final_position.z = 0.0
        section_req.altitude_mode = False
        section_req.tolerance.x = self.tolerance
        section_req.tolerance.y = self.tolerance
        section_req.tolerance.z = self.tolerance
        section_req.controller_type = WorldSectionGoal.LOSCTE
        section_req.priority = GoalDescriptor.PRIORITY_NORMAL
        section_req.surge_velocity = self.surge_velocity
        section_req.timeout = 6000

        # send section goal using actionlib
        self.success_result = False
        self.is_section_actionlib_running = True
        self.section_strategy.send_goal(section_req)

        #  Wait for result or cancel if timed out
        self.section_strategy.wait_for_result()
 
  
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

