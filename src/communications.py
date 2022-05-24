#!/usr/bin/env python
import rospy            
import actionlib
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
from cola2_msgs.msg import  NavSts
from robot import robot
import numpy as np



class communications:

    def __init__(self, name):
        self.name = name
        self.navigation_topic_r0 = get_param(self,'~navigation_topic_r0')
        self.navigation_topic_r1 = get_param(self,'~navigation_topic_r1')
        self.offset_distance = 0
        self.initial_offset = 1
        self.intersection_points =[]
        self.distance = []
        self.ns = rospy.get_namespace()
        self.robot_handler = robot("robot")

        #Subscribers
        rospy.Subscriber(self.navigation_topic_r0,
                         NavSts,    
                         self.update_robot_position_r0,
                         queue_size=1)

        rospy.Subscriber(self.navigation_topic_r1,
                        NavSts,    
                        self.update_robot_position_r1,
                        queue_size=1)


def update_robot_position_r0(self, msg):
    self.r0_position_north = msg.position.north
    self.r0_position_east = msg.position.east
    self.r0_position_depth = msg.position.depth
    self.r0_yaw = msg.orientation.yaw

def update_robot_position_r1(self, msg):
    self.r1_position_north = msg.position.north
    self.r1_position_east = msg.position.east
    self.r1_position_depth = msg.position.depth
    self.r1_yaw = msg.orientation.yaw

def distance_between_robots(self):
    x_distance = abs(self.r1_position_north - self.r0_position_north)
    y_distance = abs(self.r1_position_east - self.r0_position_east)
    self.distance = np.sqrt(x_distance**2 + y_distance**2)
    return(self.distance)
  
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
        robot = robot(rospy.get_name())
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass


