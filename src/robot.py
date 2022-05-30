#!/usr/bin/env python
from dis import dis
import rospy            
import actionlib
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
from cola2_msgs.msg import  NavSts
import numpy as np


class robot:

    def __init__(self, name):
        self.name = name
        self.ned_origin_lat = self.get_param(self,'/turbot/navigator/ned_latitude')
        self.ned_origin_lon = self.get_param(self,'/turbot/navigator/ned_longitude')
        self.navigation_topic = self.get_param('~navigation_topic','/turbot/navigator/navigation') 
        self.offset_distance = 0
        self.initial_offset = 1
        self.intersection_points =[]
        self.distance = []
        self.ns = rospy.get_namespace()
        robot_position_updated = False

        rospy.Subscriber(self.navigation_topic ,
                    NavSts,    
                    self.get_robot_position,
                    queue_size=1)
   
    def update_section_result(self,msg):
        final_status = msg.result.final_status
        return(final_status)
        # Possible section ending conditions
        # uint64 final_status
        # uint64 SUCCESS=0
        # uint64 TIMEOUT=1
        # uint64 FAILURE=2
        # uint64 BUSY=3

    def get_robot_position(self,msg):
        robot_position_north = msg.position.north
        robot_position_east = msg.position.east
        robot_position_depth = msg.position.depth
        robot_orientation_yaw = msg.orientation.yaw 
        return(robot_position_north,robot_position_east,robot_position_depth,robot_orientation_yaw)
    
    def get_robot_distance_to_point(self,robot_north, robot_east, point_x, point_y):
        x_distance = robot_north - point_x
        y_distance = robot_east - point_y
        distance = np.sqrt(x_distance**2 + y_distance**2)
        return(distance)
  
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
        rospy.init_node('robot')
        robot = robot(rospy.get_name())
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass