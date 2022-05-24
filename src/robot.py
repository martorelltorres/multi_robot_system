#!/usr/bin/env python
import rospy            
import actionlib
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
from cola2_msgs.msg import  NavSts


class robot:

    def __init__(self, name):
        self.name = name
        self.ned_origin_lat = get_param(self,'/turbot/navigator/ned_latitude')
        self.ned_origin_lon = get_param(self,'/turbot/navigator/ned_longitude')
        self.navigation_topic = self.get_param('~navigation_topic','/turbot/navigator/navigation')
        self.offset_distance = 0
        self.initial_offset = 1
        self.intersection_points =[]
        self.distance = []
        self.first_time = True
        self.ns = rospy.get_namespace()
        robot_position_updated = False

        rospy.Subscriber(self.navigation_topic ,
                    NavSts,    
                    self.update_robot_position,
                    queue_size=1)

    def robot_init(self,msg):
        return(True)
    
    def update_robot_position(self,msg):
        self.first_time = False
        self.yaw = msg.orientation.yaw
        self.north = msg.position.north
        self.east = msg.position.east
        self.depth = msg.position.depth
        robot_position_updated = True
        
        if(self.first_time == True):
            return(robot_position_updated)

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