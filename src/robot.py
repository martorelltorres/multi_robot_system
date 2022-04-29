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
        self.ned_origin_lat = get_param(self,'/xiroi/navigator/ned_latitude')
        self.ned_origin_lon = get_param(self,'/xiroi/navigator/ned_longitude')
        self.offset_distance = 0
        self.initial_offset = 1
        self.intersection_points =[]
        self.distance = []
        self.ns = rospy.get_namespace()

        #Sevice server

        #Publishers

        #Subscribers
        rospy.Subscriber(self.ns + "navigator/navigation",
                         NavSts,    
                         self.get_robot_position,
                         queue_size=1)

        rospy.Subscriber(self.ns + "pilot/world_section_req/result",
                    WorldSectionActionResult,    
                    self.update_section_result,
                    queue_size=1)
        
        #Actionlib section client
        self.section_action = actionlib.SimpleActionClient(self.ns + "pilot/world_section_req", WorldSectionAction)
        self.section_action.wait_for_server()

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

    def send_section_strategy(self,initial_point,final_point,yaw):
        initial_position_x = initial_point[0]
        final_position_x = final_point[0]
        initial_position_y = initial_point[1]
        final_position_y = final_point[1]

        section_req = WorldSectionGoal()
        section_req.initial_position.x = initial_position_x
        section_req.initial_position.y = initial_position_y
        section_req.initial_position.z = 10
        section_req.initial_yaw = yaw
        section_req.final_position.x = final_position_x
        section_req.final_position.y = final_position_y
        section_req.final_position.z = 10
        section_req.altitude_mode = False
        section_req.tolerance.x = 2
        section_req.tolerance.y = 2
        section_req.tolerance.z = 2
        section_req.controller_type = WorldSectionGoal.LOSCTE
        section_req.priority = GoalDescriptor.PRIORITY_NORMAL
        section_req.surge_velocity = 0.3
        section_req.timeout = 6000
        # send section goal using actionlib
        self.section_action.send_goal(section_req)
        #  Wait for result or cancel if timed out
        self.section_action.wait_for_result()

  
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