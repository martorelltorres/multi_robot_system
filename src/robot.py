#!/usr/bin/env python
from dis import dis
import rospy            
import actionlib
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
from cola2_msgs.msg import  NavSts
import numpy as np
from std_srvs.srv import Empty, EmptyResponse


class robot:

    def __init__(self, name):
        self.name = name
        self.ned_origin_lat = self.get_param(self,'/turbot/navigator/ned_latitude')
        self.ned_origin_lon = self.get_param(self,'/turbot/navigator/ned_longitude')
        self.tolerance = self.get_param('tolerance',2)
        self.surge_velocity = self.get_param('surge_velocity',0.5)
        self.navigation_topic = self.get_param('~navigation_topic','/turbot/navigator/navigation') 
        self.section_action = self.get_param('~section_action','/xiroi/pilot/world_section_req') 
        self.section_result = self.get_param('~section_result','/xiroi/pilot/world_section_req/result') 
        self.robot_ID = self.get_param('~robot_ID',0)
        self.robot_name = self.get_param('~robot_name','turbot')
        self.robot_slave_name = rospy.get_param('~robot_slave_name',default='xiroi')
        self.distance = []
        self.is_section_actionlib_running = False
        self.ns = rospy.get_namespace()
        
        #  # enable thrusters service
        # rospy.wait_for_service(str(self.robot_name)+'/controller/enable_thrusters', 10)
        # try:
        #     self.enable_thrusters_srv = rospy.ServiceProxy(
        #                 str(self.robot_name) + '/controller/enable_thrusters', Empty)
        # except rospy.ServiceException, e:
        #     rospy.logwarn("%s: Service call failed: %s", self.name, e)

        # # disable thrusters service
        # rospy.wait_for_service(str(self.robot_name)+'/controller/disable_thrusters', 10)
        # try:
        #     self.disable_thrusters_srv = rospy.ServiceProxy(
        #                 str(self.robot_name)+ '/controller/disable_thrusters', 
        #                 Empty)
        # except rospy.ServiceException, e:
        #     rospy.logwarn("%s: Service call failed: %s", self.name, e)


        # subscribers
        rospy.Subscriber(self.navigation_topic ,
                    NavSts,    
                    self.get_robot_position,
                    queue_size=1)

        #Actionlib section client
        self.section_strategy = actionlib.SimpleActionClient(self.section_action, WorldSectionAction)
        self.section_strategy.wait_for_server()

    def disable_thrusters(self,robot_name):
        if(robot_name == self.robot_slave_name):
            self.disable_thrusters_srv()

    # def enable_thrusters(self,robot_name):
    #     if(robot_name == self.robot_slave_name):
    #         self.enable_thrusters_srv()
    
    def send_section_strategy(self,initial_point,final_point):
        initial_position_x = initial_point[0]
        final_position_x = final_point[0]
        initial_position_y = initial_point[1]
        final_position_y = final_point[1]

        section_req = WorldSectionGoal()
        section_req.initial_position.x = initial_position_x
        section_req.initial_position.y = initial_position_y
        section_req.initial_position.z = 0.0
        section_req.initial_yaw = self.robot_orientation_yaw
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

    def get_robot_id(self):
        return(self.robot_ID)

    def cancel_section_strategy(self,section):
        if self.is_section_actionlib_running==True:
            # print("------------------------" + str(section) +"-----------------------------")
            self.section_strategy.cancel_goal()
            section_cancelled = True
        return(section_cancelled)
   
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
        self.robot_orientation_yaw = msg.orientation.yaw 
        robot_id = self.robot_ID
        return(robot_position_north,robot_position_east,robot_position_depth,self.robot_orientation_yaw,robot_id)
    
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