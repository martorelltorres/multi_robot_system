#!/usr/bin/env python
from dis import dis
import rospy            
import actionlib
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
from cola2_msgs.msg import  NavSts
from cola2_msgs.srv import Goto, GotoRequest
from sensor_msgs.msg import BatteryState
import numpy as np
from std_srvs.srv import Empty, EmptyResponse


class Robot:

    def __init__(self, name):
        self.name = name
        self.ned_origin_lat = self.get_param('ned_origin_lat')
        self.ned_origin_lon = self.get_param('ned_origin_lon')
        self.tolerance = self.get_param('tolerance',2)
        self.surge_velocity = self.get_param('surge_velocity',0.5)
        self.navigation_topic = self.get_param('~navigation_topic','/turbot/navigator/navigation') 
        self.battery_topic = self.get_param('~battery_topic','/turbot1/batteries/status')
        self.section_action = self.get_param('~section_action','/xiroi/pilot/world_section_req') 
        self.section_result = self.get_param('~section_result','/xiroi/pilot/world_section_req/result') 
        self.number_of_robots = self.get_param('number_of_robots')
        self.robot_ID = self.get_param('~robot_ID',0)
        self.robot_name = self.get_param('~robot_name','turbot1')
        self.robot_slave_name = rospy.get_param('~robot_slave_name',default='xiroi')
        self.distance = []
        self.robot0_bat = False
        self.robot1_bat = False
        self.robot2_bat = False
        self.robot_alive = False
        self.is_section_actionlib_running = False
        self.battery_status = [0,0,0]
        self.ns = rospy.get_namespace()

        robot_data = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.robots_information = []
        # self.robots = []
        # self.robot_initialization = np.array([])

        for robot in range(self.number_of_robots):
            self.robots_information.append(robot_data) #set the self.robots_information initialized to 0
            # self.robot_initialization = np.append(self.robot_initialization,False) # self.robot_initialization = [False,False;False]
            # self.robots.append(robot)  # self.robots = [0,1,2]
        

        # subscribers
        for robot in range(self.number_of_robots):
            rospy.Subscriber(
                '/robot'+str(robot+1)+'/navigator/navigation',
                NavSts,
                self.update_robot_position,
                robot,
                queue_size=1)

        rospy.Subscriber(self.battery_topic ,
                    BatteryState,    
                    self.update_battery_status,
                    queue_size=1)

        #Actionlib section client
        self.section_strategy = actionlib.SimpleActionClient(self.section_action, WorldSectionAction)
        self.section_strategy.wait_for_server()
    
    def simulation_task_time (self,init_time, final_time):
        spend_time = (final_time - init_time)/1000000000
        return(spend_time)


    def disable_thrusters(self,robot_name):
        if(robot_name == self.robot_slave_name):
            self.disable_thrusters_srv()

    # def enable_thrusters(self,robot_name):
    #     if(robot_name == self.robot_slave_name):
    #         self.enable_thrusters_srv()

    def send_goto_strategy(self, position_x, position_y,linear_velocity):
        """Goto to position x, y, z, at velocity vel."""
        # // Define waypoint attributes
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = linear_velocity
        goto_req.position.x = position_x
        goto_req.position.y = position_y
        goto_req.position.z = 0.0
        goto_req.position_tolerance.x = 2
        goto_req.position_tolerance.y = 2
        goto_req.position_tolerance.z = 2
        goto_req.blocking = True
        goto_req.keep_position = False
        goto_req.disable_axis.x = False
        goto_req.disable_axis.y = True
        goto_req.disable_axis.z = False
        goto_req.disable_axis.roll = True
        goto_req.disable_axis.yaw = False
        goto_req.disable_axis.pitch = True
        goto_req.priority = 10
        goto_req.reference = 0 #REFERENCE_NED=0  REFERENCE_GLOBAL=1 REFERENCE_VEHICLE=2
        self.goto_srv(goto_req)
        rospy.sleep(1.0)
    
    def send_section_strategy(self,initial_point,final_point,robot_id):
        initial_position_x = initial_point[0]
        final_position_x = final_point[0]
        initial_position_y = initial_point[1]
        final_position_y = final_point[1]

        section_req = WorldSectionGoal()
        section_req.initial_position.x = initial_position_x
        section_req.initial_position.y = initial_position_y
        section_req.initial_position.z = 0.0
        section_req.initial_yaw = self.robots_information[robot_id][2] #yaw
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

    def update_battery_status(self, msg):
        self.battery_charge = msg.charge
        self.get_battery_status()
    
    def get_battery_status(self):
        self.battery_status[self.robot_ID-1] =  self.battery_charge
        return(self.battery_charge)
         
    def update_section_result(self,msg):
        final_status = msg.result.final_status
        return(final_status)
        # Possible section ending conditions
        # uint64 final_status
        # uint64 SUCCESS=0
        # uint64 TIMEOUT=1
        # uint64 FAILURE=2
        # uint64 BUSY=3

    def update_robot_position(self,msg,robot_id):
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
        self.robot_id = robot_id

    
    def get_robot_position(self,robot_id):
        return(self.robots_information[robot_id][0],self.robots_information[robot_id][1],self.robots_information[robot_id][2],self.robots_information[robot_id][11])
    
    def is_robot_alive(self, ID_robot):
        if(ID_robot ==self.robot_ID and self.robot_alive==False):
            return(False)
        elif(ID_robot ==self.robot_ID and self.robot_alive==True):
            return(True)

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