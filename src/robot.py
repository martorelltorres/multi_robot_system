#!/usr/bin/env python
from dis import dis
import rospy            
import actionlib
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt
from math import *
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
from cola2_msgs.msg import  NavSts
from cola2_msgs.srv import Goto, GotoRequest
from sensor_msgs.msg import BatteryState
import numpy as np
from std_srvs.srv import Empty, EmptyResponse
from multi_robot_system.msg import TravelledDistance, CoverageStartTime



class Robot:

    def __init__(self, name):
        self.name = name
        self.tolerance = self.get_param('tolerance',2)
        self.surge_velocity = self.get_param('surge_velocity',0.5)
        self.battery_topic = self.get_param('~battery_topic','/turbot1/batteries/status')
        self.section_action = self.get_param('~section_action','/xiroi/pilot/world_section_req') 
        self.section_result = self.get_param('~section_result','/xiroi/pilot/world_section_req/result') 
        self.number_of_robots = self.get_param('number_of_robots')
        self.navigation_depth = self.get_param('~navigation_depth',10)
        self.robot_ID = self.get_param('~robot_ID',0)
        self.robot_name = self.get_param('~robot_name','turbot1')
        self.distance = []
        self.travelled_distance = []
        self.robots_travelled_distances = [0,0,0]
        self.robots_information = [[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0]]
        # self.distance_travelled
        self.robot_alive = False
        self.is_section_actionlib_running = False
        self.battery_status = [0,0,0]
        self.first_time = True
        self.ns = rospy.get_namespace()

        # robot_data = [0,0,0,0,0,0,0,0,0,0,0,0]
        # self.robots_information = []
        # self.robots = []
        # self.robot_initialization = np.array([])

        # for robot in range(self.number_of_robots):
        #     self.robots_information.append(robot_data) #set the self.robots_information initialized to 0
        #     # self.robot_initialization = np.append(self.robot_initialization,False) # self.robot_initialization = [False,False;False]
        #     # self.robots.append(robot)  # self.robots = [0,1,2]
        #     print("***********---------------------")
        #     print(self.robots_information)
        

        #Subscribers
        for robot in range(self.number_of_robots):
            rospy.Subscriber(
                '/robot'+str(robot+1)+'/navigator/navigation',
                NavSts,
                self.update_robot_position,
                robot,
                queue_size=1)
            
       
        #Publishers
        self.travelled_distance_pub = rospy.Publisher('/robot'+str(self.robot_ID)+'/travelled_distance',
                                        TravelledDistance,
                                        queue_size=1)
        # Services clients
        try:
            rospy.wait_for_service('/robot'+str(self.robot_ID+1)+'/captain/enable_goto', 20)
            self.goto_srv = rospy.ServiceProxy(
                        '/robot'+str(self.robot_ID+1)+'/captain/enable_goto', Goto)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to goto service',
                         self.name)
            rospy.signal_shutdown('Error creating client to goto service')
        
        try:
            rospy.wait_for_service('/robot'+str(self.robot_ID+1)+'/captain/disable_all_and_set_idle', 20)
            self.disable_all_and_set_idle_srv = rospy.ServiceProxy(
                        '/robot'+str(self.robot_ID+1)+'/captain/disable_all_and_set_idle', Trigger)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to disable_all_and_set_idle service',
                         self.name)
            rospy.signal_shutdown('Error creating client to disable_all_and_set_idle service')

        # Init periodic timers
        rospy.Timer(rospy.Duration(1.0), self.update_travelled_distance)

        #Actionlib section client
        self.section_strategy = actionlib.SimpleActionClient(self.section_action, WorldSectionAction)
        self.section_strategy.wait_for_server()

    def update_travelled_distance(self,event):
        if (self.first_time == True):
            self.x_old_position = 0
            self.y_old_position = 0
            self.x_current_position = self.robots_information[self.robot_ID][0]
            self.y_current_position = self.robots_information[self.robot_ID][1]

            self.travelled_distance = self.update_distance()
            self.robots_travelled_distances[self.robot_ID] = self.travelled_distance
            self.first_time = False

        else:
            self.x_old_position = self.x_current_position
            self.y_old_position = self.y_current_position
            self.x_current_position = self.robots_information[self.robot_ID][0]
            self.y_current_position = self.robots_information[self.robot_ID][1]
            self.travelled_distance = self.update_distance()
            self.robots_travelled_distances[self.robot_ID] = self.travelled_distance

        # publish the data
        msg = TravelledDistance()
        msg.header.stamp = rospy.Time.now()
        msg.travelled_distance = self.travelled_distance 
        self.travelled_distance_pub.publish(msg)

    def update_distance(self):
        x_diff =  self.x_current_position - self.x_old_position
        y_diff =  self.y_current_position - self.y_old_position 
        distance =  sqrt(x_diff**2 + y_diff**2)
        travelled_distance = self.robots_travelled_distances[self.robot_ID] + distance
        # self.robots_travelled_distances[id] = self.robots_travelled_distances[id] + distance
        return travelled_distance

    
    def simulation_task_time (self,init_time, final_time):
        spend_time = (final_time - init_time)/1000000000
        return(spend_time)

    def send_goto_strategy(self, position_x, position_y,keep_position):
        # self.disable_all_and_set_idle_srv()
        """Goto to position x, y, z, at velocity vel."""
        # // Define waypoint attributes
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = self.surge_velocity
        goto_req.position.x = position_x
        goto_req.position.y = position_y
        goto_req.position.z = 0.0
        goto_req.position_tolerance.x = 5
        goto_req.position_tolerance.y = 5
        goto_req.position_tolerance.z = 5
        goto_req.blocking = True
        goto_req.keep_position = keep_position
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
        section_req.initial_position.z = self.navigation_depth
        section_req.initial_yaw = self.robots_information[robot_id][2] #yaw
        section_req.final_position.x = final_position_x
        section_req.final_position.y = final_position_y
        section_req.final_position.z = self.navigation_depth
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
        
    def set_current_section(self,current_section):
        return(current_section)

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