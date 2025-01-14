#!/usr/bin/env python
from dis import dis
import rospy
import roslib            
import actionlib
from cola2.utils.ned import NED
import matplotlib.pyplot as plt
from math import *
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32
from cola2_msgs.msg import GoalDescriptor, CaptainStatus, CaptainStateFeedback
from cola2_msgs.msg import PilotActionResult, PilotAction, PilotGoal
from cola2_msgs.msg import  NavSts
from cola2_msgs.srv import Goto, GotoRequest, Section, SectionRequest
from shapely.geometry import Polygon
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
        self.ned_origin_lat = self.get_param('ned_origin_lat',39.543330)
        self.ned_origin_lon = self.get_param('ned_origin_lon',2.377940)
        self.robot_ID = self.get_param('~robot_ID',0)
        self.robot_name = self.get_param('~robot_name','turbot1')
        self.distance = []
        self.travelled_distance = []
        self.robots_travelled_distances = [0,0,0,0,0,0]
        self.robot_alive = False
        self.is_section_actionlib_running = False
        self.battery_status = [0,0,0]
        self.first_time = True
        self.ns = rospy.get_namespace()
        robot_data = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.robots_information = []

        for robot in range(self.number_of_robots):
            self.robots_information.append(robot_data) #set the self.robots_information initialized to 0
           
        self.ned = NED(self.ned_origin_lat, self.ned_origin_lon, 0.0)  # NED frame
        #Publishers
        self.travelled_distance_pub = rospy.Publisher('/robot'+str(self.robot_ID)+'/travelled_distance',
                                        TravelledDistance,
                                        queue_size=1)

        # rospy.Subscriber('/robot'+str(self.robot_ID)+'/captain/state_feedback',
        #         CaptainStateFeedback,    
        #         self.update_section_status,
        #         queue_size=1)

        # rospy.Subscriber('/robot'+str(self.robot_ID)+'/pilot/actionlib/result',
        #         PilotActionResult,    
        #         self.update_section_feedback,
        #         queue_size=1)

        #Actionlib section client
        self.section_strategy = actionlib.SimpleActionClient('/robot'+str(self.robot_ID)+'/pilot/actionlib',PilotAction)
        self.section_strategy.wait_for_server()
        # Services clients
        # goto
        try:
            rospy.wait_for_service('/robot'+str(self.robot_ID)+'/captain/enable_goto', 20)
            self.goto_srv = rospy.ServiceProxy(
                        '/robot'+str(self.robot_ID)+'/captain/enable_goto', Goto)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to goto service',
                         self.name)
            rospy.signal_shutdown('Error creating client to goto service')
        # section
        try:
            rospy.wait_for_service('/robot'+str(self.robot_ID)+'/captain/enable_section', 20)
            self.section_srv = rospy.ServiceProxy(
                        '/robot'+str(self.robot_ID)+'/captain/enable_section', Section)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to Section service',
                         self.name)
            rospy.signal_shutdown('Error creating client to Section service')
        
        # Init periodic timers
        rospy.Timer(rospy.Duration(1.0), self.update_travelled_distance)

    
    def disable_all_and_set_idle(self,robot_id):
        """ This method sets the captain back to idle """
        rospy.loginfo("Setting captain to idle state")
        try:
            rospy.wait_for_service('/robot'+str(robot_id)+'/captain/disable_all_and_set_idle', 20)
            self.disable_all_and_set_idle_srv = rospy.ServiceProxy(
                        '/robot'+str(robot_id)+'/captain/disable_all_and_set_idle', Trigger)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to disable_all_and_set_idle service',
                         self.name)
            rospy.signal_shutdown('Error creating client to disable_all_and_set_idle service')
    
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
        # goto_req.altitude = 0
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
        goto_req.priority = 11
        goto_req.reference = 0 #REFERENCE_NED=0  REFERENCE_GLOBAL=1 REFERENCE_VEHICLE=2
        self.goto_srv(goto_req)
        rospy.sleep(1.0)

    # PRIORITY DEFINITIONS
    # uint32 PRIORITY_TELEOPERATION_LOW = 0
    # uint32 PRIORITY_SAFETY_LOW = 5
    # uint32 PRIORITY_NORMAL = 10
    # uint32 PRIORITY_SAFETY = 30
    # uint32 PRIORITY_TELEOPERATION = 40
    # uint32 PRIORITY_SAFETY_HIGH  = 50
    # uint32 PRIORITY_TELEOPERATION_HIGH = 60

    def send_section_strategy(self,initial_point,final_point,robot_id):
        initial_position_x = initial_point[0]
        final_position_x = final_point[0]
        initial_position_y = initial_point[1]
        final_position_y = final_point[1]

        init_lat, init_lon, _ = self.ned.ned2geodetic([initial_position_x, initial_position_y, 0.0])
        final_lat, final_lon, _ = self.ned.ned2geodetic([final_position_x, final_position_y, 0.0])

        section_req = PilotGoal()
        section_req.initial_latitude = init_lat
        section_req.initial_longitude = init_lon
        section_req.initial_depth = self.navigation_depth
        # section_req.initial_yaw = self.robots_information[robot_id][2] #yaw
        section_req.final_latitude = final_lat
        section_req.final_longitude = final_lon
        section_req.final_depth = self.navigation_depth
        section_req.final_altitude = self.navigation_depth

        section_req.heave_mode = 0
        # uint64 DEPTH=0
        # uint64 ALTITUDE=1
        # uint64 BOTH=2
        section_req.tolerance_xy = self.tolerance
        section_req.surge_velocity = self.surge_velocity
        section_req.controller_type = 0
        # uint64 SECTION=0
        # uint64 ANCHOR=1
        # uint64 HOLONOMIC_KEEP_POSITION=2
        section_req.goal.priority = GoalDescriptor.PRIORITY_SAFETY_HIGH
        section_req.goal.requester = rospy.get_name()
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
        self.battery_status[self.robot_ID] =  self.battery_charge
        return(self.battery_charge)
         
    def update_section_status(self,msg):
        if(msg.state==0):
            self.section_active = True
        self.check_section_status
    
    def update_section_feedback(self,msg):
        if(msg.state==0):
            self.section_succes = True
        self.check_section_status

    def check_section_status():
        if(self.section_active == self.section_succes == True):
            return(True)
        else:
            return(False)

  
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