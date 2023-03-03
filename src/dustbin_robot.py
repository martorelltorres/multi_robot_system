#!/usr/bin/env python
import rospy
import math
import numpy as np
import random 
from math import *
import matplotlib
import actionlib       
import matplotlib.pyplot as plt
from std_msgs.msg import Int16
from cola2_msgs.msg import  NavSts,BodyVelocityReq
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from cola2_msgs.srv import Goto, GotoRequest
from multi_robot_system.msg import CoverageStartTime,CommunicationDelay,ExplorationUpdate
import os
import sys
import subprocess
#import classes
from area_partition import area_partition
 
class DustbinRobot:
  
    def __init__(self, name):
        """ Init the class """
        rospy.sleep(7)
        self.name = name

        # Get config parameters from the parameter server
        self.number_of_robots = self.get_param('number_of_robots')
        self.robot_ID = self.get_param('~robot_ID',0) 
        self.tolerance = self.get_param('tolerance',2)
        self.surge_velocity = self.get_param('surge_velocity',0.5)
        self.section_action = self.get_param('section_action','/robot4/pilot/world_section_req') 
        self.section_result = self.get_param('section_result','/robot4/pilot/world_section_req/result') 
        self.repulsion_radius = self.get_param("repulsion_radius",50)
        self.adrift_radius = self.get_param("adrift_radius",55)
        self.tracking_radius = self.get_param("tracking_radius",80)
        self.dutsbin_timer = self.get_param("dutsbin_timer",1)


        # Import classes
        self.area_handler =  area_partition("area_partition")

        # Initialize some variables
        self.pose = [0,0]
        self.get_information = False
        self.robot_at_center = False
        self.robots_id = np.array([])
        self.communication_times_delay = [0,0,0]
        self.start_recording_time = [0,0,0]
        self.communication_times_end = [0,0,0]
        self.system_init = False
        self.robot_data = [0,0]
        self.robots_information = [[0,0],[0,0],[0,0]]
        self.robots = []
        self.robot_initialization = np.array([])
        self.enable_tracking = True
        self.set_end_time = True
        self.start_dustbin_strategy =np.array([False,False,False])
        self.first_time = True
        self.trigger = False
        self.exploration_tasks_update = np.array([])

        self.time_threshold = 400
        self.initial_storage_disk = 500
        self.AUV_trigger =[self.initial_storage_disk,self.initial_storage_disk,self.initial_storage_disk]
        self.stimulus = np.array([0,0,0],dtype = float)
        self.max_value = 100
        self.min_value = 1

        tasks = self.area_handler.get_polygon_number()

        for task in range(tasks):
            self.exploration_tasks_update = np.append(self.exploration_tasks_update,False)


        # initialize the robots variables
        for robot_ in range(self.number_of_robots):
            self.robot_initialization = np.append(self.robot_initialization,False) # self.robot_initialization = [False,False;False]
            self.robots.append(robot_)  
            self.robots_id = np.append(self.robots_id,robot_)
            self.robots_id = self.robots_id.astype(int) #convert float to int type
        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

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

        rospy.Subscriber('/mrs/exploration_area_update',
                            ExplorationUpdate,    
                            self.kill_the_process,
                            queue_size=1)
        
        rospy.Subscriber('/mrs/exploration_finished',
                            Int16,    
                            self.remove_robot_from_dustbin_goals,
                            queue_size=1)
        
        for robot in range(self.number_of_robots):
            rospy.Subscriber(
            '/mrs/robot'+str(robot)+'_start_coverage_time',
            CoverageStartTime,
            self.set_coverage_start_time,
            robot,
            queue_size=1)      

        #Publishers
        self.corrected_bvr = rospy.Publisher('/robot'+str(self.robot_ID)+'/controller/body_velocity_req',
                                                BodyVelocityReq,
                                                queue_size=1)

        self.markerPub_repulsion = rospy.Publisher('repulsion_radius',
                                                    Marker,
                                                    queue_size=1)

        self.markerPub_adrift = rospy.Publisher('adrift_radius',
                                                Marker,
                                                queue_size=1)

        self.markerPub_tracking = rospy.Publisher('tracking_radius',
                                                Marker,
                                                queue_size=1)
        
        self.communication_delay_time = rospy.Publisher("communication_time_delay",
                                        CommunicationDelay,
                                        queue_size=1)

        # Services clients
        try:
            rospy.wait_for_service('/robot'+str(self.robot_ID)+'/captain/enable_goto', 20)
            self.goto_srv = rospy.ServiceProxy(
                        '/robot'+str(self.robot_ID)+'/captain/enable_goto', Goto)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to goto service',
                         self.name)
            rospy.signal_shutdown('Error creating client to goto service')
        
        try:
            rospy.wait_for_service('/robot'+str(self.robot_ID)+'/captain/disable_all_and_set_idle', 20)
            self.disable_all_and_set_idle_srv = rospy.ServiceProxy(
                        '/robot'+str(self.robot_ID)+'/captain/disable_all_and_set_idle', Trigger)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to disable_all_and_set_idle service',
                         self.name)
            rospy.signal_shutdown('Error creating client to disable_all_and_set_idle service')
        
        # move the robot to the central area
        if (self.robot_at_center == False):
            self.goto_central_area()
        
        # Init periodic timers
        rospy.Timer(rospy.Duration(0.1), self.dustbin_trigger)

    
    def dustbin_trigger(self, event):
         # start the dustbin_strategy if all the has started the coverage
        if (np.all(self.start_dustbin_strategy) and self.first_time == True):
            self.dustbin_strategy()
            self.first_time = False
   
    def update_robot_position(self, msg):
        self.asv_north_position = msg.position.north
        self.asv_east_position = msg.position.east      
        self.asv_yaw = msg.orientation.yaw

        if(self.get_information==True and self.enable_tracking==True):
            self.tracking()     
    
    def set_coverage_start_time(self,msg,robot_id):
        self.start_dustbin_strategy[robot_id]= True
        self.t_start = msg.time.secs
        self.time_robot_id = msg.robot_id
        self.start_recording_time[self.time_robot_id] = self.t_start 

    def set_comm_start_time(self,time):
        self.t_start = time.secs
        self.start_recording_time[self.robot_goal_id] = self.t_start
    
    def set_comm_end_time(self,time):
        self.t_end = time.secs
        self.communication_times_end[self.robot_goal_id] = self.t_end
        self.get_comm_time_delay()
    
    def get_comm_time_delay(self):
        end_time = self.communication_times_end[self.robot_goal_id]
        start_time = self.start_recording_time[self.robot_goal_id]
        comm_delay = end_time - start_time
        self.communication_times_delay[self.robot_goal_id] =  comm_delay
        self.set_comm_start_time(rospy.Time.now())

        # check if there are zeros in the array
        if(self.communication_times_delay.count(0) == 0):
            self.trigger = True

        # publish the communication_delay_time
        msg = CommunicationDelay()
        msg.header.stamp = rospy.Time.now()
        msg.comm_delay = self.communication_times_delay 
        self.communication_delay_time.publish(msg)

    # *********************************************************************
    
    def get_time_threshold(self,robot_id):
        start_time = self.start_recording_time[robot_id]
        current_time = rospy.Time.now()
        time_spend = current_time.secs - start_time
        time_threshold = self.time_threshold-time_spend
        return(int(time_threshold))

    def get_distance(self, robot_id):
        x_diff =  self.asv_north_position - self.robots_information[robot_id][0]
        y_diff =  self.asv_east_position - self.robots_information[robot_id][1] 
        distance =  sqrt(x_diff**2 + y_diff**2)
        return(distance)

    def get_AUV_trigger(self,robot_id):
        occupied_memory = random.randint(0,4)
        new_value = self.AUV_trigger[robot_id]-occupied_memory
        self.AUV_trigger[robot_id] = new_value
    
       
    def normalize(self, values):
        normalized_values = np.array([])
        max_value = 1000
        min_value = 0.1
        for element in range(len(values)): 
            normalized_value = ((values[element]-min_value)/(max_value-min_value))*100
            normalized_values = np.append (normalized_values,normalized_value)
        return(normalized_values)

    def standarize(self, values):
        mean = np.mean(values)
        std_dev = np.std(values)
        standarized_values = np.array([])
        for element in range(len(values)):
            std_value = (values[element]-mean)/std_dev
            standarized_values = np.append(standarized_values,std_value)
        return(standarized_values)

    def min_max_scale(self,values):
        scaled_values = np.array([])
        for value in range(len(values)):
            calc =(values[value]- self.min_value)/(np.max(values)-self.min_value)*self.max_value
            scaled_values = np.append(scaled_values,calc)
        return(scaled_values)


    def get_stimulus(self,event):
        stimulus_variables = np.array([])

        for robot in range(self.number_of_robots):
            time_threshold = self.get_time_threshold(robot)
            stimulus_variables = np.append(stimulus_variables,time_threshold)

            distance = self.get_distance(robot)
            stimulus_variables = np.append(stimulus_variables,distance)

            self.get_AUV_trigger(robot)
            stimulus_variables = np.append(stimulus_variables,self.AUV_trigger[robot])
            
            # rescale the values 
            # normalized_values = self.normalize(stimulus_variables)
            # standar_values = self.standarize(stimulus_variables)
            min_max_scaled = self.min_max_scale(stimulus_variables)
                     

            # obtain the stimulus value
            self.stimulus[robot] = min_max_scaled[2] / ((min_max_scaled[1])*(min_max_scaled[0]))
            print("....................................................")
            # print("The stimulus inpust are: "+str(stimulus_variables))
            # print("The standarized values are: "+str(standar_values))
            print("The min_max_scaled values are: "+str(min_max_scaled))
            # print("normalized values are: "+str(normalized_values))
            print("the result stimulus is: "+str(self.stimulus[robot]))
            stimulus_variables = np.array([]) 
        
        # print("the stimulus are: "+str(self.stimulus[0])+" "+str(self.stimulus[1])+" "+str(self.stimulus[2]))            

        # extract the goal robot ID
        # max_stimulus_value = np.amax(self.stimulus)
        # goal_robot = np.where(self.stimulus==max_stimulus_value)
        # goal_robot = list(self.stimulus).index(max_stimulus_value)

        # print("The goal robot is : "+str(goal_robot)) 


    def time_trigger(self, event):
        # this function set the robot goal id for the dustbin_strategy
        # The trigger flag is True only when there are no zeros in the self.communication_times_delay array 
        if (self.trigger==True):
            self.set_comm_start_time(rospy.Time.now())
        
        self.robots_id = np.roll(self.robots_id,1)
        self.robot_goal_id = self.robots_id[0]      
        # flags to handle the data communication time delay
        self.get_information = True
        self.set_end_time = True
 
    def update_robots_position(self, msg, robot_id):
        # fill the robots_information array with the robots information received from the NavSts 
        self.robots_information[robot_id][0] = msg.position.north
        self.robots_information[robot_id][1] = msg.position.east

        # check the system initialization
        if(self.system_init == False):
            self.initialization(robot_id) 

    def dustbin_strategy(self):
        if(self.system_init==True):
            # Init periodic timer 
            rospy.Timer(rospy.Duration(self.dutsbin_timer), self.time_trigger)
            rospy.Timer(rospy.Duration(3), self.get_stimulus)
            self.get_information = False
    
    def kill_the_process(self,msg):
        # update area explored
        self.exploration_tasks_update[msg.explored_sub_area] = True
        print("******TASK UPDATE******")
        print(self.exploration_tasks_update)
        # check if the area is fully explored 
        if all(self.exploration_tasks_update):
            print("****************The exploration area is totally explored***************")
            list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
            list_output = list_cmd.stdout.read()
            retcode = list_cmd.wait()
            assert retcode == 0, "List command returned %d" % retcode
            for str in list_output.split("\n"):
                if (str.startswith('/record_')):
                    os.system("rosnode kill " + str)

    
    def remove_robot_from_dustbin_goals(self,msg):
        # remove the robot from the dustbin goals
        robot_id = msg.data
        self.robots_id = np.delete(self.robots_id, np.where(self.robots_id == robot_id))
        self.check_dustbin_robot()
    
    def check_dustbin_robot(self):
        if(np.size(self.robots_id)==0):
            self.enable_tracking = False
            self.goto_central_area()
                  
    def goto_central_area(self):
        self.central_point = self.area_handler.get_main_polygon_centroid()
        self.pose = [self.central_point.x, self.central_point.y]
        self.transit_to(self.pose)
        self.robot_at_center = True

    def initialization(self,robot_id):
        # check if all the n robots are publishing their information
        if(self.robots_information[robot_id][0] != 0):
            self.robot_initialization[robot_id] = True
        
        if((self.robot_initialization == True).all()):
            self.system_init = True
                 
    def tracking(self):
        self.auv_position_north = self.robots_information[self.robot_goal_id][0]
        self.asv_position_north = self.asv_north_position
        self.auv_position_east = self.robots_information[self.robot_goal_id][1]
        self.asv_position_east = self.asv_east_position

        self.adrift_marker()
        self.repulsion_marker()
        self.tracking_marker()

        self.x_distance = self.auv_position_north-self.asv_position_north
        self.y_distance = self.auv_position_east-self.asv_position_east
        self.radius = sqrt((self.x_distance)**2 + (self.y_distance)**2)
        self.initialized = True

        if( self.radius < (self.adrift_radius+2) and self.set_end_time == True):
            self.set_comm_end_time(rospy.Time.now())
            self.set_end_time = False

        if(self.radius > self.adrift_radius):
            self.tracking_strategy()

        # elif(self.repulsion_radius <= self.radius <= self.adrift_radius):
        #     # rospy.loginfo("----------------- ADRIFT STRATEGY -----------------")
        #     

        elif(self.radius <= self.repulsion_radius):
            self.extract_safety_position()
            self.repulsion_strategy(self.x, self.y)
    
    def extract_safety_position(self):
        self.m =-(1/ (self.auv_position_east-self.asv_position_east)/(self.auv_position_north-self.asv_position_north))
        self.pointx = self.asv_position_north
        self.pointy = self.asv_position_east
        if (self.auv_position_north > self.asv_position_north):
            self.x = self.asv_position_north-1
        else:
            self.x = self.asv_position_north+1
        self.y = self.m*(self.x - self.pointy) + self.pointx 

    def repulsion_strategy(self, position_x, position_y):
        constant_linear_velocity = 4
        constant_angular_velocity = 2 
        linear_velocity = constant_linear_velocity
        alpha_ref = atan2(position_y,position_x)
        #obtain the minimum agle between both robots
        angle_error = atan2(sin(alpha_ref-self.asv_yaw), cos(alpha_ref-self.asv_yaw))
        self.angular_velocity = constant_angular_velocity * angle_error
        self.xr = linear_velocity*cos(angle_error)
        self.yr = linear_velocity*sin(angle_error)
        self.corrected_bvr_pusblisher(self.xr, self.yr,self.angular_velocity)


    def tracking_strategy(self):
        constant_linear_velocity = 3
        constant_angular_velocity = 2 

        if (self.radius<self.tracking_radius and self.radius>self.adrift_radius):
            self.velocity_adjustment = (self.radius-(self.adrift_radius))/(self.tracking_radius-(self.adrift_radius))
        else:
            self.velocity_adjustment = 1  
        linear_velocity = self.velocity_adjustment * constant_linear_velocity
        alpha_ref = atan2(self.y_distance,self.x_distance)
        angle_error = atan2(sin(alpha_ref-self.asv_yaw), cos(alpha_ref-self.asv_yaw))
        self.angular_velocity = constant_angular_velocity * angle_error
        self.xr = linear_velocity*cos(angle_error)
        self.yr = linear_velocity*sin(angle_error)
        if(self.xr < 0 ):
            self.xr = 0
        self.corrected_bvr_pusblisher(self.xr, self.yr,self.angular_velocity)
    
    def corrected_bvr_pusblisher(self,corrected_velocity_x,corrected_velocity_y,corrected_angular_z):
        # NEW PRIORITY DEFINITIONS
        # PRIORITY_TELEOPERATION_LOW = 0
        # PRIORITY_SAFETY_LOW = 5
        # PRIORITY_NORMAL = 10
        # PRIORITY_NORMAL_HIGH = 20
        # PRIORITY_TELEOPERATION = 40
        # PRIORITY_SAFETY = 45
        # PRIORITY_SAFETY_HIGH  = 50
        # PRIORITY_TELEOPERATION_HIGH = 60 
        bvr = BodyVelocityReq()
        bvr.header.frame_id    = '/robot'+str(self.robot_ID)+'/base_link'
        bvr.header.stamp       = rospy.Time.now()
        bvr.goal.requester     =  self.name
        bvr.disable_axis.x     = False
        bvr.disable_axis.y     = False
        bvr.disable_axis.z     = True
        bvr.disable_axis.roll  = True
        bvr.disable_axis.pitch = True
        bvr.disable_axis.yaw   = False
        bvr.twist.linear.x     = corrected_velocity_x
        bvr.twist.linear.y     = corrected_velocity_y
        bvr.twist.linear.z     = 0.0
        bvr.twist.angular.x    = 0.0
        bvr.twist.angular.y    = 0.0
        bvr.twist.angular.z    = corrected_angular_z
        bvr.goal.priority      = 20
        self.corrected_bvr.publish(bvr)
    
    def transit_to(self,pose):    
        self.disable_all_and_set_idle_srv()
        """Goto to position x, y, z, at velocity vel."""
        # // Define waypoint attributes
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = 1
        goto_req.position.x = pose[0]
        goto_req.position.y = pose[1]
        goto_req.position.z = 0.0
        goto_req.position_tolerance.x = 5
        goto_req.position_tolerance.y = 5
        goto_req.position_tolerance.z = 5
        goto_req.blocking = True
        goto_req.keep_position = False
        goto_req.disable_axis.x = False
        goto_req.disable_axis.y = True
        goto_req.disable_axis.z = False
        goto_req.disable_axis.roll = True
        goto_req.disable_axis.yaw = False
        goto_req.disable_axis.pitch = True
        goto_req.priority = 20
        goto_req.reference = 0 #REFERENCE_NED=0  REFERENCE_GLOBAL=1 REFERENCE_VEHICLE=2
        self.goto_srv(goto_req)

    def adrift_marker(self):
        self.anchor = Marker()
        self.anchor.header.frame_id = "world_ned"
        self.anchor.header.stamp = rospy.Time.now()
        self.anchor.ns = "mrs"
        self.anchor.id = 0
        self.anchor.type = Marker.CYLINDER
        self.anchor.action = Marker.ADD
        self.anchor.pose.position.x = self.auv_position_north
        self.anchor.pose.position.y = self.auv_position_east
        self.anchor.pose.position.z = 0
        self.anchor.pose.orientation.x = 0
        self.anchor.pose.orientation.y = 0
        self.anchor.pose.orientation.z = 0
        self.anchor.pose.orientation.w = 1.0
        self.anchor.scale.x = self.adrift_radius*2
        self.anchor.scale.y = self.adrift_radius*2
        self.anchor.scale.z = 0.1
        self.anchor.color.r = 0.0
        self.anchor.color.g = 1.0
        self.anchor.color.b = 0.0
        self.anchor.color.a = 0.3
        self.markerPub_adrift.publish(self.anchor)

    def repulsion_marker(self):
        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = "world_ned"
        self.robotMarker.header.stamp = rospy.Time.now()
        self.robotMarker.ns = "mrs"
        self.robotMarker.id = 0
        self.robotMarker.type = Marker.CYLINDER
        self.robotMarker.action = Marker.ADD
        self.robotMarker.pose.position.x = self.auv_position_north
        self.robotMarker.pose.position.y = self.auv_position_east
        self.robotMarker.pose.position.z = 0
        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = self.repulsion_radius*2
        self.robotMarker.scale.y = self.repulsion_radius*2
        self.robotMarker.scale.z = 0.1
        self.robotMarker.color.r = 1.0
        self.robotMarker.color.g = 0.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 0.3
        self.markerPub_repulsion.publish(self.robotMarker)

    def tracking_marker(self):
        self.follow = Marker()
        self.follow.header.frame_id = "world_ned"
        self.follow.header.stamp = rospy.Time.now()
        self.follow.ns = "mrs"
        self.follow.id = 0
        self.follow.type = Marker.CYLINDER
        self.follow.action = Marker.ADD
        self.follow.pose.position.x = self.auv_position_north
        self.follow.pose.position.y = self.auv_position_east
        self.follow.pose.position.z = 0
        self.follow.pose.orientation.x = 0
        self.follow.pose.orientation.y = 0
        self.follow.pose.orientation.z = 0
        self.follow.pose.orientation.w = 1.0
        self.follow.scale.x = self.tracking_radius*2
        self.follow.scale.y = self.tracking_radius*2
        self.follow.scale.z = 0.1
        self.follow.color.r = 0.0
        self.follow.color.g = 0.0
        self.follow.color.b = 1.0
        self.follow.color.a = 0.3
        self.markerPub_tracking.publish(self.follow)

  
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

