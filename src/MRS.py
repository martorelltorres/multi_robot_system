#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import Imu, MagneticField, Range
from std_srvs.srv import Empty, EmptyResponse
import tf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import datetime
from numpy import *
import message_filters
import PyKDL
import actionlib
import sys
from std_srvs.srv import Empty
from std_srvs.srv import Trigger, TriggerRequest
from cola2_msgs.msg import GoalDescriptor, NavSts
from cola2_msgs.srv import Goto, GotoRequest
from cola2_msgs.msg import WorldSectionAction
from cola2_xiroi.srv import SharePosition
from cola2_lib.utils.angles import wrap_angle
from cola2_lib.utils.ned import NED
from std_srvs.srv import Empty, EmptyRequest
from cola2_lib.rosutils import param_loader
from cola2_lib.rosutils.param_loader import get_ros_params
from operator import indexOf
import numpy as np
from numpy import asarray
import math
import os
from shapely.geometry import MultiPolygon, Point, MultiLineString,Polygon,asMultiPoint,asPolygon,asLineString
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.ops import split, LineString,triangulate
from geometry_msgs.msg import Point, PolygonStamped
from visualization_msgs.msg import Marker



class MultiRobotSystem:
    

    def __init__(self, name):
        """ Init the class """
        # Get config parameters
        self.name = name
        # self.get_config()

        # Services clients
        # try:
        #     rospy.wait_for_service('captain/enable_goto', 20)
        #     self.goto_srv = rospy.ServiceProxy(
        #                 'captain/enable_goto', Goto)
        # except rospy.exceptions.ROSException:
        #     rospy.logerr('%s: error creating client to goto service',
        #                  self.name)
        #     rospy.signal_shutdown('Error creating client to goto service')

        # try:
        #     rospy.wait_for_service('captain/disable_all_and_set_idle', 20)
        #     self.disable_all_and_set_idle_srv = rospy.ServiceProxy(
        #                 'captain/disable_all_and_set_idle', Trigger)
        # except rospy.exceptions.ROSException:
        #     rospy.logerr('%s: error creating client to disable_all service',
        #                  self.name)
        #     rospy.signal_shutdown('Error creating client to disable_all service')

        #Define subscribers
        # rospy.Subscriber("/xiroi/navigator/navigation",
        #                  NavSts,    
        #                  self.update_asv_nav_sts,
        #                  queue_size=1)

        # rospy.Subscriber("/turbot/navigator/navigation",
        #                  NavSts,
        #                  self.update_auv_nav_sts,
        #                  queue_size=1)

        # rospy.Subscriber("/mrs/grid_map_handler/grid_map_ASV_data",
        #                  Range,
        #                  self.update_ASV_data,
        #                  queue_size=1)

        # rospy.Subscriber("/mrs/grid_map_handler/grid_map_AUV_data",
        #                  Range,
        #                  self.update_AUV_data,
        #                  queue_size=1)

        # Service server
        self.goto = rospy.Service('mrs/goto',
                                   Empty,
                                   self.goto)
        #Actionlib section client
        section_action = actionlib.SimpleActionClient("/turbot/pilot/world_section_req", WorldSectionAction)
        section_action.wait_for_server()


        ASV_navigation_sub = message_filters.Subscriber("/xiroi/navigator/navigation",NavSts)
        AUV_navigation_sub = message_filters.Subscriber("/turbot/navigator/navigation",NavSts)
        ASV_data_sub = message_filters.Subscriber("/mrs/grid_map_handler/grid_map_ASV_data",Range)
        AUV_data_sub = message_filters.Subscriber("/mrs/grid_map_handler/grid_map_AUV_data",Range)

        # Topic synchronization
        ts_ASV =  message_filters.ApproximateTimeSynchronizer([ASV_navigation_sub, ASV_data_sub], 1, 1)
        ts_AUV =  message_filters.ApproximateTimeSynchronizer([ASV_navigation_sub, ASV_data_sub], 1, 1)
        # Internal callbacks
        ts_ASV.registerCallback(self.ASV_update)
        ts_AUV.registerCallback(self.AUV_update)

        #Define publishers

        #Define node parameters
        # self.robot_name
        self.min_ASV_depth = 1
        self.max_ASV_depth = 4
        self.min_AUV_depth = 4
        self.max_AUV_depth = 40

        
        # Show messa
        rospy.loginfo('[%s]: initialized', self.name)


    def section(self,initial_position_x,initial_position_y,initial_position_z, initial_yaw, final_position_x, final_position_y, final_position_z):

        section_req = WorldSectionAction()
        section_req.initial_position.x = initial_position_x
        section_req.initial_position.y = initial_position_y
        section_req.initial_position.z = initial_position_z
        section_req.initial.yaw = initial_yaw
        section_req.final_position.x = final_position_x
        section_req.final_position.y = final_position_y
        section_req.final_position.z = final_position_z
        section_req.altitude_mode = True
        section_req.tolerance.x = 2.0
        section_req.tolerance.y = 2.0
        section_req.tolerance.z = 2.0
        section_req.surge_velocity = 0.7
        section_req.timeout = 120
        print("--------------------section initialized-----------------------------")
        section_action(section_req)
        rospy.sleep(1.0)

    

    def goto(self, req):
        """ This method sets the captain back to idle """
        try:
            self.disable_all_and_set_idle_srv(TriggerRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr("Error setting captain to idle state")

        """Goto to NED position."""
        # // Define waypoint attributes
        goto_req = GotoRequest()
        goto_req.altitude = 0
        goto_req.altitude_mode = False
        goto_req.linear_velocity.x = 0.5
        goto_req.position.x = self.ned_position_x
        goto_req.position.y = self.ned_position_y
        goto_req.position.z = 0.0
        goto_req.position_tolerance.x = 2.0
        goto_req.position_tolerance.y = 2.0
        goto_req.position_tolerance.z = 2.0
        goto_req.blocking = True
        goto_req.keep_position = False
        goto_req.disable_axis.x = False
        goto_req.disable_axis.y = True
        goto_req.disable_axis.z = False
        goto_req.disable_axis.roll = True
        goto_req.disable_axis.yaw = False
        goto_req.disable_axis.pitch = True
        goto_req.priority = 60 #PRIORITY_TELEOPERATION_LOW = 0 PRIORITY_SAFETY_LOW = 5 PRIORITY_NORMAL = 10 PRIORITY_SAFETY = 30 PRIORITY_TELEOPERATION = 40 PRIORITY_SAFETY_HIGH  = 50 PRIORITY_TELEOPERATION_HIGH = 60
        goto_req.reference = 1 #REFERENCE_NED=0  REFERENCE_GLOBAL=1 REFERENCE_VEHICLE=2
        self.goto_srv(goto_req)
        rospy.sleep(1.0)
    
    def ASV_update(self, ASV_nav_sts_msg, ASV_data_msg):
        self.asv_data = ASV_data_msg.range
        self.asv_position_north = ASV_nav_sts_msg.position.north
        self.asv_position_east = ASV_nav_sts_msg.position.east
        self.asv_yaw = ASV_nav_sts_msg.orientation.yaw
        self.robot_name = "ASV"
        self.data = self.asv_data
        self.explore()

    def AUV_update(self, AUV_nav_sts_msg, AUV_data_msg):
        self.auv_data = AUV_data_msg.range
        self.auv_position_north = AUV_nav_sts_msg.position.north
        self.auv_position_east = AUV_nav_sts_msg.position.east
        self.auv_position_depth = AUV_nav_sts_msg.position.depth
        self.auv_yaw = AUV_nav_sts_msg.orientation.yaw
        self.robot_name = "AUV"
        self.data = self.auv_data
        self.explore()

    
    def explore(self):
        if(self.robot_name == "ASV"):
            if(self.data < self.min_ASV_depth):
                print ("MOVE ASV TO A DEEP WATER ZONE")
                self.get_deeper_location(self.asv_yaw, self.asv_position_north, self.asv_position_east)
                 # section(self,initial_position_x,initial_position_y,initial_position_z, initial_yaw, final_position_x, final_position_y, final_position_z, req):
                self.section(self,self.asv_position_north,self.asv_position_east,0.0,self.asv_yaw,self.asv_position_north+10, self.asv_position_east+10,0.0)
            if(self.data > self.max_ASV_depth):
                print ("MOVE ASV TO A SHALLOW WATER ZONE")
                self.move_to_shallow_zone()
                self.section(self.asv_position_north,self.asv_position_east,0.0,self.asv_yaw,self.asv_position_north+10, self.asv_position_east+10,0.0)
            if(self.min_ASV_depth < self.data < self.max_ASV_depth):
                print ("ASV APPLYING SENSOR MAPPING STRATEGY")
                self.sensor_mapping_navigation()
        else:
            if(self.data < self.min_AUV_depth):
                print ("MOVE AUV TO A DEEP WATER ZONE")
            if(self.data > self.max_AUV_depth):
                print ("MOVE AUV TO A SHALLOW WATER ZONE")
            if(self.min_AUV_depth < self.data < self.max_AUV_depth):
                print ("AUV APPLYING SENSOR MAPPING STRATEGY")

    def get_deeper_location(yaw, x_position, y_position):
        self.turning_angle = yaw + 135
       
    def move_to_shallow_zone(self):
        print ("2222222")
        
    def sensor_mapping_navigation(self):
        print("INNN")


  

if __name__ == '__main__':
    try:
        rospy.init_node('multi_robot_system')
        return_to_NED = MultiRobotSystem(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass