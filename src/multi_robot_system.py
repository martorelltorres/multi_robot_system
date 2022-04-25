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
from cola2_msgs.msg import WorldSectionAction
from std_srvs.srv import Empty
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
from cola2_msgs.msg import  NavSts
#import classes
from area_partition import area_partition

class MultiRobotSystem:
    
    def __init__(self, name):
        """ Init the class """
        # Get config parameters
        self.name = name
        # self.get_config()
        self.ned_origin_lat = self.get_param(self,'/xiroi/navigator/ned_latitude')
        self.ned_origin_lon = self.get_param(self,'/xiroi/navigator/ned_longitude')    
        self.area_handler =  area_partition("area_partition")
        self.goal_points = []
        self.check_current_position = True
        
        
        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        # mrs_control/send_position service
        rospy.wait_for_service('mrs_control/send_position', 10)
        try:
            self.send_position_srv = rospy.ServiceProxy(
                        'mrs_control/send_position', Empty)
        except rospy.ServiceException, e:
            rospy.logwarn("%s: Service call failed: %s", self.name, e)

        #Subscribers
        rospy.Subscriber("/xiroi/navigator/navigation",
                         NavSts,    
                         self.update_robot_position,
                         queue_size=1)

        #Actionlib section client
        self.section_action = actionlib.SimpleActionClient("/turbot/pilot/world_section_req", WorldSectionAction)
        self.section_action.wait_for_server()

    def update_robot_position(self,msg):
        self.position_north = msg.position.north
        self.position_east = msg.position.east
        if(self.check_current_position == True):
            self.goal_polygon = self.area_handler.determine_nearest_polygon(self.position_north,self.position_east)
            self.mrs_coverage()
            self.check_current_position = False
    
    def mrs_coverage(self):
        # obtain all the polygons goal_points
        self.goal_points = self.area_handler.define_path_coverage(self.goal_polygon)
        self.polygon_goal_points = self.goal_points[self.goal_polygon]
        all_sections = self.polygon_goal_points[1]
        print("77777777777777777777777777777777777777777")
        print(all_sections)


        for section in range((len(all_sections)-1)):
            # current_section = all_sections[section]
            # initial_point = current_section[0]
            # final_point = current_section[1]
            # self.send_section_strategy(initial_point,final_point)
          
            # First section
            if (section==0):
                current_section = all_sections[section]
                initial_point = current_section[0]
                final_point = current_section[1]
                self.send_section_strategy(initial_point,final_point)

            #if is an odd number
            elif(section%2==0):
                current_section = all_sections[section]
                initial_point = current_section[0]
                final_point = current_section[1]
                self.send_section_strategy(initial_point,final_point)
            #if is an even number
            else:
                current_section = all_sections[section]
                initial_point = current_section[0]
                final_point = current_section[1]
                self.send_section_strategy(initial_point,final_point)


    def send_section_strategy(self,initial_point,final_point):
        initial_position_x = initial_point[0]
        final_position_x = final_point[0]
        initial_position_y = initial_point[1]
        final_position_y = final_point[1]
        # print("333333333333333333333333333333")
        # print(initial_point)
        # print(final_point)
        # print(initial_position_x)
        # print(initial_position_y)

        section_req = WorldSectionAction()
        section_req.initial_position.x = initial_position_x
        section_req.initial_position.y = initial_position_y
        section_req.initial_position.z = 0.0
        section_req.initial.yaw = initial_yaw
        section_req.final_position.x = final_position_x
        section_req.final_position.y = final_position_y
        section_req.final_position.z = 0.0
        section_req.altitude_mode = True
        section_req.tolerance.x = 2.0
        section_req.tolerance.y = 2.0
        section_req.tolerance.z = 2.0
        section_req.surge_velocity = 0.7
        section_req.timeout = 120
        print("--------------------section initialized-----------------------------")
        self.section_action(section_req)
        rospy.sleep(1.0)


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
        rospy.init_node('multi_robot_system')
        return_to_NED = MultiRobotSystem(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass