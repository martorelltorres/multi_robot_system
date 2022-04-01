#!/usr/bin/env python

from operator import indexOf
import roslib
import rospy
from array import *
import numpy as np
import math
import os
import re
import geopandas as gpd
from shapely.geometry import MultiPolygon
from shapely.geometry import Polygon
from shapely.ops import split, LineString
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point, PolygonStamped
from visualization_msgs.msg import Marker



class path_planner:

    def __init__(self, name):
        self.name = name
        self.ned_origin_lat = get_param(self,'/xiroi/navigator/ned_latitude')
        self.ned_origin_lon = get_param(self,'/xiroi/navigator/ned_longitude')
        self.read_file()
        self.extract_NED_positions()
        self.characterize_polygon()
        self.define_path_coverage()

    def read_file(self):
        data = []
        self.latitude = []
        self.longitude = []

        file = open("/home/uib/iquaview/missions/220328104705_plygon_mission.xml", "r")

        for line in file:
            data.append(line)

        for line in data:
            if "<latitude>" in line :
                start = line.find("<latitude>") + len("<latitude>")
                end = line.find("</latitude>")
                substring_lat = line[start:end]
                self.latitude.append(float(substring_lat))

            if "<longitude>" in line :
                start = line.find("<longitude>") + len("<longitude>")
                end = line.find("</longitude>")
                substring_long = line[start:end]
                self.longitude.append(float(substring_long))
        
    def extract_NED_positions(self):
        self.ned = NED(self.ned_origin_lat, self.ned_origin_lon, 0.0)  # NED frame
        self.north_position =[]
        self.east_position =[]

        for i in range(len(self.latitude)):
            north, east, depht = self.ned.geodetic2ned([self.latitude[i],
                                                self.longitude[i],
                                                0.0])
            self.north_position.append(north)
            self.east_position.append(east) 
            
        self.north_position.append(self.north_position[0])
        self.east_position.append(self.east_position[0])
        plt.plot(self.north_position, self.east_position, c="red")
        
    
    def characterize_polygon(self):
        #obtain the global_points (lat,long) of the polygon
        self.global_points=[]
        self.global_coords=[]
        for i in range(len(self.latitude)):
            self.global_points.append([self.latitude[i],self.longitude[i]])
        self.global_coords.append(self.global_points[i])

        #obtain the local_points (lat,long) of the polygon
        self.local_points=[]
        self.local_coords=[]
        for i in range(len(self.north_position)):
            self.local_points.append([self.north_position[i],self.east_position[i]])
        self.local_coords.append(self.local_points[i])

        # obtain the distance[m] between the different points of the polygon
        self.distance =[]
        for i in range(len(self.north_position)):

            if(i==(len(self.north_position)-1)):
                x_distance = self.north_position[i]-self.north_position[(len(self.north_position)-1)]
                y_distance = self.east_position[i]-self.east_position[(len(self.north_position)-1)]

            else:
                x_distance = self.north_position[i]-self.north_position[(i+1)]
                y_distance = self.east_position[i]-self.east_position[(i+1)]

            cartesian_distance = np.sqrt(x_distance**2 + y_distance**2)
            self.distance.append(cartesian_distance)
        
        # find the max distance reference_points
        max_distance = max(self.distance)
        index =  self.distance.index(max_distance)
        self.reference_points =[]
        self.reference_points.append (index)

        if (index+1 == len(self.north_position)-1):
            self.reference_points.append(0)

    def define_path_coverage(self):
        # define the initial line between the reference_points
        # point_A = self.local_points[self.reference_points[0]]
        # point_B = self.local_points[self.reference_points[0]]
        # slope = 

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
        rospy.init_node('path_planner')
        path_planner = path_planner(rospy.get_name())
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass