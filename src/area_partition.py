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
        self.point_distance = 2
        self.line_ecuations=[]
        self.MAIN()


    def MAIN(self):
        self.read_file()
        self.extract_NED_positions()
        self.characterize_polygon()
        self.define_path_coverage()


    def read_file(self):
        data = []
        self.latitude = []
        self.longitude = []

        file = open("/home/uib/iquaview/missions/220405123332_polygon_mission.xml", "r")

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
        # plt.plot(self.north_position, self.east_position, c="red")
        # plt.show()
        
    
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
        # self.local_coords.append(self.local_points[i])
        # print("11111111111111111111111111111111")
        # print(self.local_points)
        # plt.plot(self.local_points)
        # plt.plot(self.local_coords)
        # # plt.show()
        # plt.show()

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
        self.reference_points.append (index-1)

        if (index+1 == len(self.north_position)-1):
            self.reference_points.append(0)

    def define_path_coverage(self):
        # define the ecuation of the line between the reference_points
        # The reference_point are the points that defines the major edge of the polygon, for example the 0 and 5 vertex
        point_A = self.local_points[self.reference_points[0]]
        point_B = self.local_points[self.reference_points[1]]
        # Extract reference line ecuation parameters
        self.slope_reference = (point_B[1]-point_B[0])/(point_A[1]-point_A[0])
        self.n_reference = point_B[1] - (self.slope_reference*point_B[0]) 
        # self.line_ecuations.append([self.reference_points[0],self.reference_points[1],self.slope_reference,self.n_reference])
        self.find_neighbour_points(self.reference_points[0],self.reference_points[1]) 
        if(self.neig_A > self.neig_B):
            self.neig_B = self.neig_A + 1
        else:
            self.neig_B = self.neig_A - 1

        point_A = self.local_points[self.neig_A]
        point_B = self.local_points[self.neig_B]
    
        self.extract_line_ecuation(point_A,point_B)
        self.find_angle_btw_edges()
        # for i in range(len(self.reference_points)):
        self.extract_desired_point(self.reference_points[1])

    def extract_desired_point(self,reference_point):
        if(self.angle>90):
            self.angle = self.angle-90
        y = self.point_distance
        x = np.tan(self.angle)*y
        self.desired_point = []
        reference_local_point = self.local_points[reference_point]
        desired_point_north = reference_local_point[0] + x
        desired_point_east = reference_local_point[1] + y
        desired_point = [desired_point_north,desired_point_east]
        self.desired_point.append(desired_point)
      

    def find_angle_btw_edges(self):
        for i in range(len(self.line_ecuations)):
            m1=self.slope_reference
            m2=self.line_ecuations[i][2]
            print(m1)
            print(m2)
            pi= 22/7
            self.angle = abs((m1-m2)/(1+(m2*m1)))
            self.angle= np.arctan(self.angle)
            self.angle = self.angle*(180/pi)
        
        return(self.angle)


    def extract_line_ecuation(self, point_1, point_2):
        print("**************extract_line_ecuation******************")
        self.slope = (point_2[1]-point_2[0])/(point_1[1]-point_1[0])
        # ecuacion recta punto, pendiente: y=m(x-x1)+y1=mx-mx1+y1 => y=mx+n donde n=y1-mx1
        self.n = point_2[1] - (self.slope*point_2[0]) 
        self.line_ecuations.append([self.neig_A,self.neig_B,self.slope,self.n])
        # print("11111111111111111111111111111111111")
        # print(self.line_ecuations)
        return self.line_ecuations

    # The find_neighbour_points method finds the nearest edge from a point, for example given the 5 and 0 points from a 5 edges polygon, 
    # the neares edge of 5 is 4 and the neares edge of 0 is 1
    def find_neighbour_points(self,point_A,point_B):
        if(point_A > point_B):          
            self.neig_A = point_A - 1
            self.neig_B = point_B + 1
        else:
            self.neig_A = point_A + 1
            self.neig_B = point_B - 1
        return(self.neig_A,self.neig_B)
   
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