#!/usr/bin/env python

from operator import indexOf
from pickle import NONE
import roslib
import rospy            
from array import *
import numpy as np
from numpy import asarray
import math
import os
import pprint
import re
from shapely.geometry import Polygon,LineString,Point
from shapely import affinity
from random import uniform
from std_srvs.srv import Empty, EmptyResponse
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.ops import split, LineString,triangulate
from cola2_lib.utils.ned import NED
from geometry_msgs.msg import PointStamped
import matplotlib.pyplot as plt
from std_srvs.srv import Empty, EmptyRequest
from cola2_msgs.srv import Goto, GotoRequest
from cola2_msgs.msg import  NavSts
from shapely.prepared import prep
import geopandas as gpd
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
import pickle


class area_partition:
    local_points=[]
    local_coords=[]
    polygon_coords_x =0
    polygon_coords_y= 0
    voronoi_polygons= []
    first_time = False
    centroid_points = []
    cluster_centroids = []
    voronoi_polygons = []

    def __init__(self, name):
        self.name = name
        # Get config parameters from the parameter server
        self.ned_origin_lat = get_param(self,'ned_origin_lat',39.14803625)
        self.ned_origin_lon = get_param(self,'ned_origin_lon',2.93195323)
        self.offset_polygon_distance = get_param(self,'offset_polygon_distance',5)
        self.offset_coverage_distance = get_param(self,'offset_coverage_distance',10)
        self.surge_velocity = get_param(self,'surge_velocity',0.8)
        self.exploration_area = get_param(self,'exploration_area',"/home/uib/MRS_ws/src/MRS_stack/multi_robot_system/missions/230210085906_cabrera_small.xml")
        self.number_of_robots = get_param(self,'number_of_robots',6)
        self.robot_ID = get_param(self,'~robot_ID',0) 
        self.offset_distance = 0
        self.goal_polygon_defined = False
        self.equal_regions = []
        self.points = np.array([])
        self.coverage_distance = 0
        self.fixed_offset = 1
        self.distance = []
        self.goal_points = []
        self.first_time = True
        self.local_points=[]
        self.local_coords=[]
        self.centroid_points = []
        self.cluster_centroids = []
        self.voronoi_polygons = []

        self.read_area_info()
   
    def read_area_info(self):
        # Open the pickle file in binary mode
        with open('/home/uib/area_partition_data.pickle', 'rb') as file:
            # Load the data from the file
            data = pickle.load(file)

        # Access different data from the loaded data
        self.cluster_centroids = data['array1']
        self.voronoi_polygons = data['array2']
        self.main_polygon = data['array3']
        self.main_polygon_centroid = data['array4']
        self.voronoi_offset_polygons = data['array5']
    

    def get_polygon_points(self,polygon):
        polygon_points = self.voronoi_polygons[polygon]
        polygon_coords_x,polygon_coords_y = polygon_points.exterior.coords.xy
        return(polygon_coords_x,polygon_coords_y)

    def get_offset_polygon_points(self,polygon):
        polygon_points = self.voronoi_offset_polygons[polygon]
        polygon_coords_x,polygon_coords_y = polygon_points.exterior.coords.xy
        return(polygon_coords_x,polygon_coords_y)

    def create_voronoi_offset_polygon(self,polygon,offset):
        goal_polygon = self.voronoi_polygons[polygon]
        offset_polygon = goal_polygon.buffer(offset,cap_style=3, join_style=2)
        return(offset_polygon)

    def get_polygon_area(self,polygon):
        polygon_area = self.voronoi_polygons[polygon].area
        return(polygon_area)

    def get_polygon_number(self):
        polygon_number = len(self.voronoi_polygons)
        return(polygon_number)

    def get_voronoi_polygons(self):
        return(self.voronoi_polygons)

   
    def get_main_polygon_centroid(self):
        self.main_polygon = Polygon(self.local_points)
        polygon_centroid = self.main_polygon.centroid
        return(polygon_centroid)

    def get_polygon_centroids(self):
        return(self.centroid_points)
    
    def get_voronoi_offset_polygons(self):
        return(self.voronoi_offset_polygons)
    
    def get_sections_number(self):
        goal_points = self.define_path_coverage()
        number_of_sections = []
        for polygon in range(len(self.voronoi_offset_polygons)):
            # take into account that for example in case of 4 sections there will be only 3 turns
            number_of_sections.append(len(goal_points[polygon])-1)
        return(number_of_sections)
    
    def get_sections_time(self):
        goal_points = self.define_path_coverage()
        polygon_distances =[]
        distances = []
        for polygon in range(len(self.voronoi_offset_polygons)):
            section_points = goal_points[polygon]

            for points_pair in range(len(section_points)):
                point = section_points[points_pair]
                first_point = Point(point[0])
                second_point = Point(point[1])
                distance_btw_points = self.distance_between_points(first_point,second_point)
                self.coverage_distance = self.coverage_distance + distance_btw_points
            polygon_distances.append(self.coverage_distance)
            self.coverage_distance = 0
        distances.append(polygon_distances)
        return (distances)

    def get_estimated_polygons_coverage_time(self):
        estimated_coverage_time=[]
        sections_distance = self.get_sections_time()
        sections_distance = sections_distance[0]
        sections_number = self.get_sections_number()

        for polygon_id in range(len(self.voronoi_offset_polygons)):
            distance = sections_distance[polygon_id]
            turns = sections_number[polygon_id]
            sections_time = distance/self.surge_velocity
            # set an experimental predefined time to perform the turns of the coverage
            turning_time = turns*35
            total_time = turning_time + sections_time
            estimated_coverage_time.append(total_time)
        return(estimated_coverage_time)

    def define_path_coverage(self):
        #create the loop for the diferent voronoi offset polygons
        # for self.polygon_id in range(len(self.voronoi_offset_polygons)):
        #     self.find_largest_side(self.voronoi_offset_polygons[self.polygon_id])
        #     goal_points = self.cover_lines(self.voronoi_offset_polygons[self.polygon_id])
        # return(goal_points)
        for self.polygon_id in range(self.number_of_robots):
            self.find_largest_side(self.voronoi_polygons[self.polygon_id])
            goal_points = self.cover_lines(self.voronoi_polygons[self.polygon_id])
        return(goal_points)

    def distance_between_points(self,point_a, point_b):
        distance = point_a.distance(point_b)
        return(distance)

    def get_central_polygon(self,voronoi_polygons,main_centroid):
        for voronoi_polygon in range(len(voronoi_polygons)):
            is_in = self.voronoi_polygons[voronoi_polygon].contains(main_centroid)
            if (is_in == True):
                central_polygon = voronoi_polygon
                return(central_polygon)


    def determine_nearest_polygon(self,x_position, y_position,polygons):
        point = Point(x_position,y_position)
        #determine if the robots are in the predefined area
        is_in = self.main_polygon.contains(point)
        #determine in which subpolygon the robot is
        if (is_in == True):
            for voronoi_polygon in range(len(polygons)):
                is_in = self.voronoi_polygons[voronoi_polygon].contains(point)
                if (is_in == True):
                    self.goal_polygon = voronoi_polygon
                    self.goal_polygon_defined = True
        else:            
        #in case that the robot was not in any polygon, determine the nearest polygon in order to cover it
            for voronoi_polygon in range(len(polygons)):
                distance_point_to_polygons = point.distance(polygons[voronoi_polygon])
                self.distance.append(distance_point_to_polygons)
            min_distance = min(self.distance)
            self.goal_polygon = self.distance.index(min_distance)
            self.goal_polygon_defined = True

        return(self.goal_polygon)

    def get_goal_points(self, goal_polygon):
        if (self.goal_polygon_defined == True):
            goal_points = self.goal_points([goal_polygon])
        else:
            polygon_number = self.get_polygon_number()
            goal_points = []
            for polygon in range(polygon_number):
                goal_points.append(0)

        return(goal_points)

    def send_position(self, req):
        self.send_position = True
        return EmptyResponse()
    
    def define_voronoi_offset_polygons(self,offset):
        # create voronoi_offset_polygons in order to ensure the complete coverage of the areas
        for voronoi_polygon in range(len(self.voronoi_polygons)):
            new_polygon = self.create_voronoi_offset_polygon(voronoi_polygon,offset)
            self.voronoi_offset_polygons.append(new_polygon)
    
    def cover_lines(self, polygon):
        x,y = polygon.exterior.coords.xy
        slope = (y[self.reference_points[1]]-y[self.reference_points[0]])/(x[self.reference_points[1]]-x[self.reference_points[0]])
        y_coordinate =[]
        x_threshold = 100 #take care with this harcoded parameter
 
        if(x[self.reference_points[0]] > x[self.reference_points[1]]):
            x_0 = x[self.reference_points[0]] + x_threshold
            x_1 = x[self.reference_points[1]] - x_threshold
        else: 
            x_0 = x[self.reference_points[0]] - x_threshold
            x_1 = x[self.reference_points[1]] + x_threshold
        
        y_0 = slope*(x_0-x[self.reference_points[0]]) + y[self.reference_points[0]]
        y_coordinate.append(y_0)

        y_1 = slope*(x_1-x[self.reference_points[0]]) + y[self.reference_points[0]]
        y_coordinate.append(y_1)

        line = LineString([(x_0, y_coordinate[0]), (x_1, y_coordinate[1])])
        self.distance_point_to_line(line,polygon)
        self.offset_distance = 0
        self.intersection_points = []     

        while(self.offset_distance < self.max_distance):
            offset = line.parallel_offset(self.offset_distance, 'left', join_style=1)
            self.offset_distance = self.offset_distance + self.offset_coverage_distance
            self.find_intersection_points(polygon,offset)
            self.plot_line(offset)
        # the goal_points array stores the diferent intersection points of each voronoi polygon, then print(self.goal_points[0])store the intersection points of the polygon 0
        self.goal_points.append(self.intersection_points)

        # insert the first section obtained using the reference_points in the goal_points array
        initial_section = self.get_initial_section(self.polygon_id)
        initial_section = [initial_section[0][0],initial_section[1][0]]
        
        # remove empty elements and add first_section 
        if(self.goal_points[self.polygon_id][0] == []):
            self.goal_points[self.polygon_id][0] = initial_section

        return(self.goal_points)


    def find_intersection_points(self, polygon,line):
        points = line.intersection(polygon)
        # the intersection_points array stores the diferent goal points
        self.intersection_points.append(list(points.coords))
   
    def distance_point_to_line(self,line,polygon):
        x,y = polygon.exterior.coords.xy
        distance_point_to_line = []

        for i in range(len(x)):
            distance = line.distance(Point(x[i],y[i]))
            distance_point_to_line.append(distance)

        self.max_distance = max(distance_point_to_line)
        max_distance_point_x = distance_point_to_line.index(self.max_distance)

    def plot_line(ax, ob):
        parts = hasattr(ob, 'geoms') and ob or [ob]
        for part in parts:
            x, y = part.xy
            plt.plot(x, y, linewidth=3, solid_capstyle='round', zorder=1)
        plt.show()
    
    def get_initial_section(self, polygon):
        reference_points = self.find_largest_side(self.voronoi_polygons[polygon])
        polygon_coords_x,polygon_coords_y = self.get_polygon_points(polygon)
        initial_point = Point(polygon_coords_x[reference_points[0]],polygon_coords_y[reference_points[0]])
        final_point = Point(polygon_coords_x[reference_points[1]],polygon_coords_y[reference_points[1]])
        initial_section = list(initial_point.coords),list(final_point.coords)
        return(initial_section)


    def find_largest_side(self, polygon):
        x,y = polygon.exterior.coords.xy
        self.distance =[]
        for i in range(len(x)-1):
            if(i==(len(x)-1)):
                x_distance = x[i]-x[0]
                y_distance = y[i]-y[0]
            else:
                x_distance = x[i]-x[(i+1)]
                y_distance = y[i]-y[(i+1)]
            
            cartesian_distance = np.sqrt(x_distance**2 + y_distance**2)
            self.distance.append(cartesian_distance)

        max_distance = max(self.distance)
        index =  self.distance.index(max_distance)+1
        # the reference_points contains the index of the two points of the major side
        self.reference_points =[]
        self.reference_points.append(index)
        self.reference_points.append (index-1)
        return(self.reference_points)
    
    def find_largest_side_distance(self, polygon):
        x,y = polygon.exterior.coords.xy
        self.distance =[]
        for i in range(len(x)-1):
            if(i==(len(x)-1)):
                x_distance = x[i]-x[0]
                y_distance = y[i]-y[0]
            else:
                x_distance = x[i]-x[(i+1)]
                y_distance = y[i]-y[(i+1)]
            
            cartesian_distance = np.sqrt(x_distance**2 + y_distance**2)
            self.distance.append(cartesian_distance)

        max_distance = max(self.distance)
        index =  self.distance.index(max_distance)+1
        # the reference_points contains the index of the two points of the major side
        self.reference_points =[]
        self.reference_points.append(index)
        self.reference_points.append (index-1)

        point1 = polygon.exterior.coords[self.reference_points[0]]
        point2 = polygon.exterior.coords[self.reference_points[1]]
        return(point1, point2)

   
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
        rospy.init_node('area_partition')
        area_partition = area_partition(rospy.get_name())
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass