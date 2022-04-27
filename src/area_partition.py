#!/usr/bin/env python

from operator import indexOf
import roslib
import rospy            
from array import *
import numpy as np
from numpy import asarray
import math
import os
import pprint
import re
import geopandas as gpd
from shapely.geometry import Polygon,LineString,Point
from std_srvs.srv import Empty, EmptyResponse
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.ops import split, LineString,triangulate
from cola2_lib.utils.ned import NED
from geometry_msgs.msg import PointStamped
import matplotlib.pyplot as plt
from std_srvs.srv import Empty, EmptyRequest
# from multi_robot_system.srv import GoalPoint
from cola2_msgs.srv import Goto, GotoRequest
from cola2_msgs.msg import  NavSts


class area_partition:

    def __init__(self, name):
        self.name = name
        self.ned_origin_lat = get_param(self,'/xiroi/navigator/ned_latitude')
        self.ned_origin_lon = get_param(self,'/xiroi/navigator/ned_longitude')
        self.offset_distance = 0
        self.initial_offset = 1
        self.intersection_points =[]
        self.distance = []

        # Sevice server
        self.send_position = rospy.Service('mrs_control/send_position',
                                       Empty,
                                       self.send_position)
        #Publishers
        self.goal_position = rospy.Publisher("/mrs/goal_position",
                                                PointStamped,
                                                queue_size=1)
        #Subscribers
        rospy.Subscriber("/xiroi/navigator/navigation",
                         NavSts,    
                         self.update_robot_position,
                         queue_size=1)
        

        self.read_file()
        self.extract_NED_positions()
        self.divide_polygon()
        # self.define_path_coverage()
        # plt.show()
    def update_robot_position(self,msg):
        self.position_north = msg.position.north
        self.position_east = msg.position.east
    
    def get_polygon_points(self,polygon):
        polygon_points = self.voronoi_polygons[polygon]
        polygon_coords_x,polygon_coords_y = polygon_points.exterior.coords.xy
        return(polygon_coords_x,polygon_coords_y)

    def get_polygon_number(self):
        polygon_number = len(self.voronoi_polygons)
        return(polygon_number)


    def define_path_coverage(self, polygon):
        #create the loop for the diferent voronoi polygons
        self.find_largest_side(self.voronoi_polygons[polygon])
        self.cover_lines(self.voronoi_polygons[polygon])
        return(self.goal_points)

    def determine_nearest_polygon(self,x_position, y_position):
        point = Point(x_position,y_position)
        #determine if the robots are in the predefined area
        is_in = self.main_polygon.contains(point)
        #determine in which subpolygon are 
        if (is_in == True):
            for voronoi_polygon in range(len(self.voronoi_polygons)):
                is_in = self.voronoi_polygons[voronoi_polygon].contains(point)
                if (is_in == True):
                    self.goal_polygon = voronoi_polygon
                    self.goal_polygon_defined = True
        else:            
        #in case that the robot was not in any polygon, determine the nearest polygon in order to cover it
            for voronoi_polygon in range(len(self.voronoi_polygons)):
                distance_point_to_polygons = point.distance(self.voronoi_polygons[voronoi_polygon])
                self.distance.append(distance_point_to_polygons)
            min_distance = min(self.distance)
            self.goal_polygon = self.distance.index(min_distance)
            self.goal_polygon_defined = True
        return(self.goal_polygon)

    def get_goal_points(self, goal_polygon):
        if (self.goal_polygon_defined == True):
            goal_points = self.goal_points([goal_polygon])

        self.goal_polygon_defined = False
        return(goal_points)

    def send_position(self, req):
        self.send_position = True
        return EmptyResponse()
            
    
    def cover_lines(self, polygon):
        x,y = polygon.exterior.coords.xy
        slope = (y[self.reference_points[1]]-y[self.reference_points[0]])/(x[self.reference_points[1]]-x[self.reference_points[0]])
        y_coordinate =[]
        x_threshold = 10

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

        while(self.offset_distance < self.max_distance):
            self.offset_distance = self.initial_offset + self.offset_distance
            offset = line.parallel_offset(self.offset_distance, 'left', join_style=1)
            self.find_intersection_points(polygon,offset)
            self.plot_line(offset)

        # the goal_points array stores the diferent intersection points of each voronoi polygon, then print(self.goal_points[0])store the intersection points of the polygon 0
        self.goal_points = []
        for voronoi_polygon in range(len(self.voronoi_polygons)):
            self.goal_points.append([[voronoi_polygon],self.intersection_points])

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
        # plt.show()

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
    
    def divide_polygon(self):
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


        # Define the main polygon object
        self.main_polygon = Polygon(self.local_points)
        polygon_centroid = self.main_polygon.centroid
        self.polygon_points = self.local_points
        # Triangulate the polygon
        poligonized_points =Polygon(self.polygon_points)
        triangles = triangulate(poligonized_points)
        centroid_points = []
        # extract centroid points of the triangles
        for triangle in triangles:
            triangle_centroid = triangle.centroid
            centroid_points.append([triangle_centroid.x,triangle_centroid.y])
        centroid_points.append([polygon_centroid.x,polygon_centroid.y])
   
        # compute Voronoi tesselation
        voronoi_regions = Voronoi(centroid_points)
        regions, vertices = self.voronoi_finite_polygons_2d(voronoi_regions)
        voronoi_plot_2d(voronoi_regions)
        min_x = voronoi_regions.min_bound[0] - 10
        max_x = voronoi_regions.max_bound[0] + 10
        min_y = voronoi_regions.min_bound[1] - 10
        max_y = voronoi_regions.max_bound[1] + 10

        mins = np.tile((min_x, min_y), (vertices.shape[0], 1))
        bounded_vertices = np.max((vertices, mins), axis=0)
        maxs = np.tile((max_x, max_y), (vertices.shape[0], 1))
        bounded_vertices = np.min((bounded_vertices, maxs), axis=0)

        # colorize
        self.voronoi_polygons = []
        self.voronoi_polygons_points = []
        for region in regions:
            polygon = vertices[region]
            # Clipping polygon
            sub_polygons = Polygon(polygon)
            sub_polygons = sub_polygons.intersection(self.main_polygon)
            polygon = [p for p in sub_polygons.exterior.coords]
            # save the differents points of the subpolygon in voronoi_polygons as a polygon object and in voronoi_polygons_points as a polygon points
            polygon_coords = polygon
            self.voronoi_polygons.append(sub_polygons)
            self.voronoi_polygons_points.append(polygon_coords)
            plt.fill(*zip(*polygon), alpha=0.4)
        
        plt.plot(*zip(*self.polygon_points))
        plt.axis('equal')
        plt.xlim(-100,-5)
        plt.ylim(-110,20)

    def voronoi_finite_polygons_2d(self,vor, radius=None):
        if vor.points.shape[1] != 2:
            raise ValueError("Requires 2D input")

        new_regions = []
        new_vertices = vor.vertices.tolist()

        center = vor.points.mean(axis=0)
        if radius is None:
            radius = vor.points.ptp().max()*2

        # Construct a map containing all ridges for a given point
        all_ridges = {}
        for (p1, p2), (v1, v2) in zip(vor.ridge_points, vor.ridge_vertices):
            all_ridges.setdefault(p1, []).append((p2, v1, v2))
            all_ridges.setdefault(p2, []).append((p1, v1, v2))

        # Reconstruct infinite regions
        for p1, region in enumerate(vor.point_region):
            vertices = vor.regions[region]

            if all(v >= 0 for v in vertices):
                # finite region
                new_regions.append(vertices)
                continue

            # reconstruct a non-finite region
            ridges = all_ridges[p1]
            new_region = [v for v in vertices if v >= 0]

            for p2, v1, v2 in ridges:
                if v2 < 0:
                    v1, v2 = v2, v1
                if v1 >= 0:
                    # finite ridge: already in the region
                    continue

                # Compute the missing endpoint of an infinite ridge

                t = vor.points[p2] - vor.points[p1] # tangent
                t /= np.linalg.norm(t)
                n = np.array([-t[1], t[0]])  # normal

                midpoint = vor.points[[p1, p2]].mean(axis=0)
                direction = np.sign(np.dot(midpoint - center, n)) * n
                far_point = vor.vertices[v2] + direction * radius

                new_region.append(len(new_vertices))
                new_vertices.append(far_point.tolist())

            # sort region counterclockwise
            vs = np.asarray([new_vertices[v] for v in new_region])
            c = vs.mean(axis=0)
            angles = np.arctan2(vs[:,1] - c[1], vs[:,0] - c[0])
            new_region = np.array(new_region)[np.argsort(angles)]

            # finish
            new_regions.append(new_region.tolist())

        return new_regions, np.asarray(new_vertices)
   
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