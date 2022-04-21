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
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.ops import split, LineString,triangulate
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt


class area_partition:

    def __init__(self, name):
        self.name = name
        self.ned_origin_lat = get_param(self,'/xiroi/navigator/ned_latitude')
        self.ned_origin_lon = get_param(self,'/xiroi/navigator/ned_longitude')
        self.offset_distance = 1
        self.initial_offset = 1
        self.line_ecuations=[]
        self.intersection_points =[]
        self.read_file()
        self.extract_NED_positions()
        self.divide_polygon()
        self.define_path_coverage()
        plt.show()

    def define_path_coverage(self):
        #create the loop for the diferent voronoi polygons
        # for voronoi_polygon in range(len(self.voronoi_polygons)):
        self.find_largest_side(self.voronoi_polygons[2])
        self.cover_lines(self.voronoi_polygons[2])
            
    
    def cover_lines(self, polygon):
        x,y = polygon.exterior.coords.xy
        slope = (y[self.reference_points[1]]-y[self.reference_points[0]])/(x[self.reference_points[1]]-x[self.reference_points[0]])
        # # extract line ecuation 
        y_coordinate =[]
        x_threshold = 10
        y_threshold = 10


        # if(slope<0):
        print("11111111111111111111111111")
        print(slope)
        print(self.reference_points)
        print(x[self.reference_points[0]])
        print(y[self.reference_points[0]])
        print(x[self.reference_points[1]])
        print(y[self.reference_points[1]])
    
        x1 = x[self.reference_points[0]]-x_threshold
        x0 = x[self.reference_points[1]]+x_threshold

        p = (slope*((x[self.reference_points[1]]+x_threshold )-x[self.reference_points[1]])) + y[self.reference_points[1]]
        y_coordinate.append(p)

        l = (slope*((x[self.reference_points[0]]-x_threshold )-x[self.reference_points[0]])) + y[self.reference_points[0]]
        y_coordinate.append(l)
        print(x0)
        print(y_coordinate[0])
        print(x1)
        print(y_coordinate[1])

        # else:
        #     print("2222222222222222")
        #     print(slope)
        #     print(self.reference_points)
        #     print(x[self.reference_points[0]])
        #     print(y[self.reference_points[0]])
        #     print(x[self.reference_points[1]])
        #     print(y[self.reference_points[1]])


        #     x1 = x[self.reference_points[0]]+x_threshold
        #     x0 = x[self.reference_points[1]]-x_threshold
        #     p = (slope*((x[self.reference_points[1]]-x_threshold )-x[self.reference_points[1]])) + y[self.reference_points[1]]
        #     y_coordinate.append(p)

        #     l = (slope*((x[self.reference_points[0]]+x_threshold )-x[self.reference_points[0]])) + y[self.reference_points[0]]
        #     y_coordinate.append(l)
        #     print(x0)
        #     print(y_coordinate[0])
        #     print(x1)
        #     print(y_coordinate[1])
           

        # y1 = y[self.reference_points[0]]+y_threshold
        # y0 = y[self.reference_points[1]]-y_threshold
        
        line = LineString([(x0, y_coordinate[0]), (x1, y_coordinate[1])])
        # line = LineString([(x1, y_coordinate[1]), (x0, y_coordinate[0])])
        self.distance_point_to_line(line,polygon)
        # print(line.distance.)
        
        while(self.offset_distance < self.max_distance):
            self.offset_distance = self.initial_offset + self.offset_distance
            offset = line.parallel_offset(self.offset_distance, 'right', join_style=1)
            self.find_intersection_points(polygon,offset)
            self.plot_line(offset)
        
        
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
        # print("13333333333333333333333333333333333")
        # print(max_distance_point_x)
        # print(distance_point_to_line)

    def plot_line(ax, ob):
        parts = hasattr(ob, 'geoms') and ob or [ob]
        for part in parts:
            x, y = part.xy
            plt.plot(x, y, linewidth=3, solid_capstyle='round', zorder=1)

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
        # print("111111111111111")
        # print(max_distance)
        # print(self.reference_points)
        # print(x)
        # print(y)
        

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
        main_polygon = Polygon(self.local_points)
        polygon_centroid = main_polygon.centroid
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
            sub_polygons = sub_polygons.intersection(main_polygon)
            polygon = [p for p in sub_polygons.exterior.coords]
            # save the differents points of the subpolygon in voronoi_polygons as a polygon object and in voronoi_polygons_points as a polygon points
            polygon_coords = polygon
            self.voronoi_polygons.append(sub_polygons)
            self.voronoi_polygons_points.append(polygon_coords)
            plt.fill(*zip(*polygon), alpha=0.4)
        
        # print(self.voronoi_polygons)
        # print(self.voronoi_polygons_points)
        plt.plot(*zip(*self.polygon_points))
        plt.axis('equal')
        plt.xlim(-100,-5)
        plt.ylim(-110,20)
        # plt.show()

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