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
from shapely.geometry import MultiPolygon, Point, MultiLineString, box, MultiPoint
from shapely.geometry import Polygon,asMultiPoint,asPolygon,asLineString
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.ops import split, LineString,triangulate
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point, PolygonStamped
from visualization_msgs.msg import Marker



class area_partition:

    def __init__(self, name):
        self.name = name
        self.ned_origin_lat = get_param(self,'/xiroi/navigator/ned_latitude')
        self.ned_origin_lon = get_param(self,'/xiroi/navigator/ned_longitude')
        self.point_distance = 2
        self.line_ecuations=[]
        self.read_file()
        self.extract_NED_positions()
        self.divide_polygon()
        self.define_path_coverage()
        plt.show()

    def define_path_coverage(self):
        
        polygon_bounds = self.voronoi_polygons[0].bounds
        max_x = polygon_bounds[2]
        max_y = polygon_bounds[3]
        min_x = polygon_bounds[0]
        min_y = polygon_bounds[1]
        # p = polygon_bounds.minimum_rotated_rectangle
        b = box(polygon_bounds[0],polygon_bounds[1],polygon_bounds[2],polygon_bounds[3])
        p = MultiPoint([(min_x,min_y),(max_x,min_y),(max_x,max_y),(min_x,max_y)]).minimum_rotated_rectangle
        # print(b)
        # plt.plot(*b.exterior.xy)
        plt.plot(*p.exterior.xy)
        # poly = self.voronoi_polygons[0]
        # plt.plot(*self.voronoi_polygons[0].exterior.xy)
        # self.find_largest_side()


    def find_largest_side(self):
        polygon = self.voronoi_polygons[0]
        coords = polygon.exterior.coords
        print(coords)
        print(polygon)
        a = coords[1]
        print(a)
        print(a[1])

        self.distance =[]
        for i in len(coords):
        
            # x_distance = self.coords[i][-self.north_position[(len(self.north_position)-1)]
            # x_distance = coords[i]-self.north_position[(len(self.north_position)-1)]
            y_distance = self.east_position[i]-self.east_position[(len(self.north_position)-1)]


            cartesian_distance = np.sqrt(x_distance**2 + y_distance**2)
            self.distance.append(cartesian_distance)

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