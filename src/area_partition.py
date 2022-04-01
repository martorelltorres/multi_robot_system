#!/usr/bin/env python

import roslib
import rospy
from array import *
import numpy as np
import os
import re
import geopandas as gpd
from shapely.geometry import MultiPolygon
from shapely.geometry import Polygon
from shapely.ops import split, LineString
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt


class path_planner:

    def __init__(self, name):
        self.name = name
        self.ned_origin_lat = get_param(self,'/xiroi/navigator/ned_latitude')
        self.ned_origin_lon = get_param(self,'/xiroi/navigator/ned_longitude')
        self.read_file()

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
        
        self.extract_NED_positions()


    
    def extract_NED_positions(self):
        # self.ned = NED(39.52563336913869, 2.5492000075129138, 0.0)
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
        plt.show()
        self.define_polygon() 

    def define_polygon(self):
        self.polygon_area = Polygon(zip(self.east_position, self.north_position))
        plt.plot(self.north_position, self.east_position, c="red")
        crs = {'init': 'epsg:4326'}
        polygon = gpd.GeoDataFrame(index=[0], crs=crs, geometry=[self.polygon_area])  
        print(self.polygon_area.area)     
        print("defineee")

        points=[]
        coords=[]
        for i in range(len(self.latitude)):
            points.append([self.latitude[i],self.longitude[i]])
        coords.append(points[i])
        # region_polys, region_pts = voronoi_regions_from_coords(coords, self.polygon_area)
        # split_polygon(self.polygon_area)
            

def get_squares_from_rect(RectangularPolygon, side_length=0.0025):
    rect_coords = np.array(RectangularPolygon.boundary.coords.xy)
    y_list = rect_coords[1]
    x_list = rect_coords[0]
    y1 = min(y_list)
    y2 = max(y_list)
    x1 = min(x_list)
    x2 = max(x_list)
    width = x2 - x1
    height = y2 - y1

    xcells = int(np.round(width / side_length))
    ycells = int(np.round(height / side_length))

    yindices = np.linspace(y1, y2, ycells + 1)
    xindices = np.linspace(x1, x2, xcells + 1)
    horizontal_splitters = [
        LineString([(x, yindices[0]), (x, yindices[-1])]) for x in xindices
    ]
    vertical_splitters = [
        LineString([(xindices[0], y), (xindices[-1], y)]) for y in yindices
    ]
    result = RectangularPolygon
    for splitter in vertical_splitters:
        result = MultiPolygon(split(result, splitter))
    for splitter in horizontal_splitters:
        result = MultiPolygon(split(result, splitter))
    square_polygons = list(result)

    return square_polygons


def split_polygon(polygon, side_length=0.025, shape="square", thresh=0.9):
    assert side_length>0, "side_length must be a float>0"
    Rectangle    = polygon.envelope
    squares      = get_squares_from_rect(Rectangle, side_length=side_length)
    SquareGeoDF  = gpd.GeoDataFrame(squares).rename(columns={0: "geometry"})
    Geoms        = SquareGeoDF[SquareGeoDF.intersects(polygon)].geometry.values
    if shape == "square":
        self.geoms = [g for g in Geoms if ((g.intersection(polygon)).area / g.area) >= thresh]
    print("spliiit")
    plt.plot(self.geoms)
    plt.show()

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