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
    voronoy_polygons_settled = False
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
        self.voronoy_polygons_settled = False
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
    
    def get_voronoi_polygons_status(self): 
        return(self.voronoy_polygons_settled)

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

    def define_path_coverage(self, voronoi_polygons):
        #create the loop for the diferent voronoi offset polygons
        # for self.polygon_id in range(len(self.voronoi_offset_polygons)):
        #     self.find_largest_side(self.voronoi_offset_polygons[self.polygon_id])
        #     goal_points = self.cover_lines(self.voronoi_offset_polygons[self.polygon_id])
        # return(goal_points)
        for self.polygon_id in range(self.number_of_robots):
            self.find_largest_side(voronoi_polygons[self.polygon_id])
            goal_points = self.cover_lines(voronoi_polygons[self.polygon_id])
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
        x_threshold = 200 #take care with this harcoded parameter
 
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

    def read_file(self):
        data = []
        self.latitude = []
        self.longitude = []

        file = open(str(self.exploration_area), "r")

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
      

    def clustering(self):
        # reduced_data = PCA(n_components=2).fit_transform(self.points)
        reduced_data = self.points
        kmeans = KMeans(init="k-means++", n_clusters=self.number_of_robots, n_init=4)
        kmeans.fit(reduced_data)

        # Step size of the mesh. Decrease to increase the quality of the VQ.
        h = 0.02  # point in the mesh [x_min, x_max]x[y_min, y_max].

        # Plot the decision boundary. For that, we will assign a color to each
        x_min, x_max = reduced_data[:, 0].min() - 1, reduced_data[:, 0].max() + 1
        y_min, y_max = reduced_data[:, 1].min() - 1, reduced_data[:, 1].max() + 1
        xx, yy = np.meshgrid(np.arange(x_min, x_max, h), np.arange(y_min, y_max, h))

        # Obtain labels for each point in mesh. Use last trained model.
        Z = kmeans.predict(np.c_[xx.ravel(), yy.ravel()])

        # Put the result into a color plot
        Z = Z.reshape(xx.shape)
        self.cluster_centroids = kmeans.cluster_centers_
        # plt.figure(1)
        # plt.clf()
        # plt.imshow(
        #     Z,
        #     interpolation="nearest",
        #     extent=(xx.min(), xx.max(), yy.min(), yy.max()),
        #     cmap=plt.cm.Paired,
        #     aspect="auto",
        #     origin="lower",
        # )

        # plt.plot(reduced_data[:, 0], reduced_data[:, 1], "k.", markersize=2)
        # # Plot the centroids as a white X
        
        # plt.scatter(
        #     self.cluster_centroids[:, 0],
        #     self.cluster_centroids[:, 1],
        #     marker="x",
        #     s=169,
        #     linewidths=3,
        #     color="w",
        #     zorder=10,
        # )

        # plt.xlim(x_min, x_max)
        # plt.ylim(y_min, y_max)
        # plt.xticks(())
        # plt.yticks(())
        # plt.show()


    
    def divide_polygon(self):
        #obtain the global_points (lat,long) of the polygon
        self.global_points=[]
        self.global_coords=[]
        for i in range(len(self.latitude)):
            self.global_points.append([self.latitude[i],self.longitude[i]])
        self.global_coords.append(self.global_points[i])

        #obtain the local_points (lat,long) of the polygon
        for i in range(len(self.north_position)):
            self.local_points.append([self.north_position[i],self.east_position[i]])

        # Define the main polygon object
        self.main_polygon = Polygon(self.local_points)
        self.polygon_points = self.local_points
        # Triangulate the polygon
        poligonized_points =Polygon(self.polygon_points)
        # .........................................................................
        # Generate random points within the polygon
        num_points = 500
    
        while len(self.centroid_points) < num_points:
            # Generate random coordinates within the polygon's bounds
            x = uniform(self.main_polygon.bounds[0], self.main_polygon.bounds[2])
            y = uniform(self.main_polygon.bounds[1], self.main_polygon.bounds[3])
            # Create a point object
            point = Point(x,y)
            # Check if the point is within the polygon
            if self.main_polygon.contains(point):
                self.centroid_points.append([x,y])
        self.points = np.array(self.centroid_points)
        self.clustering()
        self.conpute_voronoi_tesselation()
        
    def conpute_voronoi_tesselation(self):

        # compute Voronoi tesselation
        voronoi_regions = Voronoi(self.cluster_centroids)
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
        self.voronoi_offset_polygons = []
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
        
        self.voronoy_polygons_settled = True
        
        plt.plot(*zip(*self.polygon_points))
        plt.axis('equal')
        plt.xlim(-100,100)
        plt.ylim(-100,100)
        plt.show()
        

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