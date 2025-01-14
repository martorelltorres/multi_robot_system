#!/usr/bin/env python3
import numpy as np
from sklearn.datasets import load_digits
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
from cola2.utils.ned import NED
from shapely.geometry import Polygon,LineString,Point, MultiPolygon
from std_msgs.msg import Float32MultiArray, Header
from random import uniform
from scipy.spatial import Voronoi, voronoi_plot_2d
import rospy  
import pickle
from multi_robot_system.msg import PartitionedPolygonInfo
import xml.etree.ElementTree as ET


class polygon_division:

    def __init__(self, name):
        self.exploration_area = get_param(self,'exploration_area',"/home/uib/MRS_ws/src/multi_robot_system/missions/30000.xml") 
        self.number_of_robots = get_param(self,'number_of_robots',3)
        self.pickle_path = get_param(self,'pickle_path','/home/uib/MRS_ws/src/multi_robot_system/missions/pickle/3AUVs/30000.pickle')
        self.ned_origin_lat = 39.543330
        self.ned_origin_lon = 2.377940
        self.regular_objects_number = 30
        self.priority_objects_number = 30

        self.points = np.array([])
        self.local_points=[]
        self.centroid_points = []
        self.main_polygon_centroid =[]
        self.voronoi_offset_polygons = []

        self.read_file()
        self.extract_NED_positions()
        self.divide_polygon()
        self.write_json()

    
    def write_json(self):
        data = {
            'array1': self.cluster_centroids,
            'array2': self.voronoi_polygons,
            'array3': self.main_polygon,
            'array4': self.main_polygon_centroid,
            'array5': self.voronoi_offset_polygons,
            'array6': self.regular_objects,
            'array7': self.priority_objects
        }

        with open(self.pickle_path, 'wb') as file:
            pickle.dump(data, file)
        print("...process finished")
 

    def read_file(self):
        data = []
        self.latitude = []
        self.longitude = []
        try:
            # Parsear el archivo XML
            tree = ET.parse(self.exploration_area)
            root = tree.getroot()

            # Recorrer cada "mission_step" en el XML
            for mission_step in root.findall("mission_step"):
                maneuver = mission_step.find("maneuver")

                # Extraer final_latitude y final_longitude si están presentes
                final_lat = maneuver.find("final_latitude")
                final_lon = maneuver.find("final_longitude")

                if final_lat is not None and final_lon is not None:
                    self.latitude.append(float(final_lat.text))
                    self.longitude.append(float(final_lon.text))

        except ET.ParseError as e:
            print(f"Error al analizar el archivo XML: {e}")
        except Exception as e:
            print(f"Error inesperado: {e}")
            
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

        # plt.plot(self.north_position,self.east_position)
        # plt.axis('equal')
        # plt.show()   
        

    def clustering(self):
        print("************************ CLUSTERIIIING *********************************")
        self.reduced_data = self.points
        kmeans = KMeans(init="k-means++", n_clusters=self.number_of_robots, n_init=4)
        kmeans.fit(self.reduced_data)

        # Step size of the mesh. Decrease to increase the quality of the VQ.
        h = 0.02  # point in the mesh [x_min, x_max]x[y_min, y_max].

        # Plot the decision boundary. For that, we will assign a color to each
        x_min, x_max = self.reduced_data[:, 0].min() - 1, self.reduced_data[:, 0].max() + 1
        y_min, y_max = self.reduced_data[:, 1].min() - 1, self.reduced_data[:, 1].max() + 1
        xx, yy = np.meshgrid(np.arange(x_min, x_max, h), np.arange(y_min, y_max, h))

        # Obtain labels for each point in mesh. Use last trained model.
        Z = kmeans.predict(np.c_[xx.ravel(), yy.ravel()])

        # Put the result into a color plot
        Z = Z.reshape(xx.shape)
        self.cluster_centroids = kmeans.cluster_centers_
        plt.figure(1)
        plt.clf()
        plt.imshow(
            Z,
            interpolation="nearest",
            extent=(xx.min(), xx.max(), yy.min(), yy.max()),
            cmap=plt.cm.Paired,
            aspect="auto",
            origin="lower",
        )

        plt.plot(self.reduced_data[:, 0], self.reduced_data[:, 1], "k.", markersize=2)
        # Plot the centroids as a white X
        plt.scatter(
            self.cluster_centroids[:, 0],
            self.cluster_centroids[:, 1],
            marker="x",
            s=169,
            linewidths=3,
            color="w",
            zorder=10,
        )

        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)
        plt.plot(*zip(*self.polygon_points))
        plt.xlabel("Eje X")
        plt.ylabel("Eje Y")
        plt.xticks(())
        plt.yticks(())
        # plt.show()

    # def divide_polygon(self):
    #         #obtain the global_points (lat,long) of the polygon
    #         self.global_points=[]
    #         self.global_coords=[]
    #         for i in range(len(self.latitude)):
    #             self.global_points.append([self.latitude[i],self.longitude[i]])
    #         self.global_coords.append(self.global_points[i])

    #         #obtain the local_points (lat,long) of the polygon
    #         for i in range(len(self.north_position)):
    #             self.local_points.append([self.north_position[i],self.east_position[i]])

    #         # Define the main polygon object
    #         self.main_polygon = Polygon(self.local_points)
    #         self.main_polygon_centroid = self.main_polygon.centroid
    #         self.polygon_points = self.local_points
    #         self.regular_objects = self.generate_random_points_within_polygon(self.main_polygon,self.regular_objects_number)
    #         self.priority_objects = self.generate_random_points_within_polygon(self.main_polygon,self.priority_objects_number)
            
    #         # .........................................................................
    #         # Generate random points within the polygon
    #         num_points = 500
        
    #         while len(self.centroid_points) < num_points:
    #             # Generate random coordinates within the polygon's bounds
    #             x = uniform(self.main_polygon.bounds[0], self.main_polygon.bounds[2])
    #             y = uniform(self.main_polygon.bounds[1], self.main_polygon.bounds[3])
    #             # Create a point object
    #             point = Point(x,y)
    #             # Check if the point is within the polygon
    #             if self.main_polygon.contains(point):
    #                 self.centroid_points.append([x,y])
    #         self.points = np.array(self.centroid_points)

    #         self.clustering()
    #         self.conpute_voronoi_tesselation()
    def divide_polygon(self):
        # Obtain the global points (lat, long) of the polygon
        self.global_points = []
        self.global_coords = []
        for i in range(len(self.latitude)):
            self.global_points.append([self.latitude[i], self.longitude[i]])
        self.global_coords.append(self.global_points[i])

        # Obtain the local points (north, east) of the polygon
        for i in range(len(self.north_position)):
            self.local_points.append([self.north_position[i], self.east_position[i]])

        # Define the main polygon object
        self.main_polygon = Polygon(self.local_points)
        self.main_polygon_centroid = self.main_polygon.centroid
        self.polygon_points = self.local_points
        self.regular_objects = self.generate_random_points_within_polygon(
            self.main_polygon, self.regular_objects_number)
        self.priority_objects = self.generate_random_points_within_polygon(
            self.main_polygon, self.priority_objects_number)

        # Generate random points within the polygon
        num_points = 500
        while len(self.centroid_points) < num_points:
            x = uniform(self.main_polygon.bounds[0], self.main_polygon.bounds[2])
            y = uniform(self.main_polygon.bounds[1], self.main_polygon.bounds[3])
            point = Point(x, y)
            if self.main_polygon.contains(point):
                self.centroid_points.append([x, y])
        self.points = np.array(self.centroid_points)

        # Perform clustering and compute Voronoi tessellation
        self.clustering()
        self.conpute_voronoi_tesselation()

    def conpute_voronoi_tesselation(self):
        voronoi_regions = Voronoi(self.cluster_centroids)
        regions, vertices = self.voronoi_finite_polygons_2d(voronoi_regions)
        voronoi_plot_2d(voronoi_regions)
        plt.figure(figsize=(10, 10))

        # Plot the main polygon
        plt.plot(*zip(*self.polygon_points), label="Main Polygon", color="black", lw=2)

        # Plot each Voronoi region
        self.voronoi_polygons = []
        for region in regions:
            polygon = vertices[region]
            sub_polygon = Polygon(polygon).intersection(self.main_polygon)

            if isinstance(sub_polygon, MultiPolygon):  # Handle MultiPolygon
                for poly in sub_polygon.geoms:
                    x, y = poly.exterior.xy
                    plt.fill(x, y, alpha=0.4, label="Voronoi Sub-Area")
            elif isinstance(sub_polygon, Polygon):  # Single Polygon
                x, y = sub_polygon.exterior.xy
                plt.fill(x, y, alpha=0.4, label="Voronoi Sub-Area")

            self.voronoi_polygons.append(sub_polygon)

        # Plot regular and priority objects
        for obj in self.regular_objects:
            plt.plot(obj.x, obj.y, 'bo', label="Regular Object")
        for obj in self.priority_objects:
            plt.plot(obj.x, obj.y, 'ro', label="Priority Object")

        plt.axis("equal")
        plt.xlabel("X distance [m]")
        plt.ylabel("Y distance [m]")
        plt.legend(loc="upper right")
        plt.title("Polygon with Sub-Areas and Objects")
        plt.show()
    
    def define_voronoi_offset_polygons(self,offset):
        # create voronoi_offset_polygons in order to ensure the complete coverage of the areas
        for voronoi_polygon in range(len(self.voronoi_polygons)):
            new_polygon = self.create_voronoi_offset_polygon(voronoi_polygon,offset)
            self.voronoi_offset_polygons.append(new_polygon)
    
    def create_voronoi_offset_polygon(self,polygon,offset):
        goal_polygon = self.voronoi_polygons[polygon]
        offset_polygon = goal_polygon.buffer(offset,cap_style=3, join_style=2)
        return(offset_polygon)
        

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
    
    def generate_random_points_within_polygon(self,polygon, num_points):
        # Calculate the bounding box of the polygon
        min_x, min_y, max_x, max_y = polygon.bounds
        
        random_objects = []
        while len(random_objects) < num_points:
            # Generate random coordinates within the bounding box
            x = uniform(min_x, max_x)
            y = uniform(min_y, max_y)
            point = Point(x, y)
            
            # Check if the point is within the polygon
            if polygon.contains(point):
                random_objects.append(point)
        
        return random_objects
    
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
        rospy.init_node('polygon_division')
        polygon_division = polygon_division(rospy.get_name())
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass