#!/usr/bin/env python
from setuptools import setup
import rospy 
import math
import random           
import matplotlib.pyplot as plt
from cola2_msgs.msg import  NavSts
from shapely.geometry import Point
#import classes
from area_partition import *

class task_allocation:

    def __init__(self, name):
        self.name = name

        # read parameter from the parameter server
        self.number_of_robots = self.get_param('number_of_robots')

        # define variables
        # self.robot1_init = False
        # self.robot2_init = False
        # self.system_init = False
        # self.setup_start_points = False
        # self.goal_polygon_robot1 = []
        # self.goal_polygon_robot2 = []
        self.polygons = []
        self.central_polygon_defined=False

        # create atributes
        self.area = area_partition("area_partition")

    def task_allocation(self):
        polygon_number = self.area.get_polygon_number()
        # create an array with the goal polygon_ids, from 0 to n
        for polygon in range(polygon_number):
            self.polygons.append(polygon)
        self.central_polygon_id = self.define_meeting_point()
        self.robot_goals = np.array_split(self.polygons,self.number_of_robots)
        # create a goal_polygons array for every robot
        robot_names = []
        self.tasks =[]
        self.names = []
        self.robots_tasks = []
        for robot in range(self.number_of_robots):
            name = "Robot_"+str(robot)
            robot_names.append(name)
            self.names = robot_names[robot]
            # tasks array stores the specific robot name and tasks:[['Robot_0', [0, 1, 2, 3]]
            self.tasks = [self.names,self.robot_goals[robot]]
            # robots_tasks array stores all the tasks arrays
            self.robots_tasks.append(self.tasks)
        # print(self.robots_tasks)
        return(self.robots_tasks)
    
    def define_goal_task(self):
        n_robots = self.number_of_robots
        polygon_number = self.area.get_polygon_number()-1
        goal_task = math.trunc(polygon_number/n_robots)
        return(goal_task)    

    def define_meeting_point(self):
        # find the central polygon
        voronoi_polygons = self.area.get_voronoi_polygons()
        main_polygon_centroid = self.area.get_main_polygon_centroid()
        # central_polygon_id shows the number of the center_polygon referenced to the voronoy_polygons
        self.central_polygon_id = self.area.get_central_polygon(voronoi_polygons,main_polygon_centroid)
        central_polygon_index = self.polygons.index(self.central_polygon_id)
        self.update_area(central_polygon_index)
        self.central_polygon_defined = True
        return(self.central_polygon_id)
        
    def update_goal_polygons(self,goal_polygon_1,goal_polygon_2):
            self.goal_polygon_robot1.append(goal_polygon_1)
            self.goal_polygon_robot2.append(goal_polygon_2)
    
    def update_area(self,polygon):
        # remove the obtained goal_polygon from the polygon array
        self.polygons.pop(polygon)
        # self.centroids.pop(polygon)
    
    def find_start_polygons(self):
        voronoy_polygons = self.area.get_voronoi_polygons()
        self.centroids = self.area.get_polygon_centroids()
        goal_polygon_1 = self.area.determine_nearest_polygon(self.robot2_position_north,self.robot2_position_east,voronoy_polygons)

        # find the maximum distance between the goal_polygon and the other polygons
        point_A = Point(self.centroids[goal_polygon_1])
        centroids_distance =[]
        for centroid in self.centroids:
                point_B = Point(centroid)
                distance_btw_centroids = self.area.distance_between_points(point_A,point_B)
                centroids_distance.append(distance_btw_centroids)

        # find the maximum distance in order to keep the vehicles safe
        max_distance = max(centroids_distance)
        max_distance_polygon = centroids_distance.index(max_distance)
        goal_polygon_2 = max_distance_polygon
        self.setup_start_points = True

        return(goal_polygon_1,goal_polygon_2)

    
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
        rospy.init_node('task_allocation')
        task_allocation = task_allocation(rospy.get_name())
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass