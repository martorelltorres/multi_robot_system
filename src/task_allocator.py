#!/usr/bin/env python
from setuptools import setup
import rospy            
import matplotlib.pyplot as plt
from cola2_msgs.msg import  NavSts
from shapely.geometry import Point
#import classes
from area_partition import *
from robot import robot

class task_allocation:

    def __init__(self, name):
        self.name = name
       
        # get params from the param server
        robot_1 = get_param(self,'/task_allocator/robot1')
        robot_2 = get_param(self,'/task_allocator/robot2')
        self.number_of_robots = get_param(self,'/task_allocator/number_of_robots')

        # define variables
        self.robot1_init = False
        self.robot2_init = False
        self.system_init = False
        self.setup_start_points = False
        self.goal_polygon_robot1 = []
        self.goal_polygon_robot2 = []

        #Subscribers
        rospy.Subscriber(robot_2+"/navigator/navigation",
                        NavSts,    
                        self.get_robot2_position,
                        queue_size=1)

        rospy.Subscriber(robot_1+"/navigator/navigation",
                        NavSts,    
                        self.get_robot1_position,
                        queue_size=1)
        
        # create atributes
        self.area = area_partition("area_partition")
        self.robot = robot("robot")
      

    def get_robot1_position(self,msg):
        self.robot1_position_north = msg.position.north
        self.robot1_position_east = msg.position.east
        self.robot1_position_depth = msg.position.depth
        self.robot1_orientation_yaw = msg.orientation.yaw 
        self.robot1_init = True
        self.initialization()

    def get_robot2_position(self,msg):
        self.robot2_position_north = msg.position.north
        self.robot2_position_east = msg.position.east
        self.robot2_position_depth = msg.position.depth
        self.robot2_orientation_yaw = msg.orientation.yaw 
        self.robot2_init = True
        self.initialization()
        self.main()

    def initialization(self):
        if(self.robot2_init == True and self.robot1_init==True):
            self.system_init = True
        else:
            self.system_init = False

    def main(self):
        if(self.system_init==True and self.setup_start_points==False):
            goal_polygon_1,goal_polygon_2 = self.find_start_polygons()
            self.update_goal_polygons(goal_polygon_1,goal_polygon_2)
            self.update_area(goal_polygon_1)
            self.update_area(goal_polygon_2)
        
        # define the central polygon
        voronoi_polygons = self.area.get_voronoi_polygons()
        main_polygon_centroid = self.area.get_main_polygon_centroid()
        central_polygon = self.area.get_central_polygon(voronoi_polygons,main_polygon_centroid)
        self.update_area(central_polygon)

        # print("111111111111111111111")
        print(self.goal_polygon_robot1)
        print(self.goal_polygon_robot2)
        print(self.polygons)
        print(self.centroids)

        
        # for polygon in self.polygons:
        #     x,y = self.centroids[self.goal_polygon_robot1[polygon]]
            


                
        # return(goal_polygon_robot1,goal_polygon_robot2)
    def update_goal_polygons(self,goal_polygon_1,goal_polygon_2):
            self.goal_polygon_robot1.append(goal_polygon_1)
            self.goal_polygon_robot2.append(goal_polygon_2)
    
    def update_area(self,polygon):
        # remove the obtained goal_polygon from the polygons and centroid arrays 
        self.polygons.pop(polygon)
        self.centroids.pop(polygon)
    
    def find_start_polygons(self):
        polygon_number = self.area.get_polygon_number()
        # create an array with the polygon_id
        for polygon in range(polygon_number):
            self.polygons.append(polygon)

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