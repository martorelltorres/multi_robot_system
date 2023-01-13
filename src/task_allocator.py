#!/usr/bin/env python
import rospy 
import math
import numpy as np
import random           
import matplotlib.pyplot as plt
from cola2_msgs.msg import  NavSts
from shapely.geometry import Point
from multi_robot_system.msg import TaskMonitoring 
#import classes
from area_partition import area_partition
from robot import Robot
from task_allocation_algorithms.hungarian_algorithm import Hungarian
from task_allocation_algorithms.aco import ACO


class task_allocation:

    def __init__(self, name):
        self.name = name

        # read parameter from the parameter server
        self.navigation_topic = self.get_param('~navigation_topic','/turbot/navigator/navigation') 
        self.number_of_robots = self.get_param('number_of_robots')
        self.task_allocator = self.get_param('task_allocation')
        self.robot_ID = self.get_param('~robot_ID',0) 
        self.robot_init = False

        self.polygons = []
        self.task_monitoring = []
        self.central_polygon_defined = False
        self.first_robot = True
        self.task_monitoring = []
                        # 0 --> not started
                        # 1 --> running
                        # 2 --> finished

        self.area_handler = area_partition("area_partition")
        self.robot_handler = Robot("robot")

        # subscribers
        rospy.Subscriber(self.navigation_topic ,
                         NavSts,    
                         self.update_robot_position,
                         queue_size=1)
        # publishers
        self.task_monitoring = rospy.Publisher("robot"+str(self.robot_ID)+"_task_monitoring",
                                        TaskMonitoring,
                                        queue_size=1)
        # Timers
        rospy.Timer(rospy.Duration(1.0), self.task_monitoring_publisher)
        self.update_task_status(self.robot_ID,0,0,0,[0,0],0)

    def update_robot_position(self,msg):
        self.robot_position_north = msg.position.north
        self.robot_position_east = msg.position.east
        self.robot_position_depth = msg.position.depth
        self.robot_orientation_yaw = msg.orientation.yaw 
        self.robot_id = self.robot_ID
        self.robot_init = True

    def task_allocation(self):

        # task_allocator==1 --> Split the tasks depending of the number of robots
        if(self.task_allocator==1):
            polygon_number = self.area_handler.get_polygon_number()
            # create an array with the goal polygon_ids, from 0 to n
            for polygon in range(polygon_number):
                self.polygons.append(polygon)
            self.central_polygon_id = self.define_meeting_point()
            # add the central polygon to the tasks in order to cover the hole area
            self.polygons.append(self.central_polygon_id)
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

        # task_allocator==2 --> Assign non consecutive random tasks 
        elif(self.task_allocator==2):
            polygon_number = self.area_handler.get_polygon_number()
            voronoy_polygons = self.area_handler.get_voronoi_offset_polygons()
            robot_tasks=[]

            # create an array with the goal polygon_ids, from 0 to n
            for polygon in range(polygon_number):
                self.polygons.append(polygon)

            for robot in range(self.number_of_robots):
                if(self.robot_id==robot):
                    current_nearest_polygon = self.area_handler.determine_nearest_polygon(self.robot_position_north,self.robot_position_east,voronoy_polygons)
                    robot_tasks[self.robot_id].append(current_nearest_polygon)

            #         if(current_nearest_polygon == self.nearest_polygon and self.first_robot == False):
            #             self.polygons[self.nearest_polygon]

            #         self.nearest_polygon = current_nearest_polygon
            #         robot_tasks[self.robot_id].append(self.nearest_polygon)

            # if(self.robot_id==0):
            #     self.nearest_polygon = self.area_handler.determine_nearest_polygon(self.robot_position_north,self.robot_position_east,voronoy_polygons)
            #     robot_tasks[self.robot_id].append(self.nearest_polygon)

            # for robot in range(self.number_of_robots):
            #     for task in range(polygon_number-1):

            #         robot_tasks[self.robot_id].append(self.nearest_polygon)

        #*********************************** Hungarian algorithm**************************************************
        elif(self.task_allocator==3):
            tasks_number = self.area_handler.get_polygon_number()
            estimated_time_tasks = self.area_handler.get_estimated_polygons_coverage_time()
            # battery_charge = self.robot_handler.get_battery_status()
            # task_allocation.battery_status[self.robot_ID] = battery_charge
            costs = np.array([])
            # create de cost matrix using the time_task values
            for task in range(tasks_number):
                cost_function = estimated_time_tasks[task]
                costs = np.append([costs],[cost_function])
            
            # cost_matrix = np.array([])
            # for row in range(tasks_number):
            #     cost_matrix = np.append(cost_matrix,costs)
            # cost_matrix = cost_matrix.reshape(tasks_number,tasks_number)

            # create the cost_matrix adding a random factor to the cost matrix

            # for robot in range(self.number_of_robots):
            #     cost_matrix = np.array([])
            #     random_costs =np.array([])
            #     for element in range(len(costs)):
            #         random_costs = np.append([random_costs],[costs[element]*random.uniform(1,10.5)]) 
            #         cost_matrix = np.append(cost_matrix,random_costs)
            #     print("The random costs are: " +str(random_costs))
            # # cost_matrix = cost_matrix.reshape(self.number_of_robots,tasks_number)
            # max_random_cost = max(random_costs)

            cost_matrix = np.array([])

            for robot in range(tasks_number):
                cost_matrix = np.append(cost_matrix,costs)
            cost_matrix = cost_matrix.reshape(tasks_number,tasks_number)

            cost_matrix=np.transpose(cost_matrix) 


            # # make the cost_matrix square

            # new_row = np.array([])
            # if (tasks_number>self.number_of_robots):
            #     rows_to_add = tasks_number-self.number_of_robots
            #     for element in range(tasks_number):
            #         new_row = np.append(new_row,0)

            #     for rows in range(rows_to_add):
            #         cost_matrix = np.append(cost_matrix,new_row)
            
            #     cost_matrix = cost_matrix.reshape(tasks_number,tasks_number)

            # if(self.number_of_robots > tasks_number):
            #     rows_to_add = self.number_of_robots-tasks_number
            #     new_row = np.zeros(self.number_of_robots)
            #     for rows in range(rows_to_add):
            #         cost_matrix = np.append(cost_matrix,new_row)
                
            #     cost_matrix = cost_matrix.reshape(self.number_of_robots,self.number_of_robots)

            print("The cost_matrix values are: " +str(cost_matrix))         
            self.hungarian_algortithm = Hungarian(cost_matrix)
            hungarian_output = self.hungarian_algortithm.calculate()
            print("The hungarian output is: "+str(hungarian_output))
            results = self.hungarian_algortithm.get_results()
            total_potential = self.hungarian_algortithm.get_total_potential()
            print("----------------------------------------------")
            print(results)
            print(total_potential)

        #*********************************** Particle Swarm Optimization (PSO)************************************************
        elif(self.task_allocator==4):
            tasks_number = self.area_handler.get_polygon_number()
            robots_velocity = self.robot_handler.get_robot_velocity()
            aco = ACO(self.number_of_robots,tasks_number,robots_velocity,env.targets,env.time_lim)

          

        return(self.robots_tasks,self.central_polygon_id)

    def initialize_task_status(self):
        for robot in range(self.number_of_robots):
            for task in range(len(self.robots_tasks)):
                # the self.robots_tasks has the following structure [['Robot_0', array([0, 1])], ['Robot_1', array([2, 3])]]
                robot_tasks = self.robots_tasks[robot]
                tasks = robot_tasks[1]
                self.task_monitoring.append([robot,tasks[task],0])
        # The outpus is the following one, where the first element is the robot, the second the task and finally the status
        # [[0, 0, 0], [0, 1, 0], [1, 2, 0], [1, 3, 0]]
        return(self.task_monitoring)
    
    def update_task_status(self,robot_id,task_id,status_update,central_polygon,tasks,current_section):
        self.task_msg = TaskMonitoring()
        self.task_msg.header.frame_id = "world_ned"
        self.task_msg.header.stamp = rospy.Time.now()
        self.task_msg.robot_tasks = tasks
        self.task_msg.central_polygon = central_polygon
        self.task_msg.robot_id = robot_id
        self.task_msg.current_task = task_id
        self.task_msg.task_status = status_update
        task_sections = self.area_handler.get_sections_number()
        self.task_msg.sections = task_sections[task_id]
        self.task_msg.current_section = current_section
        self.task_msg.goal_points = self.area_handler.get_goal_points(task_id)

    def task_monitoring_publisher(self,event):
        self.task_monitoring.publish(self.task_msg)

    def define_goal_task(self):
        n_robots = self.number_of_robots
        polygon_number = self.area_handler.get_polygon_number()-1
        goal_task = math.trunc(polygon_number/n_robots)
        return(goal_task,)    

    def define_meeting_point(self):
        # find the central polygon
        self.voronoi_polygons = self.area_handler.get_voronoi_polygons()
        main_polygon_centroid = self.area_handler.get_main_polygon_centroid()
        # central_polygon_id shows the number of the center_polygon referenced to the voronoy_polygons
        self.central_polygon_id = self.area_handler.get_central_polygon(self.voronoi_polygons,main_polygon_centroid)
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
        voronoy_polygons = self.area_handler.get_voronoi_polygons()
        self.centroids = self.area_handler.get_polygon_centroids()
        goal_polygon_1 = self.area_handler.determine_nearest_polygon(self.robot2_position_north,self.robot2_position_east,voronoy_polygons)

        # find the maximum distance between the goal_polygon and the other polygons
        point_A = Point(self.centroids[goal_polygon_1])
        centroids_distance =[]
        for centroid in self.centroids:
                point_B = Point(centroid)
                distance_btw_centroids = self.area_handler.distance_between_points(point_A,point_B)
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