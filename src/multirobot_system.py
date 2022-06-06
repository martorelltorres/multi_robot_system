#!/usr/bin/env python
import rospy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import datetime
from numpy import *
from shapely.geometry import Polygon,LineString,Point
import actionlib
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult,VehicleStatus
from std_srvs.srv import Empty
from geometry_msgs.msg import  PolygonStamped, Point32, Polygon
from cola2_msgs.msg import  NavSts


#import classes
from area_partition import area_partition
from task_allocator import task_allocation
from robot import robot

class MultiRobotSystem:
    
    def __init__(self, name):
        """ Init the class """
       
        self.name = name
         # Get config parameters from the parameter server
        self.robot_ID = self.get_param('~robot_ID',0)   
        self.tolerance = self.get_param('tolerance',2)
        self.navigation_topic = self.get_param('~navigation_topic','/turbot/navigator/navigation') 
        self.section_action = self.get_param('~section_action','/turbot/pilot/world_section_req') 
        self.section_result = self.get_param('~section_result','/turbot/pilot/world_section_req/result') 
        self.number_of_robots = self.get_param('number_of_robots')
        self.surge_velocity = self.get_param('surge_velocity',0.5)

        self.area_handler =  area_partition("area_partition")
        self.task_allocation_handler = task_allocation("task_allocation")
        self.robot_handler = robot("robot")

        self.system_initialization = True
        self.success_result = False
        self.data_gattered = False
        self.points = []

        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        #Subscribers
        rospy.Subscriber(self.section_result,
                         WorldSectionActionResult,    
                         self.update_section_result,
                         queue_size=1)

        rospy.Subscriber(self.navigation_topic ,
                         NavSts,    
                         self.update_robot_position,
                         queue_size=1)

        #Publishers
        self.polygon_pub = rospy.Publisher("voronoi_polygons",
                                        PolygonStamped,
                                        queue_size=1)

        self.polygon_offset_pub = rospy.Publisher("voronoi_offset_polygons",
                                        PolygonStamped,
                                        queue_size=1)

        # Init periodic timers
        rospy.Timer(rospy.Duration(1.0), self.print_polygon)
        rospy.Timer(rospy.Duration(1.0), self.print_offset_polygon)

        #Actionlib section client
        self.section_strategy = actionlib.SimpleActionClient(self.section_action, WorldSectionAction)
        self.section_strategy.wait_for_server()

        self.initialization()

    def update_section_result(self,msg):
        self.final_status = msg.result.final_status

    def update_robot_position(self,msg):
        self.robot_position_north = msg.position.north
        self.robot_position_east = msg.position.east
        self.robot_position_depth = msg.position.depth
        self.robot_orientation_yaw = msg.orientation.yaw 
    
    def initialization(self): 
        # wait 4 seconds in order to initialize the different robot architectures
        rospy.sleep(5)
        if(self.system_initialization==True):
            self.system_initialization = False 
            self.goals,self.central_polygon = self.task_allocation_handler.task_allocation()
            # [['Robot_0', array([0, 1])], ['Robot_1', array([2, 3])]]
            self.goal_polygons = self.goals[self.robot_ID][1]
            print("The central polygon meeting point is the polygon: "+str(self.central_polygon))
            print("The robot_"+str(self.robot_ID)+" has the following goals: "+str(self.goal_polygons))
            times = self.area_handler.get_estimated_polygons_coverage_time()
            print(times)
            self.goal_points = self.area_handler.define_path_coverage()
            self.robot_task_assignement()

    def robot_task_assignement(self):            
        for task in range(len(self.goal_polygons)):
            print("The robot_"+str(self.robot_ID)+" is covering the polygon: "+str(self.goal_polygons[task]))
            self.mrs_coverage(self.goal_polygons[task])

    def wait_until_section_reached(self):
        if(self.final_status==0):
            self.success_result = True    

    def mrs_coverage(self,goal):
        self.task_allocation_handler.update_task_status(self.robot_ID,goal,1,self.central_polygon)
        self.data_gattered = True
        section_points = self.goal_points[goal]

        for section in range(len(section_points)):
            self.task_allocation_handler.update_task_status(self.robot_ID,goal,2,self.central_polygon)
            current_section = section_points[section]
            self.generate_initial_section(self.robot_position_north,self.robot_position_east,current_section)

            # Check the order of the initial and final points, set the initial point to the nearest point and the final to the furthest point 
            first_point = current_section[0]
            first_point_distance = self.robot_handler.get_robot_distance_to_point(self.robot_position_north,self.robot_position_east,first_point[0],first_point[1])
            second_point = current_section[1]
            second_point_distance = self.robot_handler.get_robot_distance_to_point(self.robot_position_north,self.robot_position_east,second_point[0],second_point[1])

            if(first_point_distance < second_point_distance):
                initial_point = first_point
                final_point = second_point
            else:
                initial_point = second_point
                final_point = first_point

            self.send_section_strategy(initial_point,final_point)
            self.wait_until_section_reached()

        self.task_allocation_handler.update_task_status(self.robot_ID,goal,3,self.central_polygon)
    
    def generate_initial_section(self,position_north,position_east,section_points):
        initial_point = Point(position_north,position_east)
        initial_point = list(initial_point.coords)[0]
        final_point = section_points[0]
        self.send_section_strategy(initial_point,final_point)
        self.wait_until_section_reached()

    def send_section_strategy(self,initial_point,final_point):
        initial_position_x = initial_point[0]
        final_position_x = final_point[0]
        initial_position_y = initial_point[1]
        final_position_y = final_point[1]

        section_req = WorldSectionGoal()
        section_req.initial_position.x = initial_position_x
        section_req.initial_position.y = initial_position_y
        section_req.initial_position.z = 0.0
        # obtain the yaw of each robot
        
        section_req.initial_yaw = self.robot_orientation_yaw
        section_req.final_position.x = final_position_x
        section_req.final_position.y = final_position_y
        section_req.final_position.z = 0.0
        section_req.altitude_mode = False
        section_req.tolerance.x = self.tolerance
        section_req.tolerance.y = self.tolerance
        section_req.tolerance.z = self.tolerance
        section_req.controller_type = WorldSectionGoal.LOSCTE
        section_req.priority = GoalDescriptor.PRIORITY_NORMAL
        section_req.surge_velocity = self.surge_velocity
        section_req.timeout = 6000

        # send section goale using actionlib
        self.success_result = False
        self.section_strategy.send_goal(section_req)

        #  Wait for result or cancel if timed out
        self.section_strategy.wait_for_result()

    def print_polygon(self,event):
        if(self.data_gattered==True):      
            points = []
            polygon_number = self.area_handler.get_polygon_number()
            polygon_number = polygon_number -1
            for polygon in range(polygon_number):
                polygon_coords_x,polygon_coords_y = self.area_handler.get_polygon_points(polygon)

                for coord in range(len(polygon_coords_x)):
                    polygon_points = Point32()
                    polygon_points.x = polygon_coords_x[coord]
                    polygon_points.y = polygon_coords_y[coord]
                    polygon_points.z = 0
                    points.append(polygon_points)

                    # create polygon message
                    polygon_msg = Polygon()
                    polygon_msg.points = points
                    
                    # create polygon_stamped message
                    polygon_stamped_msg = PolygonStamped()
                    polygon_stamped_msg.header.frame_id = "world_ned"
                    polygon_stamped_msg.header.stamp = rospy.Time.now()
                    polygon_stamped_msg.polygon = polygon_msg
                    self.polygon_pub.publish(polygon_stamped_msg)
        else:
            rospy.logwarn("Unable to print polygon, data not gattered yet")      

    def print_offset_polygon(self,event):
        if(self.data_gattered==True):      
            points = []
            polygon_number = self.area_handler.get_polygon_number()
            polygon_number = polygon_number -1
            for polygon in range(polygon_number):
                polygon_coords_x,polygon_coords_y = self.area_handler.get_offset_polygon_points(polygon)

                for coord in range(len(polygon_coords_x)):
                    polygon_points = Point32()
                    polygon_points.x = polygon_coords_x[coord]
                    polygon_points.y = polygon_coords_y[coord]
                    polygon_points.z = 0
                    points.append(polygon_points)

                    # create polygon message
                    polygon_msg = Polygon()
                    polygon_msg.points = points
                    
                    # create polygon_stamped message
                    polygon_stamped_msg = PolygonStamped()
                    polygon_stamped_msg.header.frame_id = "world_ned"
                    polygon_stamped_msg.header.stamp = rospy.Time.now()
                    polygon_stamped_msg.polygon = polygon_msg
                    self.polygon_offset_pub.publish(polygon_stamped_msg)
        else:
            rospy.logwarn("Unable to print polygon, data not gattered yet")  

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
        rospy.init_node('multi_robot_system')
        return_to_NED = MultiRobotSystem(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass