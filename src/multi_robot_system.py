#!/usr/bin/env python
import rospy
import tf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import datetime
from numpy import *
import actionlib
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, PolygonStamped, Point32, Polygon
from cola2_msgs.msg import  NavSts
from visualization_msgs.msg import Marker
#import classes
from area_partition import area_partition

class MultiRobotSystem:
    
    def __init__(self, name):
        """ Init the class """
        # Get config parameters
        self.name = name
        # self.get_config()
        self.ned_origin_lat = self.get_param(self,'/xiroi/navigator/ned_latitude')
        self.ned_origin_lon = self.get_param(self,'/xiroi/navigator/ned_longitude')    
        self.area_handler =  area_partition("area_partition")
        self.goal_points = []
        self.check_current_position = True
        self.success_result = False
        self.data_gattered = False
        
        
        # Show initialization message
        rospy.loginfo('[%s]: initialized', self.name)

        # mrs_control/send_position service
        rospy.wait_for_service('mrs_control/send_position', 10)
        try:
            self.send_position_srv = rospy.ServiceProxy(
                        'mrs_control/send_position', Empty)
        except rospy.ServiceException, e:
            rospy.logwarn("%s: Service call failed: %s", self.name, e)

        #Subscribers
        rospy.Subscriber("/turbot/pilot/world_section_req/result",
                         WorldSectionActionResult,    
                         self.update_section_result,
                         queue_size=1)

        rospy.Subscriber("/xiroi/navigator/navigation",
                         NavSts,    
                         self.update_robot_position,
                         queue_size=1)
        #Publishers
        self.polygon_pub = rospy.Publisher("polygon_area",
                                        PolygonStamped,
                                        queue_size=1)
        # Init periodic check timer
        rospy.Timer(rospy.Duration(1.0), self.print_polygon)

        #Actionlib section client
        self.section_action = actionlib.SimpleActionClient("/turbot/pilot/world_section_req", WorldSectionAction)
        self.section_action.wait_for_server()

    def update_section_result(self,msg):
        self.final_status = msg.result.final_status
        # Possible section ending conditions
        # uint64 final_status
        # uint64 SUCCESS=0
        # uint64 TIMEOUT=1
        # uint64 FAILURE=2
        # uint64 BUSY=3

    def update_robot_position(self,msg):
        self.position_north = msg.position.north
        self.position_east = msg.position.east
        self.yaw = msg.orientation.yaw
        if(self.check_current_position == True):
            self.goal_polygon = self.area_handler.determine_nearest_polygon(self.position_north,self.position_east)
            self.mrs_coverage()
            self.check_current_position = False

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
                    polygon_points.z = 10
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

    def wait_until_section_reached(self):
        if(self.final_status==0):
            self.success_result = True
        
    def mrs_coverage(self):
        self.data_gattered = True
        # obtain all the polygons goal_points
        self.goal_points = self.area_handler.define_path_coverage(self.goal_polygon)
        self.polygon_goal_points = self.goal_points[self.goal_polygon]
        all_sections = self.polygon_goal_points[1]

        for self.section in range((len(all_sections)-1)):
           
            # First section
            if (self.section==0):
                current_section = all_sections[self.section]
                initial_point = current_section[1]
                final_point = current_section[0]
                self.send_section_strategy(initial_point,final_point)
                self.wait_until_section_reached()

            #if is an odd number
            elif(self.section%2==0 & self.success_result):
                current_section = all_sections[self.section]
                initial_point = current_section[1]
                final_point = current_section[0]
                self.send_section_strategy(initial_point,final_point)
                self.wait_until_section_reached()
            #if is an even number
            elif (self.success_result):
                current_section = all_sections[self.section]
                initial_point = current_section[0]
                final_point = current_section[1]
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
        section_req.initial_position.z = 10
        section_req.initial_yaw = self.yaw
        section_req.final_position.x = final_position_x
        section_req.final_position.y = final_position_y
        section_req.final_position.z = 10
        section_req.altitude_mode = False
        section_req.tolerance.x = 2
        section_req.tolerance.y = 2
        section_req.tolerance.z = 2
        section_req.controller_type = WorldSectionGoal.LOSCTE
        section_req.priority = GoalDescriptor.PRIORITY_NORMAL
        section_req.surge_velocity = 1
        section_req.timeout = 6000

        # send section goale using actionlib
        self.is_section_actionlib_running = True
        self.success_result = False
        self.section_action.send_goal(section_req)

        #  Wait for result or cancel if timed out
        self.section_action.wait_for_result()

    def anchor_marker(self):
        self.anchor = Marker()
        self.anchor.header.frame_id = "world_ned"
        self.anchor.header.stamp = rospy.Time.now()
        self.anchor.ns = "mrs"
        self.anchor.id = 0
        self.anchor.type = Marker.LINE_STRIP
        self.anchor.action = Marker.ADD
        self.anchor.pose.position.x = self.auv_position_north
        self.anchor.pose.position.y = self.auv_position_east
        self.anchor.pose.position.z = 0
        self.anchor.pose.orientation.x = 0
        self.anchor.pose.orientation.y = 0
        self.anchor.pose.orientation.z = 0
        self.anchor.pose.orientation.w = 1.0
        self.anchor.scale.x = self.anchor_radius*2
        self.anchor.scale.y = self.anchor_radius*2
        self.anchor.scale.z = 0.1
        self.anchor.color.r = 0.0
        self.anchor.color.g = 1.0
        self.anchor.color.b = 0.0
        self.anchor.color.a = 0.3
        self.markerPub_anchor.publish(self.anchor)

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