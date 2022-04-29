#!/usr/bin/env python
import rospy            
from cola2_lib.utils.ned import NED
import matplotlib.pyplot as plt
from cola2_msgs.msg import WorldSectionAction,WorldSectionGoal,GoalDescriptor,WorldSectionGoal,WorldSectionActionResult
from cola2_msgs.msg import  NavSts
#import classes
from area_partition import *
from robot import robot

class task_allocation:

    def __init__(self, name):
        self.name = name
        # get params from the param server
        self.robot_1 = get_param(self,'/task_allocator/robot1')
        self.robot_2 = get_param(self,'/task_allocator/robot2')
        self.number_of_robots = get_param(self,'/task_allocator/number_of_robots')
        # create object classes
        self.area = area_partition("area_partition")
        self.robot = robot("robot")
        # define variables
        self.robot1_init = False
        self.robot2_init = False

        #Subscribers
        rospy.Subscriber(self.robot_1+"/navigator/navigation",
                         NavSts,    
                         self.get_robot1_position,
                         queue_size=1)

        rospy.Subscriber(self.robot_2+"/navigator/navigation",
                    NavSts,    
                    self.get_robot2_position,
                    queue_size=1)
        self.main()

    def get_robot1_position(self,msg):
        self.robot1_position_north = msg.position.north
        self.robot1_position_east = msg.position.east
        self.robot1_position_depth = msg.position.depth
        self.robot1_orientation_yaw = msg.orientation.yaw 
        self.robot1_init = True

    def get_robot2_position(self,msg):
        self.robot2_position_north = msg.position.north
        self.robot2_position_east = msg.position.east
        self.robot2_position_depth = msg.position.depth
        self.robot2_orientation_yaw = msg.orientation.yaw 
        self.robot2_init = True
        self.initialization()

    def initialization(self):
        if(self.robot2_init == True & self.robot1_init==True):
            self.system_init = True
        else:
            self.system_init = False

    def main(self):
        goal_polygon_robot1 = []
        goal_polygon_robot2 = []
        if(self.system_init == True):
            polygon_number = self.area.get_polygon_number()
            voronoy_polygons = self.area.get_voronoi_polygons()
            goal_polygon = self.area.determine_nearest_polygon(self.robot2_position_north,self.robot2_position_east,voronoy_polygons)
            voronoy_polygons.remove(goal_polygon)
            centroids = self.area.get_polygon_centroids()
            print("111111111111111111111111111")
            print(centroids)
            # goal_polygon_robot1.append(goal_polygon)


    
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