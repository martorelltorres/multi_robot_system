#!/usr/bin/env python

import rospy
from cola2_msgs.msg import  NavSts
import numpy as np
from multi_robot_system.msg import TaskMonitoring

#import classes
from task_allocator import task_allocation

# 1) wait a predefined time to communicate info between two specific robots
# 2) wait until the first robot ends a hole task to communicate info between robots
# 3) fix the central polygon as a meeting communication point

class communication_strategy:

    def __init__(self, name):
        self.name = name
        self.number_of_robots = self.get_param('number_of_robots')
        self.task_allocation_handler = task_allocation("task_allocation")
        self.goals,self.central_polygon = self.task_allocation_handler.task_allocation()
        self.system_init = False
        self.robot_tasks = []
        self.task_status = []

        node_name = rospy.get_name()

        #Subscribers
        for robot in range(self.number_of_robots):
            rospy.Subscriber("/robot"+str(robot+1)+"/task_monitoring",
                                TaskMonitoring,    
                                self.monitoring_robot_tasks,
                                robot,
                                queue_size=1)
    
    def monitoring_robot_tasks(self, msg, robot):
        self.robot_id = msg.robot_id
        self.goal_polygons = self.goals[self.robot_id][1]
        self.current_task = msg.task
        self.task_status = msg.status

        
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
        rospy.init_node('communication_strategy')
        communication_strategy = communication_strategy(rospy.get_name())
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass


