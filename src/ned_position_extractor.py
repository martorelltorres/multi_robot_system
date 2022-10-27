#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import Imu, MagneticField
from std_srvs.srv import Empty, EmptyResponse
import tf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import datetime
import numpy as np 
from numpy import *
from math import *
import PyKDL
import sys
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger, TriggerRequest
from cola2_msgs.msg import GoalDescriptor, NavSts, BodyVelocityReq
from cola2_msgs.srv import Goto, GotoRequest,GotoResponse
from cola2_xiroi.srv import SharePosition
from cola2_lib.utils.angles import wrap_angle
from cola2_lib.utils.ned import NED
from std_srvs.srv import Empty, EmptyRequest
from cola2_lib.rosutils import param_loader
from cola2_lib.rosutils.param_loader import get_ros_params


class AUVTracking:
    

    def __init__(self, name):
        """ Init the class """

        # Get config parameters
        self.name = name

        self.ned = NED(39.5319633484, 2.57906508446, 0.0)
        
        pose_ASV = self.ned.geodetic2ned([39.5317860306,
                                2.57898033966,
                                0.0])
        pose_NED = self.ned.geodetic2ned([39.5319633484,
                        2.57906508446,
                        0.0])

        wp_1 = self.ned.geodetic2ned([39.53161531489686,
                2.57879732224885,
                0.0])

        wp_2 = self.ned.geodetic2ned([39.53162808069977,
                2.5788579598126646,
                0.0])

        wp_3 = self.ned.geodetic2ned([39.53022469723246,
                2.579350659459291,
                0.0])

        wp_4 = self.ned.geodetic2ned([39.53021193142645,
                2.579290021805248,
                0.0])

        print(pose_ASV)
        print(pose_NED)
        print("-----------")
        print(wp_1)
        print(wp_2)
        print(wp_3)
        print(wp_4)
        


#         
#         <latitude>39.53021193142645</latitude>
#         <longitude>2.579290021805248</longitude>

#         
#         <latitude>39.53021193142645</latitude>
#         <longitude>2.579290021805248</longitude>
#         
#         <latitude>39.53161531489686</latitude>
#         <longitude>2.57879732224885</longitude>
#         

if __name__ == '__main__':
    try:
        rospy.init_node('AUV_tracking')
        AUV_tracking = AUVTracking(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
