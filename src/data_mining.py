import math
import numpy as np
import random 
import csv
import time
from math import *
import time
import signal
import psutil
import yaml
import matplotlib
import pickle
import actionlib       
import matplotlib.pyplot as plt
from std_msgs.msg import Int16, Bool, Int16MultiArray,Float32MultiArray,Float32
from geometry_msgs.msg  import PointStamped
from cola2_msgs.msg import  NavSts,BodyVelocityReq
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from cola2_msgs.srv import Goto, GotoRequest
from multi_robot_system.msg import CoverageStartTime,TravelledDistance,ExplorationUpdate,CommunicationDelay,ObjectInformation,Communication,Distances, Data
import os
import sys
import subprocess

 
class DataExtraction:

    def __init__(self):

        self.alpha = [0,2,4,6,8,10]
        self.beta = [0,2,4,6,8,10]
        self.gamma = [0,2,4,6,8,10]
        self.n = [0,2,4,6,8,10]
        self.w1 = [0/10,2/10,4/10,6/10,8/10,10/10]
        self.w1 = [0/10,2/10,4/10,6/10,8/10,10/10]
        self.w3 = [0/10,2/10,4/10,6/10,8/10,10/10]
        self.simulation_count = -1
        self.combinations = []
        self.robot_id = 0
        self.exploration_tasks_update = np.array([False, False, False, False, False, False])
        self.launchfile = 'roslaunch multi_robot_system MRS.launch'
        self.bagfile = "rosbag record -a"
        self.yaml_file_path = "/mnt/storage_disk/MRS_ws/src/MRS_stack/multi_robot_system/config/data_extraction.yaml"
    
        # Define the different parameter combinations
        self. response_threshold_combinations()
        # self.owas_combinations()
        self.process()
    
    def process(self):
        self.simulation_count = self.simulation_count+1
        # Set the simulation parameters
        self.set_parameters()
        # Launch the simulation
        subprocess.Popen(self.launchfile, shell=True)
        # Start recording the data in a bagfile
        os.chdir('/mnt/storage_disk/extracted_results')
        launch_process = subprocess.Popen(['rosbag', 'record', '-a', '-O', 'results_'+ str(self.simulation_count)+'.bag'])
        launch_process.wait()
        self.check_if_proces_end()

    def check_if_proces_end(self):
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith('/record_')==False):
                # os.system('pkill ros')
                os.system('killall -9 rosmaster')
                self.process()


    def set_parameters(self):  
        print("**********************************SIMULATION COUN: "+str(self.simulation_count)+"*********************************")
        with open(self.yaml_file_path, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)

        if 'parameter_values' in data and isinstance(data['parameter_values'], list):
            for param in data['parameter_values']:
                if param['name'] == 'alpha':
                    param['alpha'] = self.combinations[self.simulation_count][0]
                
                elif param['name'] == 'beta':
                    param['beta'] = self.combinations[self.simulation_count][1]

                elif param['name'] == 'gamma':
                    param['gamma'] = self.combinations[self.simulation_count][2]

        with open(self.yaml_file_path, 'w') as yaml_file:
            yaml.dump(data, yaml_file)

    def owas_combinations(self):
        for element_w1 in self.w1:
            for element_w2 in self.w2:
                for element_w3 in self.w3:
                    if element_w1 + element_w2 + element_w3 == 1:
                        self.combinations.append((element_w1, element_w2, element_w3))

    def response_threshold_combinations(self):
        for a in self.alpha:
            for b in self.beta:
                for g in self.gamma:
                    if a + b + g == 10:
                        self.combinations.append([a, b, g])

if __name__ == "__main__":
    my_object = DataExtraction()


