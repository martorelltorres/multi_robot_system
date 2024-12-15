#!/usr/bin/env python3
import math
import numpy as np
import random 
import csv
import time
from math import *
from itertools import product
import time
import signal
import psutil
import yaml
import matplotlib
import pickle
import actionlib       
import matplotlib.pyplot as plt
from std_msgs.msg import Int16, Bool, Float64MultiArray,Float32MultiArray,Float32
from geometry_msgs.msg  import PointStamped
from cola2_msgs.msg import  NavSts,BodyVelocityReq
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from cola2_msgs.srv import Goto, GotoRequest
from multi_robot_system.msg import CoverageStartTime,TravelledDistance,ExplorationUpdate,Communication,Distances, Data
import os
import sys
import subprocess
 
class DataExtraction:

    def __init__(self):
        # ARTM
        self.alpha = [0,2.5,5,7.5,10]
        self.beta = [0,2.5,5,7.5,10]

        # OWA
        self.w1 = [0,0.2,0.4,0.6,0.8,1]
        self.w2 = [0,0.2,0.4,0.6,0.8,1]
        self.w3 = [0,0.2,0.4,0.6,0.8,1]

        # Define the range for w1, w2, and w3
        self.w_range = [i / 10 for i in range(11)]

        self.simulation_count = -1
        self.aggregation_model = 1
        self.data_path = '/home/uib/MRS_data/NN/40000/4AUVs/'

        self.response_threshold_folder = self.data_path+'response_threshold'
        self.RTM_bagfiles = self.data_path+'response_threshold/bagfiles'
        self.RTM_params = self.data_path+'response_threshold/params'
        self.RTM_csv = self.data_path+'response_threshold/csv'

        self.owa_folder = self.data_path+'owa'
        self.owa_bagfiles = self.data_path+'owa/bagfiles'
        self.owa_params = self.data_path+'owa/params'
        self.owa_csv = self.data_path+'owa/csv'

        self.launchfile = 'roslaunch multi_robot_system mrs.launch'
        self.yaml_file_path = "/home/uib/MRS_ws/src/multi_robot_system/config/data_extraction.yaml"

        self.topics = [
                '/mrs/allocator_communication_latency',
                '/mrs/asv0_communication_latency',
                '/mrs/allocator_data_buffered',
                '/mrs/asv0_data_buffered',
                '/mrs/allocator_data_transmited',
                '/mrs/asv0_data_transmited',
                '/mrs/asv_travelled_distance',
                '/robot0/travelled_distance',
                '/robot1/travelled_distance',
                '/robot2/travelled_distance',
                '/robot3/travelled_distance',
                '/robot4/travelled_distance',
                '/robot5/travelled_distance',
                '/mrs/allocator_elapsed_time',
                '/mrs/asv0_elapsed_time',
                '/mrs/asv0_priority_communication_latency',
                '/mrs/asv0_regular_communication_latency',
                '/mrs/aggregation_model_info'
            ]
        
        self.package_name = "multi_robot_system"
        self.launch_file = "mrs.launch"
        self.run = None
        self.process()
    
    def process(self):
        self.combinations = []
        self.create_data_folders()
        self.simulation_count = self.simulation_count+1

        # define the different combinations aggregation_model and parameters
        if(self.aggregation_model == 1):
            self.bagfiles_folder = self.RTM_bagfiles
            self.params_folder = self.RTM_params
            # self.csv_folder = self.RTM_csv
            self.response_threshold_combinations()

        elif(self.aggregation_model == 2):
            self.bagfiles_folder = self.owa_bagfiles
            self.params_folder = self.owa_params
            # self.csv_folder = self.owa_csv
            self.owas_combinations()

        # Set the simulation parameters
        self.set_parameters()
        # Launch the simulation
        subprocess.Popen(self.launchfile, shell=True)

        # Start recording the data 
        os.chdir(self.bagfiles_folder)
        # Flatten the topics list into individual arguments
        rosbag_command = ['rosbag', 'record'] + self.topics + ['-O', 'results_' + str(self.simulation_count) + '.bag']
        # Launch the subprocess
        launch_process = subprocess.Popen(rosbag_command)
        launch_process.wait()

        param_filename = "params_"+ str(self.simulation_count)+".yaml"
        params_process = subprocess.Popen('rosparam dump ' + param_filename, shell=True, cwd=self.params_folder)
        params_process.wait()

        self.check_if_proces_end()
    
    def create_data_folders(self):
        if not os.path.exists(self.response_threshold_folder):
            os.makedirs(self.response_threshold_folder)
        if not os.path.exists(self.RTM_bagfiles):
            os.makedirs(self.RTM_bagfiles)
        if not os.path.exists(self.RTM_params):
            os.makedirs(self.RTM_params)

        if not os.path.exists(self.owa_folder):
            os.makedirs(self.owa_folder)
        if not os.path.exists(self.owa_bagfiles):
            os.makedirs(self.owa_bagfiles)
        if not os.path.exists(self.owa_params):
            os.makedirs(self.owa_params)

    def check_if_proces_end(self):
        # Get the list of ROS nodes
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()  # Read the command output
        retcode = list_cmd.wait()
        assert retcode == 0, f"List command returned {retcode}"
        
        # Decode the output and process it
        for line in list_output.decode('utf-8').split("\n"):
            if not line.startswith('/record_'):  # Check if the node name does not start with '/record_'
                # # Terminate rosmaster
                subprocess.run(["pkill", "-f", "rosmaster"], check=True)
                # # Terminate roscore
                # subprocess.run(["pkill", "-f", "roscore"], check=True)
                # Kill all running ROS nodes
                os.system("rosnode kill -a")
                print("_____________________________")
                print("PROCESS KILLED CORRECTLY!!!!")
                print("_____________________________")
                time.sleep(20)
        
        # Proceed to the next step in the simulation
        if self.simulation_count < len(self.combinations):
            self.process()
        else:
            self.aggregation_model = 2
            self.process()               

    def set_parameters(self):  
        data = self.read_yaml()
        if(self.aggregation_model==1):
            if all(key in data for key in ['aggregation_model','alpha', 'beta']):
                data['aggregation_model'] = 1
                data['alpha'] = self.combinations[self.simulation_count][0]  
                data['beta'] = self.combinations[self.simulation_count][1]  

        if(self.aggregation_model==2):
            if all(key in data for key in ['aggregation_model','w1','w2','w3']):
                # for combination in self.combinations:
                data['aggregation_model'] = 2
                data['w1'] = self.combinations[self.simulation_count][0]  
                data['w2'] = self.combinations[self.simulation_count][1] 
                data['w3'] = self.combinations[self.simulation_count][2]

        self.write_yaml(data)

    def read_yaml(self):
        with open(self.yaml_file_path, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)
            return data
        
    def write_yaml(self, data):
        with open(self.yaml_file_path, 'w') as yaml_file:
            yaml.dump(data, yaml_file)

    def owas_combinations(self):
        values = product(self.w1,self.w2,self.w3)
        self.combinations= [combo for combo in values if (sum(combo)==1 and (combo[0]>=combo[1]>=combo[2]))]
        print("*********************There are "+ str(len(self.combinations))+ " combinations.*********************")
        print(self.combinations)


    def response_threshold_combinations(self):
        for a in self.alpha:
            for b in self.beta:
                if a + b  == 10:
                    self.combinations.append([a, b])
        print("*********************There are "+ str(len(self.combinations))+ " combinations.*********************")
        print(self.combinations)

if __name__ == "__main__":
    my_object = DataExtraction()


