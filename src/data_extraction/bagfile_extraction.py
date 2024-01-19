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
        self.alpha = [0,1,2,3,4,5,6,7,8,9,10]
        self.beta = [0,1,2,3,4,5,6,7,8,9,10]
        self.gamma = [0,1,2,3,4,5,6,7,8,9,10]
        self.n = [0,2,4,6,8,10]

        # self.w1 = [0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5]
        # self.w2 = [0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5]
        # self.w3 = [0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5]
        # self.w4 = [0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5]

        self.simulation_count = -1
        self.optimization_model = 1
        self.combinations = []

        self.response_threshold_folder ='/mnt/storage_disk/extracted_results/response_threshold'
        self.RTM_bagfiles = '/mnt/storage_disk/extracted_results/response_threshold/data'
        self.RTM_params = '/mnt/storage_disk/extracted_results/response_threshold/params'
        self.RTM_csv = '/mnt/storage_disk/extracted_results/response_threshold/csv'

        self.owa_folder ='/mnt/storage_disk/extracted_results/owa'
        self.owa_bagfiles = '/mnt/storage_disk/extracted_results/owa/bagfiles'
        self.owa_params = '/mnt/storage_disk/extracted_results/owa/params'
        self.owa_csv = '/mnt/storage_disk/extracted_results/owa/csv'

        self.max_folder ='/mnt/storage_disk/extracted_results/max_stimulus'
        self.max_bagfiles = '/mnt/storage_disk/extracted_results/max_stimulus/bagfiles'
        self.max_params = '/mnt/storage_disk/extracted_results/max_stimulus/params'
        self.max_csv = '/mnt/storage_disk/extracted_results/max_stimulus/csv'

        self.rr_folder ='/mnt/storage_disk/extracted_results/round_robin'
        self.rr_bagfiles = '/mnt/storage_disk/extracted_results/round_robin/bagfiles'
        self.rr_params = '/mnt/storage_disk/extracted_results/round_robin/params'
        self.rr_csv = '/mnt/storage_disk/extracted_results/round_robin/csv'

        self.launchfile = 'roslaunch multi_robot_system MRS.launch'
        # self.yaml_file_path = "/mnt/storage_disk/MRS_ws/src/MRS_stack/multi_robot_system/config/data_extraction.yaml"
        self.yaml_file_path = "/home/uib/MRS_ws/src/MRS_stack/multi_robot_system/config/data_extraction.yaml"
        self.process()
    
    def process(self):
        self.create_data_folders()
        self.simulation_count = self.simulation_count+1
        # define the different combinations optimization_model and parameters
        if(self.optimization_model == 1):
            self.bagfiles_folder = self.RTM_bagfiles
            self.params_folder = self.RTM_params
            self.csv_folder = self.RTM_csv
            self. response_threshold_combinations()

        elif(self.optimization_model == 2):
            self.bagfiles_folder = self.owa_bagfiles
            self.params_folder = self.owa_params
            self.csv_folder = self.owa_csv
            self.owas_combinations()

        # Set the simulation parameters
        self.set_parameters()
        # Launch the simulation
        subprocess.Popen(self.launchfile, shell=True)

        # Start recording the data 
        os.chdir(self.bagfiles_folder)
        launch_process = subprocess.Popen(['rosbag', 'record', '-a', '-O', 'results_'+ str(self.simulation_count)+'.bag'])
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
        if not os.path.exists(self.RTM_csv):
            os.makedirs(self.RTM_csv)

        if not os.path.exists(self.owa_folder):
            os.makedirs(self.owa_folder)
        if not os.path.exists(self.owa_bagfiles):
            os.makedirs(self.owa_bagfiles)
        if not os.path.exists(self.owa_params):
            os.makedirs(self.owa_params)
        if not os.path.exists(self.owa_csv):
            os.makedirs(self.owa_csv)

        if not os.path.exists(self.max_folder):
            os.makedirs(self.max_folder)
        if not os.path.exists(self.max_bagfiles):
            os.makedirs(self.max_bagfiles)
        if not os.path.exists(self.max_params):
            os.makedirs(self.max_params)
        if not os.path.exists(self.max_csv):
            os.makedirs(self.max_csv)

        if not os.path.exists(self.rr_folder):
            os.makedirs(self.rr_folder)
        if not os.path.exists(self.rr_bagfiles):
            os.makedirs(self.rr_bagfiles)
        if not os.path.exists(self.rr_params):
            os.makedirs(self.rr_params)
        if not os.path.exists(self.rr_csv):
            os.makedirs(self.rr_csv)

    def check_if_proces_end(self):
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith('/record_')==False):
                os.system('killall -9 rosmaster')

                if(self.simulation_count<len(self.combinations) and self.optimization_model==1 ):
                    self.process()
                else:
                    self.simulation_count = -1
                    self.optimization_model = self.optimization_model +1
                    self.process()

    def set_parameters(self):  
        data = self.read_yaml()
        if(self.optimization_model==1):
            if all(key in data for key in ['optimization_model','alpha', 'beta', 'gamma']):
                data['optimization_model'] = 1
                data['alpha'] = self.combinations[self.simulation_count][0]  
                data['beta'] = self.combinations[self.simulation_count][1]  
                data['gamma'] = self.combinations[self.simulation_count][2]  

        if(self.optimization_model==2):
            if all(key in data for key in ['optimization_model','w1','w2','w3','w4']):
                # for combination in self.combinations:
                data['optimization_model'] = 2
                data['w1'] = self.combinations[self.simulation_count][0]  
                data['w2'] = self.combinations[self.simulation_count][1] 
                data['w3'] = self.combinations[self.simulation_count][2]
                data['w4'] = self.combinations[self.simulation_count][3]   

        self.write_yaml(data)

    def owas_combinations(self):
        # Generate all possible combinations
        values = product(self.w1, self.w2, self.w3, self.w4)
        # Filter combinations where W1 + W2 + W3 + W4 equals 1
        self.combinations = [combo for combo in values if sum(combo) == 1]
        print("****************There are "+ str(len(self.combinations))+ " combinations.*********************")

    def response_threshold_combinations(self):
        for a in self.alpha:
            for b in self.beta:
                for g in self.gamma:
                    if a + b + g == 10:
                        self.combinations.append([a, b, g])
        print("*********************There are "+ str(len(self.combinations))+ " combinations.*********************")
        print(self.combinations)
    def read_yaml(self):
        with open(self.yaml_file_path, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)
            return data

    def write_yaml(self, data):
        with open(self.yaml_file_path, 'w') as yaml_file:
            yaml.dump(data, yaml_file)

if __name__ == "__main__":
    my_object = DataExtraction()


