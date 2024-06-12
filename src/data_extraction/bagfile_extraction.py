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
from multi_robot_system.msg import CoverageStartTime,TravelledDistance,ExplorationUpdate,ObjectInformation,Communication,Distances, Data
import os
import sys
import subprocess
 
class DataExtraction:

    def __init__(self):
        self.alpha = [0,1,2,3,4,5,6,7,8,9,10]
        self.beta = [0,1,2,3,4,5,6,7,8,9,10]
        self.gamma = [0,1,2,3,4,5,6,7,8,9,10]
        self.n = [0,2,4,6,8,10]

        self.w1 = [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]
        self.w2 = [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]
        self.w3 = [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]
        self.w4 = [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]

        # self.combinations = [[10,0,0],[0,0,10],[5,0,5],[7,0,3],[3,0,7],[8,0,2],[2,0,8],[6,0,4],[4,0,6]]

        # Define the range for w1, w2, and w3
        self.w_range = [i / 10 for i in range(11)]

        self.simulation_count = -1
        self.optimization_model = 1
        self.data_path = '/home/uib/MRS_data/test_6/'

        self.response_threshold_folder = self.data_path+'response_threshold'
        self.RTM_bagfiles = self.data_path+'response_threshold/bagfiles'
        self.RTM_params = self.data_path+'response_threshold/params'
        self.RTM_csv = self.data_path+'response_threshold/csv'

        self.owa_folder = self.data_path+'owa'
        self.owa_bagfiles = self.data_path+'owa/bagfiles'
        self.owa_params = self.data_path+'owa/params'
        self.owa_csv = self.data_path+'owa/csv'

        self.max_folder = self.data_path+'max_stimulus'
        self.max_bagfiles = self.data_path+'/max_stimulus/bagfiles'
        self.max_params = self.data_path+'max_stimulus/params'
        self.max_csv = self.data_path+'max_stimulus/csv'

        self.rr_folder = self.data_path+'round_robin'
        self.rr_bagfiles = self.data_path+'round_robin/bagfiles'
        self.rr_params = self.data_path+'round_robin/params'
        self.rr_csv = self.data_path+'round_robin/csv'

        self.launchfile = 'roslaunch multi_robot_system MRS.launch'
        self.yaml_file_path = "/home/uib/MRS_ws/src/MRS_stack/multi_robot_system/config/data_extraction.yaml"

        self.topics = [
                '/mrs/allocator_communication_latency',
                '/mrs/asv0_communication_latency',
                '/mrs/asv1_communication_latency',
                '/mrs/allocator_data_buffered',
                '/mrs/asv0_data_buffered',
                '/mrs/asv1_data_buffered',
                '/mrs/allocator_data_transmited',
                '/mrs/asv0_data_transmited',
                '/mrs/asv1_data_transmited',
                '/mrs/asv_travelled_distance',
                '/robot0/travelled_distance',
                '/robot1/travelled_distance',
                '/robot2/travelled_distance',
                '/robot3/travelled_distance',
                '/robot4/travelled_distance',
                '/robot5/travelled_distance',
                '/mrs/allocator_elapsed_time',
                '/mrs/asv0_elapsed_time',
                '/mrs/asv1_elapsed_time'
            ]
        self.package_name = "multi_robot_system"
        self.launch_file = "MRS.launch"
        self.run = None

        self.process()
    
    def process(self):
        self.combinations = []
        self.create_data_folders()
        self.simulation_count = self.simulation_count+1

        # define the different combinations optimization_model and parameters
        if(self.optimization_model == 1):
            self.bagfiles_folder = self.RTM_bagfiles
            self.params_folder = self.RTM_params
            self.csv_folder = self.RTM_csv
            self.response_threshold_combinations()

        elif(self.optimization_model == 2):
            self.bagfiles_folder = self.owa_bagfiles
            self.params_folder = self.owa_params
            self.csv_folder = self.owa_csv
            # self.owas_combinations()

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


    def start_launch_file(self):
        # print("Starting launch file: {self.launch_file}")
        self.run = subprocess.Popen(['roslaunch', self.package_name, self.launch_file])

    def stop_launch_file(self):
        if self.run:
            print("Sending SIGINT to stop the launch file...")
            self.run.send_signal(signal.SIGINT)
            self.run.wait()
            os.system('sudo killall roscore')
            time.sleep(5)
            print("Launch file stopped.")

    def check_if_proces_end(self):
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith('/record_')==False):
                print("_____________________________")
                print("PROCESS KILLED CORRECTLY!!!!")
                print("_____________________________")
                os.system('killall rosmaster')
                os.system('killall roscore')
                time.sleep(10)

            if(self.simulation_count<len(self.combinations)):
                self.process()
            else:
                print(" SIMULATION FINISHED!!!!!!!!!!!!!!!!!!!")


    def set_parameters(self):  
        data = self.read_yaml()
        if(self.optimization_model==1):
            if all(key in data for key in ['optimization_model','alpha', 'gamma']):
                data['optimization_model'] = 1
                data['alpha'] = self.combinations[self.simulation_count][0]  
                # data['beta'] = self.combinations[self.simulation_count][1]  
                data['gamma'] = self.combinations[self.simulation_count][1]  

        if(self.optimization_model==2):
            if all(key in data for key in ['optimization_model','w1','w2','w3','w4']):
                # for combination in self.combinations:
                data['optimization_model'] = 2
                data['w1'] = self.combinations[self.simulation_count][0]  
                data['w2'] = self.combinations[self.simulation_count][1] 
                data['w3'] = self.combinations[self.simulation_count][2]
                data['w4'] = self.combinations[self.simulation_count][3]   

        self.write_yaml(data)

    def read_yaml(self):
        with open(self.yaml_file_path, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)
            return data
        
    def write_yaml(self, data):
        with open(self.yaml_file_path, 'w') as yaml_file:
            yaml.dump(data, yaml_file)

    def owas_combinations(self):
        # values = combinations_with_replacement(self.w1, 4)
        values = product(self.w1,self.w2,self.w3,self.w4)
        self.combinations= [combo for combo in values if (sum(combo)==1 and (combo[0]>=combo[1]>=combo[2]>=combo[3]))]
        print("*********************There are "+ str(len(self.combinations))+ " combinations.*********************")
        print(self.combinations)

    def response_threshold_combinations(self):
        for a in self.alpha:
            for g in self.gamma:
                # for g in self.gamma:
                if a + g  == 10:
                    self.combinations.append([a, g])
        print("*********************There are "+ str(len(self.combinations))+ " combinations.*********************")
        print(self.combinations)

if __name__ == "__main__":
    my_object = DataExtraction()


