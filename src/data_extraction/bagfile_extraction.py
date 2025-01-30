#!/usr/bin/env python3
import os
import subprocess
import time
import yaml
from itertools import product
import rospy


class DataExtraction:
    def __init__(self, number_of_auvs, area_exploration,aggregation_model):
        self.number_of_auvs = number_of_auvs
        self.aggregation_model = aggregation_model
        self.number_of_asvs = 1
        self.number_of_robots = self.number_of_auvs + self.number_of_asvs
        self.area_exploration = area_exploration

        # ARTM parameters
        self.alpha = [0, 2.5, 5, 7.5, 10]
        self.beta = [0, 2.5, 5, 7.5, 10]

        # OWA parameters
        self.w1 = [0, 0.2, 0.4, 0.6, 0.8, 1]
        self.w2 = [0, 0.2, 0.4, 0.6, 0.8, 1]
        self.w3 = [0, 0.2, 0.4, 0.6, 0.8, 1]

        self.simulation_count = -1

        self.data_path = '/home/uib/MRS_data/simulation_data/'+ str(self.area_exploration)+'/'+str(self.number_of_auvs)+'AUVs/'

        self.response_threshold_folder = os.path.join(self.data_path, 'artm')
        self.RTM_bagfiles = os.path.join(self.response_threshold_folder, 'bagfiles')
        self.RTM_params = os.path.join(self.response_threshold_folder, 'params')

        self.owa_folder = os.path.join(self.data_path, 'owa')
        self.owa_bagfiles = os.path.join(self.owa_folder, 'bagfiles')
        self.owa_params = os.path.join(self.owa_folder, 'params')

        self.yaml_file_path = "/home/uib/MMRS_ws/src/MMRS_stack/multi_robot_system/config/data_extraction.yaml"

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
            '/mrs/aggregation_model_info',
        ]

        self.launch_file = "mrs.launch"
        self.launch_command = f"roslaunch multi_robot_system {self.launch_file} number_of_auvs:={self.number_of_auvs}"
        self.process()

    def process(self):
        self.combinations = []
        self.create_data_folders()
        self.simulation_count += 1

        if self.aggregation_model == 1:
            self.bagfiles_folder = self.RTM_bagfiles
            self.params_folder = self.RTM_params
            self.response_threshold_combinations()

        elif self.aggregation_model == 2:
            self.bagfiles_folder = self.owa_bagfiles
            self.params_folder = self.owa_params
            self.owas_combinations()

        self.set_parameters()
        rospy.sleep(1)
        subprocess.Popen(self.launch_command, shell=True)

        os.chdir(self.bagfiles_folder)
        rosbag_command = ['rosbag', 'record'] + self.topics + ['-O', f'results_{self.simulation_count}.bag']
        launch_process = subprocess.Popen(rosbag_command)
        launch_process.wait()

        param_filename = f"params_{self.simulation_count}.yaml"
        params_process = subprocess.Popen(f'rosparam dump {param_filename}', shell=True, cwd=self.params_folder)
        params_process.wait()

        self.check_if_process_end()

    def create_data_folders(self):
        os.makedirs(self.response_threshold_folder, exist_ok=True)
        os.makedirs(self.RTM_bagfiles, exist_ok=True)
        os.makedirs(self.RTM_params, exist_ok=True)

        os.makedirs(self.owa_folder, exist_ok=True)
        os.makedirs(self.owa_bagfiles, exist_ok=True)
        os.makedirs(self.owa_params, exist_ok=True)

    def check_if_process_end(self):
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, f"List command returned {retcode}"

        for line in list_output.decode('utf-8').split("\n"):
            if not line.startswith('/record_'):
                subprocess.run(["pkill", "-f", "rosmaster"], check=True)
                os.system("rosnode kill -a")
                print("Process terminated correctly!")
                time.sleep(30)

            if self.simulation_count < len(self.combinations):
                self.process()
            else:
                self.aggregation_model = 2
                self.process()

    def set_parameters(self):
        data = self.read_yaml()
        data['number_of_auvs'] = self.number_of_auvs
        data['area_exploration'] = self.area_exploration
        data['number_of_asvs'] = self.number_of_asvs
        data['number_of_robots'] = self.number_of_robots
        data['pickle_path'] =  f'/home/uib/MMRS_ws/src/MMRS_stack/multi_robot_system/missions/pickle/{number_of_auvs}AUVs/{area_exploration}.pickle'

        if self.aggregation_model == 1:
            data['aggregation_model'] = 1
            data['alpha'] = self.combinations[self.simulation_count][0]
            data['beta'] = self.combinations[self.simulation_count][1]

        if self.aggregation_model == 2:
            data['aggregation_model'] = 2
            data['w1'] = self.combinations[self.simulation_count][0]
            data['w2'] = self.combinations[self.simulation_count][1]
            data['w3'] = self.combinations[self.simulation_count][2]

        self.write_yaml(data)

    def read_yaml(self):
        with open(self.yaml_file_path, 'r') as yaml_file:
            return yaml.safe_load(yaml_file)

    def write_yaml(self, data):
        with open(self.yaml_file_path, 'w') as yaml_file:
            yaml.dump(data, yaml_file)

    def owas_combinations(self):
        values = product(self.w1, self.w2, self.w3)
        self.combinations = [combo for combo in values if sum(combo) == 1 and combo[0] >= combo[1] >= combo[2]]
        print(f"There are {len(self.combinations)} combinations.")
        print(self.combinations)

    def response_threshold_combinations(self):
        self.combinations = [[a, b] for a, b in product(self.alpha, self.beta) if a + b == 10]
        print(f"There are {len(self.combinations)} combinations.")
        print(self.combinations)


if __name__ == "__main__":
    number_of_auvs = int(input("Enter the number of AUVs: "))
    area_exploration = input("Enter the area of exploration: ")
    aggregation_model = input("Enter the aggregation model 1--> ARTM   2--> OWA: ")
    DataExtraction(number_of_auvs, area_exploration,aggregation_model)
