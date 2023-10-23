# -*- coding: utf-8 -*-

# Simulation with several parameter values for different CA algorithms and strategies

import os
import subprocess
import time
import yaml

global alpha
global beta
global gamma
global n

global w1
global w2
global w3

global combinations
global simulation_count


alpha = [0,2,4,6,8,10]
beta = [0,2,4,6,8,10]
gamma = [0,2,4,6,8,10]
n = [0,2,4,6,8,10]

w1 = [0/10,2/10,4/10,6/10,8/10,10/10]
w1 = [0/10,2/10,4/10,6/10,8/10,10/10]
w3 = [0/10,2/10,4/10,6/10,8/10,10/10]

simulation_count = 0
combinations = []


def main():
    simulation_count = 0
    response_threshold_combinations()
    # owas_combinations()
    launch_file_path = '/mnt/storage_disk/MRS_ws/src/MRS_stack/multi_robot_system/launch/MRS.launch'

    while True:
        # Modify parameters.yaml
        # If all parameter values have been simulated then stop simulating
        if modify_parameters():
            break

        # Start simulation in a separate process
        launch_process = subprocess.Popen(['roslaunch', launch_file_path])

        while True:
            if check_ros_processes():
                # Record results topic, make sure that the folder has the correct w/r rights
                os.chdir('/mnt/storage_disk/extracted_results')
                bag_process = subprocess.Popen(['rosbag', 'record', '-a', '-O', 'results_'+ str(simulation_count)+'.bag'])
                break
            time.sleep(5)

        # Wait for the launch process to finish before continuing
        launch_process.wait()

        # Wait for the bag file to be created before running rostopic
        bag_file_path = '/mnt/storage_disk/extracted_results/results_'+str(simulation_count)+'.bag'
        while not os.path.exists(bag_file_path):
            time.sleep(1)
        
        simulation_count = simulation_count + 1

        # # Run rostopic to extract data
        # rostopic_process = subprocess.Popen(['rostopic', 'echo', '-b', bag_file_path, '-p', '/results'], stdout=subprocess.PIPE)
        # csv_data, _ = rostopic_process.communicate()

        # # Append the extracted data to the CSV file
        # with open('testing_results.csv', 'a') as csv_file:
        #     csv_file.write(csv_data.decode())  # Write the data to the CSV file

        # # Remove the bag file if it still exists
        # if os.path.exists(bag_file_path):
        #     os.remove(bag_file_path)

def owas_combinations():
    for element_w1 in w1:
        for element_w2 in w2:
            for element_w3 in w3:
                if element_w1 + element_w2 + element_w3 == 1:
                    combinations.append((element_w1, element_w2, element_w3))

def response_threshold_combinations():
    for a in alpha:
        for b in beta:
            for g in gamma:
                if a + b + g == 10:
                    combinations.append([a, b, g])

def check_ros_processes():
    process = subprocess.Popen('rosnode list', shell=True, stdout=subprocess.PIPE)
    output = process.stdout.read().decode()
    return output

def modify_parameters():
    global alpha
    global beta
    global gamma
    global n
    global combinations
    global simulation_count
   
    yaml_file_path = "/mnt/storage_disk/MRS_ws/src/MRS_stack/multi_robot_system/config/data_extraction.yaml"

    with open(yaml_file_path, 'r') as yaml_file:
            data = yaml.load(yaml_file, Loader=yaml.Loader)

            if 'config' in data and isinstance(data['config'], list):

                for param in data['config']:
                    if param['name'] == 'alpha':
                        param['alpha'] = combinations[simulation_count][0]
                    
                    elif param['name'] == 'beta':
                        param['beta'] = combinations[simulation_count][1]

                    elif param['name'] == 'gamma':
                        param['gamma'] = combinations[simulation_count][2]

    with open(yaml_file_path, 'w') as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False)

    if (simulation_count > len(combinations)):
        return True
    else:
        return False

if __name__ == "__main__":
    main()