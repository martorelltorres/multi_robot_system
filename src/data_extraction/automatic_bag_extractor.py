#!/usr/bin/env python3
import os
import subprocess
import time
from itertools import product
import yaml
import sys
import subprocess

class DataExtraction:
    def __init__(self):
        # Parámetros de ARTM
        self.alpha = [0, 2.5, 5, 7.5, 10]
        self.beta = [0, 2.5, 5, 7.5, 10]

        # Parámetros de OWA
        self.w1 = [0, 0.2, 0.4, 0.6, 0.8, 1]
        self.w2 = [0, 0.2, 0.4, 0.6, 0.8, 1]
        self.w3 = [0, 0.2, 0.4, 0.6, 0.8, 1]

        self.simulation_count = 0
        self.aggregation_model = 1
        self.yaml_file_path = "/home/uib/MRS_ws/src/multi_robot_system/config/data_extraction.yaml"
        self.launchfile = 'roslaunch multi_robot_system mrs.launch'
        self.number_of_asvs = 1  
        self.auv_range = range(3, 7)  
        self.area_range = range(10000, 70000, 10000)  
        self.base_pickle_path = "/home/uib/MRS_ws/src/multi_robot_system/missions/pickle/"
        self.base_data_path = "/home/uib/MRS_data/simulation_data/"

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

        self.run_simulations()

    def create_data_folders(self, data_path):
        os.makedirs(os.path.join(data_path, 'bagfiles'), exist_ok=True)
        os.makedirs(os.path.join(data_path, 'params'), exist_ok=True)

    def generate_combinations(self):
        if self.aggregation_model == 1:
            return [[a, b] for a, b in product(self.alpha, self.beta) if a + b == 10]
        elif self.aggregation_model == 2:
            return [combo for combo in product(self.w1, self.w2, self.w3) if sum(combo) == 1 and combo[0] >= combo[1] >= combo[2]]
        elif self.aggregation_model == 3:
            return [[None]]  # No parámetros específicos para RR

    def set_parameters(self, params, num_auvs, exploration_area):
        data = self.read_yaml()
        data['aggregation_model'] = self.aggregation_model

        # Actualizar parámetros según el modelo
        if self.aggregation_model == 1:
            data['alpha'], data['beta'] = params
        elif self.aggregation_model == 2:
            data['w1'], data['w2'], data['w3'] = params

        # Actualizar parámetros dinámicos
        data['pickle_path'] = f"{self.base_pickle_path}/{num_auvs}AUVs/{exploration_area}.pickle"
        data['number_of_robots'] = num_auvs + self.number_of_asvs
        data['number_of_auvs'] = num_auvs
        data['number_of_asvs'] = self.number_of_asvs

        self.write_yaml(data)
        

    def read_yaml(self):
        with open(self.yaml_file_path, 'r') as yaml_file:
            return yaml.safe_load(yaml_file)

    def write_yaml(self, data):
        with open(self.yaml_file_path, 'w') as yaml_file:
            yaml.dump(data, yaml_file)
        time.sleep(5)

    def run_simulations(self):
        for model in range(1, 4):  
            self.aggregation_model = model

            for num_auvs in self.auv_range:
                for exploration_area in self.area_range:
                    data_path = f"{self.base_data_path}/{exploration_area}/{num_auvs}AUVs/{self.get_model_folder(model)}"
                    self.create_data_folders(data_path)

                    combinations = self.generate_combinations()
                    self.simulation_count = 0

                    for params in combinations:
                        self.set_parameters(params, num_auvs, exploration_area)
                        self.run_simulation(model, params, data_path, num_auvs)
                        self.simulation_count += 1

    def run_simulation(self, model, params, data_path, num_auvs):
        bagfiles_folder = os.path.join(data_path, 'bagfiles')
        params_folder = os.path.join(data_path, 'params')

        # Lanzar la simulación con el parámetro number_of_auvs
        launch_command = f"{self.launchfile} number_of_auvs:={num_auvs}"
        subprocess.Popen(launch_command, shell=True)

        # Grabación de datos con rosbag
        os.chdir(bagfiles_folder)
        rosbag_command = ['rosbag', 'record'] + self.topics + ['-O', f'results_{self.simulation_count}.bag']
        rosbag_process = subprocess.Popen(rosbag_command)
        rosbag_process.wait()

        # Guardar parámetros en YAML
        param_filename = f"params_{self.simulation_count}.yaml"
        params_process = subprocess.Popen(['rosparam', 'dump', param_filename], cwd=params_folder)
        params_process.wait()

        # Finalizar nodos de ROS
        self.terminate_ros_nodes()
    
    def terminate_ros_nodes(self):
        # Obtener la lista de nodos activos
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read().decode('utf-8')  # Leer la salida del comando
        retcode = list_cmd.wait()
        if retcode != 0:
            print("Error al listar los nodos de ROS.")
            return

        # Filtrar nodos que no deben ser eliminados
        protected_nodes = ['/roscore', '/roslaunch', '/rosmaster']
        nodes_to_kill = []

        for line in list_output.split("\n"):
            if line.strip() and line.strip() not in protected_nodes:
                nodes_to_kill.append(line.strip())

        # Matar únicamente los nodos filtrados
        for node in nodes_to_kill:
            try:
                subprocess.run(["rosnode", "kill", node], check=True)
                print(f"Nodo {node} terminado correctamente.")
            except subprocess.CalledProcessError as e:
                print(f"Error al intentar terminar el nodo {node}: {e}")

        print("_____________________________")
        print("Nodos terminados correctamente (excepto los protegidos).")
        print("_____________________________")

    # def terminate_ros_nodes(self):
    #     # Get the list of ROS nodes
    #     list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    #     list_output = list_cmd.stdout.read()  # Read the command output
    #     retcode = list_cmd.wait()
    #     assert retcode == 0, f"List command returned {retcode}"
        
    #     # Decode the output and process it
    #     for line in list_output.decode('utf-8').split("\n"):
    #         if not line.startswith('/record_'):  # Check if the node name does not start with '/record_'
    #             # # Terminate rosmaster
    #             subprocess.run(["pkill", "-f", "rosmaster"], check=True)
    #             # Kill all running ROS nodes
    #             os.system("rosnode kill -a")
    #             print("_____________________________")
    #             print("PROCESS KILLED CORRECTLY!!!!")
    #             print("_____________________________")
    #             time.sleep(30)

    def get_model_folder(self, model):
        if model == 1:
            return "artm"
        elif model == 2:
            return "owa"
        elif model == 3:
            return "rr"
        return "unknown"

if __name__ == "__main__":
    DataExtraction()
