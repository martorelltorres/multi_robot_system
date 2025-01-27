#!/usr/bin/env python3
import rosbag
import csv
import os
import numpy as np
import pandas as pd

# Define paths and parameters
base_path = "/home/uib/MRS_data/simulation_data"
# areas = [10000, 20000, 40000, 60000]  # Exploration areas
areas = [ 60000]  # Exploration areas
auv_counts = ["3AUVs", "4AUVs", "5AUVs", "6AUVs"]  # Number of AUVs
aggregation_methods = ["owa", "artm"]  # Aggregation methods

# Topics of interest
topics_of_interest = [
    "/mrs/allocator_communication_latency",
    "/mrs/asv_travelled_distance",
    "/mrs/allocator_data_transmited",
    "/mrs/allocator_data_buffered",
    "/mrs/asv0_priority_communication_latency",
    "/mrs/asv0_regular_communication_latency",
    "/mrs/aggregation_model_info"
]

# Define parameter combinations for each method
parameter_combinations_map = {
    "owa": [[4, 4, 2], [6, 2, 2], [6, 4, 0], [8, 2, 0], [10, 0, 0]],  # OWA
    "artm": [[0, 10], [2.5, 7.5], [5, 5], [7.5, 2.5], [10, 0]]  # ARTM
}

# Create lists to store OWA and response threshold data
owa_data = []
response_threshold_data = []

# Iterate over all combinations of area, AUVs, and aggregation methods
for area in areas:
    for auv_count in auv_counts:
        for method in aggregation_methods:
            bagfile_path = os.path.join(base_path, str(area), auv_count, method, "bagfiles")
            if not os.path.exists(bagfile_path):
                print(f"Directory not found: {bagfile_path}")
                continue

            print(f"Processing directory: {bagfile_path}")

            # Get all bag files in the directory
            all_files = os.listdir(bagfile_path)
            bag_files = [os.path.join(bagfile_path, filename) for filename in all_files if filename.endswith('.bag')]
            print(f"Found {len(bag_files)} bagfiles in {bagfile_path}")

            # Get parameter combinations for the method
            parameter_combinations = parameter_combinations_map.get(method, [])

            # Store data for this specific folder
            folder_data = []

            # Process each bag file
            for bag_file_index in range(len(bag_files)):
                bag = rosbag.Bag(os.path.join(bagfile_path, f"results_{bag_file_index}.bag"))
                start_time = bag.get_start_time()

                # Arrays to store latency values
                reg_latency_values = np.array([])
                prior_latency_values = np.array([])

                sum_data, sum_reg_objects, sum_prior_objects, travelled_distance = 0, 0, 0, 0
                parameter_combination = parameter_combinations[bag_file_index % len(parameter_combinations)]

                for topic, msg, t in bag.read_messages(topics=topics_of_interest):
                    if "/mrs/asv0_regular_communication_latency" in topic:
                        reg_latency = getattr(msg, 'comm_latency', (0, 0, 0, 0, 0, 0))
                        reg_time_latency = (msg.header.stamp.secs - start_time) / 60
                        sum_reg_latency = sum(reg_latency)
                        reg_latency_values = np.append(reg_latency_values, sum_reg_latency)

                    if "/mrs/asv0_priority_communication_latency" in topic:
                        prior_latency = getattr(msg, 'comm_latency', (0, 0, 0, 0, 0, 0))
                        sum_prior_latency = sum(prior_latency)
                        prior_latency_values = np.append(prior_latency_values, sum_prior_latency)

                    if "/mrs/asv_travelled_distance" in topic:
                        travelled_distance = msg.travelled_distance

                    if "/mrs/allocator_data_transmited" in topic:
                        data_transmitted = getattr(msg, 'transmitted_data', (0, 0, 0, 0, 0, 0))
                        regular_objects = getattr(msg, 'transmitted_regular_objects', (0, 0, 0, 0, 0, 0))
                        priority_objects = getattr(msg, 'transmitted_priority_objects', (0, 0, 0, 0, 0, 0))
                        sum_data = sum(data_transmitted)
                        sum_reg_objects = sum(regular_objects)
                        sum_prior_objects = sum(priority_objects)

                bag.close()

                # Calculate latency and standard deviation values
                reg_latency = np.mean(reg_latency_values) if len(reg_latency_values) > 0 else 0
                prior_latency = np.mean(prior_latency_values) if len(prior_latency_values) > 0 else 0

                # Prepare parameter values based on method
                if method == "owa":
                    w1, w2, w3 = parameter_combination
                    # Append data to folder_data list
                    folder_data.append({
                        'area': int(area),
                        'auv_count': int(auv_count.replace('AUVs', '')),
                        'method': method,
                        'w1': float(w1),
                        'w2': float(w2),
                        'w3': float(w3),
                        'regular_latency': reg_latency,
                        'priority_latency': prior_latency,
                        'transmitted_data': sum_data,
                        'travelled_distance': travelled_distance
                    })
                elif method == "artm":
                    a, b = parameter_combination
                    # Append data to folder_data list
                    folder_data.append({
                        'area': int(area),
                        'auv_count': int(auv_count.replace('AUVs', '')),
                        'method': method,
                        'a': float(a),
                        'b': float(b),
                        'regular_latency': reg_latency,
                        'priority_latency': prior_latency,
                        'transmitted_data': sum_data,
                        'travelled_distance': travelled_distance
                    })

            # Convert folder data to DataFrame and normalize columns
            df_folder = pd.DataFrame(folder_data)

            # Normalize directly and inversely within the folder
            columns_to_inv_normalize = ['regular_latency', 'priority_latency']
            for column in columns_to_inv_normalize:
                min_val = df_folder[column].min()
                max_val = df_folder[column].max()
                normalized_column = 1 - (df_folder[column] - min_val) / (max_val - min_val)
                df_folder[column + '_inv_normalized'] = normalized_column

            columns_to_normalize = ['transmitted_data', 'travelled_distance']
            for column in columns_to_normalize:
                min_val = df_folder[column].min()
                max_val = df_folder[column].max()
                normalized_column = (df_folder[column] - min_val) / (max_val - min_val)
                df_folder[column + '_normalized'] = normalized_column

            # Define constants for R and C
            alpha, beta, gamma, delta, epsilon = 0.3, 0.2, 0.5, 0.7, 0.3
            df_folder['R'] = (alpha * df_folder['priority_latency_inv_normalized'] +
                              beta * df_folder['regular_latency_inv_normalized'] +
                              gamma * df_folder['transmitted_data_normalized'])
            df_folder['C'] = (delta * df_folder['priority_latency_inv_normalized'] +
                              epsilon * df_folder['regular_latency_inv_normalized'])
            df_folder['utility'] = df_folder['R'] - df_folder['C']

            # Append folder data to the respective method's list
            if method == "owa":
                owa_data.extend(df_folder.to_dict('records'))
            elif method == "artm":
                response_threshold_data.extend(df_folder.to_dict('records'))

# Convert the OWA and response threshold data to separate DataFrames
owa_df = pd.DataFrame(owa_data)
response_threshold_df = pd.DataFrame(response_threshold_data)

# Save the DataFrames to separate CSV files
owa_csv_path = os.path.join(base_path, "owa_data.csv")
response_threshold_csv_path = os.path.join(base_path, "artm_data.csv")

owa_df.to_csv(owa_csv_path, index=False)
response_threshold_df.to_csv(response_threshold_csv_path, index=False)

print("\nOWA and Response Threshold Data Extraction Finished")
