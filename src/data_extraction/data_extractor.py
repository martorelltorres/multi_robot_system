#!/usr/bin/env python3
import rosbag
import csv
import os
import numpy as np
import pandas as pd
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Process ROS bag files and extract data.")
parser.add_argument(
    "--bagfile_path",
    type=str,
    required=True,
    help="Path to the folder containing the ROS bag files."
)
args = parser.parse_args()

# Use the input argument for the bagfile path
bagfile_path = args.bagfile_path

topics_of_interest = ["/mrs/allocator_communication_latency",
                      "/mrs/asv_travelled_distance",
                      "/mrs/allocator_data_transmited",
                      "/mrs/allocator_data_buffered",
                      "/mrs/asv0_priority_communication_latency",
                      "/mrs/asv0_regular_communication_latency",
                      "/mrs/aggregation_model_info"]

all_files = os.listdir(bagfile_path)
# Get a list of all bag files in the folder
bag_files = [os.path.join(bagfile_path, filename) for filename in all_files if filename.endswith('.bag')]
print("I found " + str(len(bag_files)) + " bagfiles!!")

# Define parameter combinations
if "owa" in bagfile_path.lower():
    print("The bagfile path contains the word 'owa'.")
    parameter_combinations = [[4,4,2],[6,2,2],[6,4,0],[8,2,0],[10,0,0]] # OWA
if "response" in bagfile_path.lower():
    print("The bagfile path contains the word 'response'.")
    parameter_combinations = [[0, 10], [2.5, 7.5], [5, 5], [7.5, 2.5], [10, 0]] # ARTM

# Create a list to store data from all bag files
all_data = []

# Extract data from each bag file one by one
for bag_file in range(len(bag_files)):
    bag = rosbag.Bag(bagfile_path + "/results_" + str(bag_file) + ".bag")
    start_time = bag.get_start_time()

    # Arrays to store latency values
    reg_latency_values = np.array([]) 
    reg_time_latency_values = np.array([])
    prior_latency_values = np.array([]) 
    prior_time_latency_values = np.array([])

    sum_data, sum_reg_objects, sum_prior_objects, travelled_distance = 0, 0, 0, 0
    aggregation_model = None
    parameter_combination = parameter_combinations[bag_file % len(parameter_combinations)]

    for topic, msg, t in bag.read_messages(topics=topics_of_interest):
        if "/mrs/asv0_regular_communication_latency" in topic:
            reg_latency = getattr(msg, 'comm_latency', (0, 0, 0, 0, 0, 0))
            reg_time_latency = (msg.header.stamp.secs - start_time) / 60
            sum_reg_latency = sum(reg_latency)
            reg_latency_values = np.append(reg_latency_values, sum_reg_latency)
            reg_time_latency_values = np.append(reg_time_latency_values, reg_time_latency)
        
        if "/mrs/asv0_priority_communication_latency" in topic:
            prior_latency = getattr(msg, 'comm_latency', (0, 0, 0, 0, 0, 0))
            prior_time_latency = (msg.header.stamp.secs - start_time) / 60
            sum_prior_latency = sum(prior_latency)
            prior_latency_values = np.append(prior_latency_values, sum_prior_latency)
            prior_time_latency_values = np.append(prior_time_latency_values, prior_time_latency)
                        
        if "/mrs/asv_travelled_distance" in topic:
            travelled_distance = msg.travelled_distance

        if "/mrs/allocator_data_transmited" in topic:
            data_transmited = getattr(msg, 'transmitted_data', (0, 0, 0, 0, 0, 0))
            regular_objects = getattr(msg, 'transmitted_regular_objects', (0, 0, 0, 0, 0, 0))
            priority_objects = getattr(msg, 'transmitted_priority_objects', (0, 0, 0, 0, 0, 0))
            sum_data = sum(data_transmited)
            sum_reg_objects = sum(regular_objects)
            sum_prior_objects = sum(priority_objects)

        if "/mrs/aggregation_model_info" in topic:
            aggregation_model = getattr(msg, 'model_name', 'unknown')

    bag.close()

    # Calculate latency and standard deviation values
    reg_latency = sum(reg_latency_values) / len(reg_latency_values) if len(reg_latency_values) > 0 else 0
    reg_std = np.std(reg_latency_values) if len(reg_latency_values) > 0 else 0
    prior_latency = sum(prior_latency_values) / len(prior_latency_values) if len(prior_latency_values) > 0 else 0
    prior_std = np.std(prior_latency_values) if len(prior_latency_values) > 0 else 0

    # Append data to all_data list
    all_data.append({
        'parameter_combination': parameter_combination,
        'regular_latency': reg_latency,
        'reg_std_latency': reg_std,
        'priority_latency': prior_latency,
        'prior_std_latency': prior_std,
        'transmitted_data': sum_data,
        'regular_objects': sum_reg_objects,
        'priority_objects': sum_prior_objects,
        'travelled_distance': travelled_distance
    })
    print(f"Extracting data from results_{bag_file}.bag complete.")

# Save all data to a CSV after processing all bag files
output_csv_path = os.path.join(bagfile_path, "data.csv")
df = pd.DataFrame(all_data)

# Normalize inversely only the specified columns and insert below original columns
columns_to_inv_normalize = ['regular_latency', 'priority_latency']
for column in columns_to_inv_normalize:
    min_val = df[column].min()
    max_val = df[column].max()
    normalized_column = 1 - (df[column] - min_val) / (max_val - min_val)
    df.insert(df.columns.get_loc(column) + 1, column + '_inv_normalized', normalized_column)

# Normalize and insert below original columns
columns_to_normalize = ['reg_std_latency', 'prior_std_latency','transmitted_data','travelled_distance']
for column in columns_to_normalize:
    min_val = df[column].min()
    max_val = df[column].max()
    normalized_column = (df[column] - min_val) / (max_val - min_val)
    df.insert(df.columns.get_loc(column) + 1, column + '_normalized', normalized_column)

# Define constants for R and C
# Calculate R and C using normalized values
alpha, beta, gamma, delta, epsilon = 0.3, 0.1, 0.6, 0.5, 0.5  # Example values
df['R'] = alpha * df['priority_latency_inv_normalized'] + beta * df['regular_latency_inv_normalized']+gamma * df['transmitted_data_normalized']
df['C'] = (delta * df['prior_std_latency_normalized'] +
           epsilon * df['reg_std_latency_normalized'] )

# Calculate Utility U = R - C
df['Utility'] = df['R'] - df['C']

# Save to CSV
df.to_csv(output_csv_path, index=False)

print("\nDATA EXTRACTION PROCESS FINISHED")
