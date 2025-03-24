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
    parameter_combinations = [[4, 4, 2], [6, 2, 2], [6, 4, 0], [8, 2, 0], [10, 0, 0]]  # OWA
if "response" in bagfile_path.lower():
    print("The bagfile path contains the word 'response'.")
    parameter_combinations = [[0, 10], [2.5, 7.5], [5, 5], [7.5, 2.5], [10, 0]]  # ARTM

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

# Save all data to a DataFrame
df = pd.DataFrame(all_data)

# Separate parameter_combination into columns
if "owa" in bagfile_path.lower():
    df[['w1', 'w2', 'w3']] = pd.DataFrame(df['parameter_combination'].tolist(), index=df.index)
    df = df.drop(columns=['parameter_combination'])
    # Reorder columns to place w1, w2, w3 at the beginning
    column_order = ['w1', 'w2', 'w3'] + [col for col in df.columns if col not in ['w1', 'w2', 'w3']]
    df = df[column_order]
if "response" in bagfile_path.lower():
    df[['a', 'b']] = pd.DataFrame(df['parameter_combination'].tolist(), index=df.index)
    df = df.drop(columns=['parameter_combination'])
    # Reorder columns to place a, b at the beginning
    column_order = ['a', 'b'] + [col for col in df.columns if col not in ['a', 'b']]
    df = df[column_order]

# Normalize and insert below original columns
columns_to_normalize = ['regular_latency','reg_std_latency', 'priority_latency', 'prior_std_latency','transmitted_data','travelled_distance','regular_objects','priority_objects']
for column in columns_to_normalize:
    min_val = df[column].min()
    max_val = df[column].max()
    normalized_column = (df[column] - min_val) / (max_val - min_val)
    df.insert(df.columns.get_loc(column) + 1, column + '_normalized', normalized_column)

# Define constants for R and C
# Calculate R and C using normalized values

df['priority_objects'] = df['priority_objects_normalized']/(1+df['prior_std_latency_normalized']+df['priority_latency_normalized'])
df['regular_objects'] = (0.5*df['regular_objects_normalized'])/(1+df['reg_std_latency_normalized']+df['regular_latency_normalized'])
df['distance'] = 1/(1+df['travelled_distance_normalized'])

df['utility'] = df['priority_objects'] + df['regular_objects'] + df['distance']    

# Save to CSV
output_csv_path = os.path.join(bagfile_path, "data.csv")
df.to_csv(output_csv_path, index=False)

print("\nDATA EXTRACTION PROCESS FINISHED")
