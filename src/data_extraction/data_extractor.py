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
parameter_combinations = [[0, 10], [2.5, 7.5], [5, 5], [7.5, 2.5], [10, 0]]

# Create a list to store data from all bag files
all_data = []

# Extract data from each bag file one by one
for bag_file in range(len(bag_files)):
    # Open the bag file
    bag = rosbag.Bag(bagfile_path + "/results_" + str(bag_file) + ".bag")
    # Get the start time of the bag file
    start_time = bag.get_start_time()

    # Extract data from the specified topics and write to the CSV file
    reg_latency_values = np.array([]) 
    reg_time_latency_values = np.array([])
    prior_latency_values = np.array([]) 
    prior_time_latency_values = np.array([])

    prior_objects = []
    reg_objects = []

    aggregation_model = None
    parameter_combination = parameter_combinations[bag_file % len(parameter_combinations)]  # Assign parameter combination

    for topic, msg, t in bag.read_messages(topics=topics_of_interest):
        init_time = msg.header.stamp.secs

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
            # data
            sum_data = sum(data_transmited)
            # regular_objects
            sum_reg_objects = sum(regular_objects)
            # priority_objects
            sum_prior_objects = sum(priority_objects)

        if "/mrs/aggregation_model_info" in topic:
            aggregation_model = getattr(msg, 'model_name', 'unknown')

    # Close the bag file after extraction
    bag.close()

    # handle REGULAR OBJECTS latency values
    reg_latency = sum(reg_latency_values) / len(reg_latency_values) if len(reg_latency_values) > 0 else 0
    reg_std = np.std(reg_latency_values) if len(reg_latency_values) > 0 else 0

    # handle PRIORITY OBJECTS latency values
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
        'priority_objects': sum_prior_objects,
        'regular_objects': sum_reg_objects,
        'travelled_distance': travelled_distance,
        'regular_latency_values': list(reg_latency_values),
        'regular_time_latency': list(reg_time_latency_values),
        'priority_latency_values': list(prior_latency_values),
        'priority_time_latency': list(prior_time_latency_values)
    })

    print(f"Extracting data from results_{bag_file}.bag complete.")

# Save all data to a CSV after processing all bag files
output_csv_path = os.path.join(bagfile_path, "data.csv")
df = pd.DataFrame(all_data)
df.to_csv(output_csv_path, index=False)

print("   ")
print("DATA EXTRACTION PROCESS FINISHED")