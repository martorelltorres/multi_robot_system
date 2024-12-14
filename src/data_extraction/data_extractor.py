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
bag_files = [os.path.join(bagfile_path, filename) for filename in all_files if filename.endswith('.bag')]

print(f"I found {len(bag_files)} bagfiles!!")

# Consolidate all data
all_data = []

for bag_file in range(len(bag_files)):
    bag = rosbag.Bag(bagfile_path + f"/results_{bag_file}.bag")
    start_time = bag.get_start_time()

    reg_latency_values = np.array([])
    reg_time_latency_values = np.array([])
    prior_latency_values = np.array([])
    prior_time_latency_values = np.array([])

    sum_data = sum_prior_objects = sum_reg_objects = travelled_distance = 0

    for topic, msg, t in bag.read_messages(topics=topics_of_interest):
        if "/mrs/asv0_regular_communication_latency" in topic:
            reg_latency = msg.comm_latency
            reg_time_latency = (msg.header.stamp.secs - start_time) / 60
            reg_latency_values = np.append(reg_latency_values, sum(reg_latency))
            reg_time_latency_values = np.append(reg_time_latency_values, reg_time_latency)

        if "/mrs/asv0_priority_communication_latency" in topic:
            prior_latency = msg.comm_latency
            prior_time_latency = (msg.header.stamp.secs - start_time) / 60
            prior_latency_values = np.append(prior_latency_values, sum(prior_latency))
            prior_time_latency_values = np.append(prior_time_latency_values, prior_time_latency)

        if "/mrs/asv_travelled_distance" in topic:
            travelled_distance = msg.travelled_distance

        if "/mrs/allocator_data_transmited" in topic:
            data_transmited = msg.transmitted_data
            regular_objects = msg.transmitted_regular_objects
            priority_objects = msg.transmitted_priority_objects
            sum_data += sum(data_transmited)
            sum_reg_objects += sum(regular_objects)
            sum_prior_objects += sum(priority_objects)

        if "/mrs/aggregation_model_info" in topic:
            aggregation_model = msg.aggregation_model
            alpha = msg.alpha
            beta = msg.beta
            w1 = msg.w1
            w2 = msg.w2
            w3 = msg.w3

    bag.close()

    # Calculate statistics
    reg_latency = reg_latency_values.mean() if len(reg_latency_values) > 0 else 0
    reg_std = reg_latency_values.std() if len(reg_latency_values) > 0 else 0
    prior_latency = prior_latency_values.mean() if len(prior_latency_values) > 0 else 0
    prior_std = prior_latency_values.std() if len(prior_latency_values) > 0 else 0

    # Append data for the current bag file
    all_data.append({
        'aggregation_info': "model:"+str(aggregation_model) +" alpha: "+ str(alpha)+" beta: "+str(beta) +" w1: "+str(w1) +" w2: " +str(w2)+ " w3: " +str(w3),
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

# Save all data to a CSV
output_csv_path = os.path.join(bagfile_path, "data.csv")
df = pd.DataFrame(all_data)
df.to_csv(output_csv_path, index=False)

print("DATA EXTRACTION PROCESS FINISHED")