import rosbag
import csv
import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np
import argparse
import time

# Parameters to set
bagfile_path = "/home/uib/MRS_data/new_architecture/test_2/response_threshold/bagfiles"
# /mnt/storage_disk/ARTM/ctt
# extracted_data_path = "/home/uib/MRS_data/response_threshold/bagfiles"
topics_of_interest = [  "/mrs/allocator_communication_latency",
                        "/mrs/asv_travelled_distance",
                        "/mrs/allocator_data_transmited",
                        "/mrs/allocator_data_buffered",
                        '/mrs/asv0_priority_communication_latency',
                        '/mrs/asv0_regular_communication_latency'
                        ]

all_files = os.listdir(bagfile_path)
# Get a list of all bag files in the folder
bag_files = [os.path.join(bagfile_path, filename) for filename in all_files if filename.endswith('.bag')]
print ("I found "+str(len(bag_files))+" bagfiles!!")
# Extract data from each bag file one by one
for bag_file in range(len(bag_files)):
    # Open the bag file
    bag = rosbag.Bag(bagfile_path+"/results_"+str(bag_file)+".bag")
    # Get the start time of the bag file
    start_time = bag.get_start_time()

    # Extract data from the specified topics and write to the CSV file
    reg_latency_values = np.array([]) 
    prior_latency_values = np.array([]) 

    prior_objects = []
    reg_objects = []

    for topic, msg, t in bag.read_messages(topics=topics_of_interest):
            
        if "/mrs/asv0_regular_communication_latency" in topic:
            reg_latency = getattr(msg, 'comm_latency', (0, 0, 0, 0, 0, 0))
            sum_reg_latency = sum(reg_latency)
            reg_latency_values= np.append(reg_latency_values,sum_reg_latency)
        
        if "/mrs/asv0_priority_communication_latency" in topic:
            prior_latency = getattr(msg, 'comm_latency', (0, 0, 0, 0, 0, 0))
            sum_prior_latency = sum(prior_latency)
            prior_latency_values= np.append(prior_latency_values,sum_prior_latency)
                      
        if "/mrs/asv_travelled_distance" in topic:
            # time_distance.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 - start_time)
            travelled_distance = msg.travelled_distance

        if "/mrs/allocator_data_transmited" in topic:
            data_transmited = getattr(msg, 'transmitted_data', (0, 0, 0, 0, 0, 0))
            regular_objects =  getattr(msg, 'transmitted_regular_objects', (0, 0, 0, 0, 0, 0))
            priority_objects =  getattr(msg, 'transmitted_priority_objects', (0, 0, 0, 0, 0, 0))
            # data
            sum_data = sum(data_transmited)
            # regular_objects
            sum_reg_objects = sum(regular_objects)
            # priority_objects
            sum_prior_objects = sum(priority_objects)

    # Close the bag file after extraction
    bag.close()

    # handle REGULAR OBJECTS latency values
    reg_latency = sum(reg_latency_values)/len(reg_latency_values)
    print("    ")
    print("Extracting data from results_"+str(bag_file)+".bag")
    print("Regular objects latency mean: "+str(reg_latency))
    reg_std = np.std(reg_latency_values)
    print("Regular objects standar deviation: "+str(reg_std))

    # handle PRIORITY OBJECTS latency values
    prior_latency = sum(prior_latency_values)/len(prior_latency_values)
    print("Priority objects latency mean: "+str(prior_latency))
    prior_std = np.std(prior_latency_values)
    print("Prioriry objects standar deviation: "+str(prior_std))

    # data transmitted
    print("Total data transmitted: "+str(sum_data))
    print("Priority objects: "+str(sum_prior_objects))
    print("Regular objects: "+str(sum_reg_objects))
    print("Distance travelled by ASV: "+str(travelled_distance))
    print(" ___________________________________________________")


    # SAVE THE DATA INTO A CSV
    # Create the directory
    data_folder = bagfile_path
    # Path to the output CSV file
    output_csv_path = data_folder+str("data.csv")

    # Create a DataFrame with the new data
    data = {'regular latency': [reg_latency],
            'reg_std_latency': [reg_std],
            'priority latency': [prior_latency],
            'prior_std_latency': [prior_std],
            'transmitted_data': [sum_data],
            'priority_objects' : [sum_prior_objects],
            'regular_objects' : [sum_reg_objects],
            'travelled_distance':[travelled_distance]
    }

    df = pd.DataFrame(data)
    df.to_csv(output_csv_path, mode='a', index=False, header=False)
    
print("   ")
print("DATA EXTRACTION PROCESS FINISHED")







