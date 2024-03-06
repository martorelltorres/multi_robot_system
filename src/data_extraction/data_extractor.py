import rosbag
import csv
import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np
import argparse
import time

# Parameters to set
bagfile_path = "/mnt/storage_disk/ARTM"
extracted_data_path = "/mnt/storage_disk/ARTM"
topics_of_interest = [  "/mrs/communication_latency",
                        "/mrs/asv_travelled_distance",
                        "/mrs/data_transmited",
                        "/mrs/data_buffered"]

all_files = os.listdir(bagfile_path)
# Get a list of all bag files in the folder
bag_files = [os.path.join(bagfile_path, filename) for filename in all_files if filename.endswith('.bag')]
print ("I found "+str(len(bag_files))+" bagfiles!!")
# Extract data from each bag file one by one
for bag_file in range(len(bag_files)):
    print("Extracting data from results_"+str(bag_file)+".bag")
    # Open the bag file
    bag = rosbag.Bag(bagfile_path+"/results_"+str(bag_file)+".bag")
    # Get the start time of the bag file
    start_time = bag.get_start_time()

    # Extract data from the specified topics and write to the CSV file
    latency_R1_values = np.array([])
    latency_R2_values = np.array([])
    latency_R3_values = np.array([])
    latency_R4_values = np.array([])
    latency_R5_values = np.array([])
    latency_R6_values = np.array([])
    latency_mean = np.array([])
    time_latency = np.array([])

    travelled_distance = []
    time_distance = []

    transmitted_data_R1_values =[]
    transmitted_data_R2_values =[]
    transmitted_data_R3_values =[]
    transmitted_data_R4_values =[]
    transmitted_data_R5_values =[]
    transmitted_data_R6_values =[]
    transmitted_data_time_values =[]
    all_transmitted_data=[]

    buffered_data_R1_values =[]
    buffered_data_R2_values =[]
    buffered_data_R3_values =[]
    buffered_data_R4_values =[]
    buffered_data_R5_values =[]
    buffered_data_R6_values =[]
    buffered_data_time_values =[]
    all_buffered_data=[]

    for topic, msg, t in bag.read_messages(topics=topics_of_interest):
        if "/mrs/communication_latency" in topic:
            latency_R1, latency_R2, latency_R3,latency_R4, latency_R5, latency_R6 = getattr(msg, 'comm_delay', (0, 0, 0, 0, 0, 0))
            latency_R1_values = np.append(latency_R1_values,latency_R1)
            latency_R2_values= np.append(latency_R2_values,latency_R2)
            latency_R3_values= np.append(latency_R3_values,latency_R3)
            latency_R4_values= np.append(latency_R4_values,latency_R4)
            latency_R5_values= np.append(latency_R5_values,latency_R5)
            latency_R6_values= np.append(latency_R6_values,latency_R6)
            latency_mean = np.append(latency_mean,((latency_R1+latency_R2+latency_R3+latency_R4+latency_R5+latency_R6)/6))
            time_latency = np.append(time_latency,msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 -start_time)  
            

        if "/mrs/asv_travelled_distance" in topic:
            time_distance.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 - start_time)
            travelled_distance = msg.travelled_distance

        if "/mrs/data_transmited" in topic:
            data_R1, data_R2, data_R3,data_R4, data_R5, data_R6 = getattr(msg, 'transmitted_data', (0, 0, 0, 0, 0, 0))
            transmitted_data_R1_values.append(data_R1)
            transmitted_data_R2_values.append(data_R2)
            transmitted_data_R3_values.append(data_R3)
            transmitted_data_R4_values.append(data_R4)
            transmitted_data_R5_values.append(data_R5)
            transmitted_data_R6_values.append(data_R6)
            transmitted_data_time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 - start_time
            transmitted_data_time_values.append(transmitted_data_time)
            all_transmitted_data.append(data_R1+data_R2+data_R3+data_R4+data_R5+data_R6)

        if "/mrs/data_buffered " in topic:
            buffer_R1, buffer_R2, buffer_R3,buffer_R4, buffer_R5, buffer_R6 = getattr(msg, 'storage', (0, 0, 0, 0, 0, 0))
            buffered_data_R1_values.append(buffer_R1)
            buffered_data_R2_values.append(buffer_R2)
            buffered_data_R3_values.append(buffer_R3)
            buffered_data_R4_values.append(buffer_R4)
            buffered_data_R5_values.append(buffer_R5)
            buffered_data_R6_values.append(buffer_R6)
            all_buffered_data.append(buffer_R1+buffer_R2+buffer_R3+buffer_R4+buffer_R5+buffer_R6)
            buffered_data_time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 - start_time
            buffered_data_time_values.append(buffered_data_time)
            

    # Close the bag file after extraction
    bag.close()


    # Calculate the mean latency
    mean_latency = np.mean(latency_mean)
    # Calculate the standard deviation
    std_latency = np.std(latency_mean)

    print("MEAN LATENCY:"+str(mean_latency))
    print("STD: "+str(std_latency))
    transmitted_data = all_transmitted_data[-1]/70
    print("data transmitted: "+ str(transmitted_data))
    print("distance: "+str(travelled_distance))

    # SAVE THE DATA INTO A CSV
    # Create the directory
    data_folder = bagfile_path
    # Path to the output CSV file
    output_csv_path = data_folder+str("data.csv")

    # Create a DataFrame with the new data
    data = {'mean latency': [mean_latency],
            'std_latency': [std_latency],
            'transmitted_data': [transmitted_data],
            'travelled_distance':[travelled_distance]
    }

    df = pd.DataFrame(data)
    df.to_csv(output_csv_path, mode='a', index=False, header=False)
    
    # Wait for five minutes before processing the next bag file
    print("   ")
print("DATA EXTRACTION PROCESS FINISHED")







