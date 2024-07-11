import rosbag
import csv
import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np
import argparse
import time

# Parameters to set
bagfile_path = "/home/uib/MRS_data/threshold_analysis/RSSI"
# /mnt/storage_disk/ARTM/ctt
# extracted_data_path = "/home/uib/MRS_data/response_threshold/bagfiles"
topics_of_interest = [  "/mrs/allocator_communication_latency",
                        "/mrs/asv_travelled_distance",
                        "/mrs/allocator_data_transmited",
                        "/mrs/allocator_data_buffered",
                        ]
latency_1_mean = 0
latency_2_mean = 0
latency_3_mean = 0
latency_4_mean = 0
latency_5_mean = 0
latency_6_mean = 0

std_latency_1 =0
std_latency_2 =0
std_latency_3 =0
std_latency_4 =0
std_latency_5 =0
std_latency_6 =0

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
        if "/mrs/allocator_communication_latency" in topic:
            latency_R1, latency_R2, latency_R3,latency_R4, latency_R5, latency_R6 = getattr(msg, 'comm_latency', (0, 0, 0, 0, 0, 0))
            if(latency_R1!=0):
                latency_R1_values = np.append(latency_R1_values,latency_R1)
            if(latency_R2!=0):
                latency_R2_values= np.append(latency_R2_values,latency_R2)
            if(latency_R3!=0):
                latency_R3_values= np.append(latency_R3_values,latency_R3)
            if(latency_R4!=0):
                latency_R4_values= np.append(latency_R4_values,latency_R4)
            if(latency_R5!=0):
                latency_R5_values= np.append(latency_R5_values,latency_R5)
            if(latency_R6!=0):
                latency_R6_values= np.append(latency_R6_values,latency_R6)
            

            # latency_mean = np.append(latency_mean,((latency_R1+latency_R2+latency_R3+latency_R4+latency_R5+latency_R6)/6))
            # time_latency = np.append(time_latency,msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 -start_time)  
            

        if "/mrs/asv_travelled_distance" in topic:
            time_distance.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 - start_time)
            travelled_distance = msg.travelled_distance

        if "/mrs/allocator_data_transmited" in topic:
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

        # if "/mrs/allocator_data_buffered" in topic:
        #     buffer_R1, buffer_R2, buffer_R3,buffer_R4, buffer_R5, buffer_R6 = getattr(msg, 'storage', (0, 0, 0, 0, 0, 0))
        #     buffered_data_R1_values.append(buffer_R1)
        #     buffered_data_R2_values.append(buffer_R2)
        #     buffered_data_R3_values.append(buffer_R3)
        #     buffered_data_R4_values.append(buffer_R4)
        #     buffered_data_R5_values.append(buffer_R5)
        #     buffered_data_R6_values.append(buffer_R6)
        #     all_buffered_data.append(buffer_R1+buffer_R2+buffer_R3+buffer_R4+buffer_R5+buffer_R6)
        #     buffered_data_time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 - start_time
        #     buffered_data_time_values.append(buffered_data_time)
            

    # Close the bag file after extraction
    bag.close()

    # handle latency values
    if(len(latency_R1_values)!=0):
        latency_1_mean = sum(latency_R1_values)/len(latency_R1_values)
    if(len(latency_R2_values)!=0):    
        latency_2_mean = sum(latency_R2_values)/len(latency_R2_values)
    if(len(latency_R3_values)!=0):    
        latency_3_mean = sum(latency_R3_values)/len(latency_R3_values)
    if(len(latency_R4_values)!=0):    
        latency_4_mean = sum(latency_R4_values)/len(latency_R4_values)
    if(len(latency_R5_values)!=0):    
        latency_5_mean = sum(latency_R5_values)/len(latency_R5_values)
    if(len(latency_R6_values)!=0):    
        latency_6_mean = sum(latency_R6_values)/len(latency_R6_values)


    # Calculate the mean latency
    mean_latency = (latency_1_mean + latency_2_mean + latency_3_mean + latency_4_mean + latency_5_mean + latency_6_mean)/ 6

    # Calculate the standard deviation
    if(len(latency_R1_values)!=0):
        std_latency_1 = np.std(latency_R1_values)
    if(len(latency_R2_values)!=0):    
        std_latency_2 = np.std(latency_R2_values)
    if(len(latency_R3_values)!=0):    
        std_latency_3 = np.std(latency_R3_values)
    if(len(latency_R4_values)!=0):    
        std_latency_4 = np.std(latency_R4_values)
    if(len(latency_R5_values)!=0):    
        std_latency_5 = np.std(latency_R5_values)
    if(len(latency_R6_values)!=0):    
        std_latency_6 = np.std(latency_R6_values) 
    
    
    std_latency = (std_latency_1+std_latency_2+std_latency_3+std_latency_4+std_latency_5+std_latency_6)/6

    print("MEAN LATENCY:"+str(mean_latency))
    print("STD: "+str(std_latency))
    transmitted_data = all_transmitted_data[-1]/30
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
    
    print("   ")
print("DATA EXTRACTION PROCESS FINISHED")







