import rosbag
import csv
import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np
import argparse

# Parameters to set
# bagfile_name ="results_0.bag"
bagfile_path = "/mnt/storage_disk/extracted_results/response_threshold/bagfiles/"
extracted_data_path = "/mnt/storage_disk/extracted_results/response_threshold/bagfiles/extracted_data/"
topics_of_interest = [  "/mrs/communication_latency",
                        "/mrs/asv_travelled_distance",
                        "/mrs/data_transmited",
                        "/mrs/data_buffered"]
# Create the parser
parser = argparse.ArgumentParser(description='Description of your program.')
# Add an argument
parser.add_argument('bagfile_name', type=str, help='name of the bagfile')
# Parse the arguments
args = parser.parse_args()

# Open the bag file
bag = rosbag.Bag(bagfile_path+ str(args.bagfile_name))
# Get the start time of the bag file
start_time = bag.get_start_time()

# Extract data from the specified topics and write to the CSV file
latency_R1_values = np.array([])
latency_R2_values = np.array([])
latency_R3_values = np.array([])
latency_R4_values = np.array([])
latency_R5_values = np.array([])
latency_R6_values = np.array([])
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
        time_latency = np.append(time_latency,msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 -start_time)  
        

    if "/mrs/asv_travelled_distance" in topic:
        time_distance.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 - start_time)
        travelled_distance.append(msg.travelled_distance)

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


# SAVE THE DATA INTO A CSV
# Create the directory
data_folder = extracted_data_path+str(args.bagfile_name)
os.mkdir(data_folder)

# Crear un marco de datos con los datos
data = {'latency_R1': latency_R1_values,
        'latency_R2': latency_R2_values,
        'latency_R3': latency_R3_values,
        'latency_R4': latency_R4_values,
        'latency_R5': latency_R5_values,
        'latency_R6': latency_R6_values,
        'time_latency': time_latency,
}
# Path to the output CSV file
# output_csv_path = data_folder+str("/latency.csv")
# df = pd.DataFrame(data)
# df.to_csv(output_csv_path, mode='w', index=False, header=True)

# Crear un marco de datos con los datos
data = {'distance': travelled_distance,
        'time_distance': time_distance}
# Path to the output CSV file
# output_csv_path = data_folder+str("/distance.csv")
# df = pd.DataFrame(data)
# df.to_csv(output_csv_path, mode='w', index=False, header=True)

# Crear un marco de datos con los datos
data = {'transmitted_data_R1': transmitted_data_R1_values,
        'transmitted_data_R2': transmitted_data_R2_values,
        'transmitted_data_R3': transmitted_data_R3_values,
        'transmitted_data_R4': transmitted_data_R4_values,
        'transmitted_data_R5': transmitted_data_R5_values,
        'transmitted_data_R6': transmitted_data_R6_values,
        'transmitted_data_time': transmitted_data_time_values,
        'all_transmitted_data': all_transmitted_data
}
# Path to the output CSV file
# output_csv_path = data_folder+str("/transmitted_data.csv")
# df = pd.DataFrame(data)
# df.to_csv(output_csv_path, mode='w', index=False, header=True)

# Calculate the mean latency
mean_latency = (np.mean(latency_R1_values) + np.mean(latency_R2_values) + np.mean(latency_R3_values)+ np.mean(latency_R4_values)+np.mean(latency_R5_values)+np.mean(latency_R6_values)) / 6

# Calculate the standard deviation
std_latency = (np.std(latency_R1_values)+np.std(latency_R2_values)+np.std(latency_R3_values)+np.std(latency_R4_values)+np.std(latency_R5_values)+np.std(latency_R6_values))/6

print("MEAN LATENCY:"+str(mean_latency))
print("STD: "+str(std_latency))


# # Crear un marco de datos con los datos
# data = {'mean': np.mean(mean_latency),
#         'std': np.mean(std_latency)
# }
# # Path to the output CSV file
# output_csv_path = data_folder+str("/std_deviation.csv")
# df = pd.DataFrame(data)
# df.to_csv(output_csv_path, mode='w', index=False, header=True)


# ------------------------------- PLOT THE DATA --------------------------------
# Create a plot for other data (distance over time)
plt.plot(time_distance, travelled_distance, label='Distance')
plt.title('Distance over Time')
plt.xlabel('Time')
plt.ylabel('Distance')
plt.legend()
plt.savefig(data_folder+str("/travelled_distance_plot.png"))
plt.show()

# Create a plot for latency data
plt.plot(time_latency, latency_R1_values, label='latency_R1')
plt.plot(time_latency, latency_R2_values, label='latency_R2')
plt.plot(time_latency, latency_R3_values, label='latency_R3')
plt.plot(time_latency, latency_R4_values, label='latency_R4')
plt.plot(time_latency, latency_R5_values, label='latency_R5')
plt.plot(time_latency, latency_R6_values, label='latency_R6')
plt.title('Latency over Time')
plt.xlabel('Time')
plt.ylabel('Latency')
plt.legend()
plt.savefig(data_folder+str("/latency_plot.png"))
plt.show()


# Plot the mean latency with standard deviation
plt.plot(time_latency, mean_latency, label='Mean Latency')
plt.fill_between(time_latency, mean_latency-std_latency,mean_latency+std_latency, alpha=0.2)
plt.title('Mean Latency over Time with Standard Deviation')
plt.xlabel('Time')
plt.ylabel('Latency')
plt.legend()
plt.savefig(data_folder + str("/mean_latency_with_std.png"))
plt.show()

# DATA TRANSMITTED
plt.plot(transmitted_data_time_values, transmitted_data_R1_values, label='transmitted_data_R1')
plt.plot(transmitted_data_time_values, transmitted_data_R2_values,  label='transmitted_data_R2')
plt.plot(transmitted_data_time_values, transmitted_data_R3_values, label='transmitted_data_R3')
plt.plot(transmitted_data_time_values, transmitted_data_R4_values,  label='transmitted_data_R4')
plt.plot(transmitted_data_time_values, transmitted_data_R5_values,  label='transmitted_data_R5')
plt.plot(transmitted_data_time_values, transmitted_data_R6_values, label='transmitted_data_R6')
plt.title('Transmitted data over Time')
plt.xlabel('Time')
plt.ylabel('Transmitted data')
plt.legend()
# Save the plot to a desired path
save_path = data_folder+"/transmitted_data_plot.png"
plt.savefig(save_path)
plt.show()

# plt.plot(transmitted_data_time_values, all_transmitted_data, label='all_transmitted_data')
# plt.title('All transmitted data over Time')
# plt.xlabel('Time')
# plt.ylabel('Transmitted data')
# plt.legend()
# # Save the plot to a desired path
# save_path = data_folder+"/all_transmitted_data_plot.png"
# plt.savefig(save_path)
# plt.show()

# BUFFERED DATA
# plt.plot(buffered_data_time_values, buffered_data_R1_values, label='buffered_data_R1')
# plt.plot(buffered_data_time_values, buffered_data_R2_values, label='buffered_data_R2')
# plt.plot(buffered_data_time_values, buffered_data_R3_values, label='buffered_data_R3')
# plt.plot(buffered_data_time_values, buffered_data_R4_values, label='buffered_data_R4')
# plt.plot(buffered_data_time_values, buffered_data_R5_values, label='buffered_data_R5')
# plt.plot(buffered_data_time_values, buffered_data_R6_values, label='buffered_data_R6')
# plt.title('Buffered data over Time')
# plt.xlabel('Time')
# plt.ylabel('Buffered data')
# plt.legend()
# # Save the plot to a desired path
# save_path = data_folder+"/buffered_data_plot.png"
# plt.savefig(save_path)
# plt.show()

# plt.plot(buffered_data_time_values, all_buffered_data, label='buffered_data')
# plt.title('Buffered data over Time')
# plt.xlabel('Time')
# plt.ylabel('Buffered data')
# # plt.legend()
# # Save the plot to a desired path
# save_path = data_folder+"/all_buffered_data_plot.png"
# plt.savefig(save_path)
# plt.show()