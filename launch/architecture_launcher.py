#!/usr/bin/env python3
import subprocess
import time
import signal
import sys

def launch_ros_processes(show_terminal=True):
    robot_names = ["robot0", "robot1", "robot2", "robot3", "robot4"]
    processes = []

    for i, name in enumerate(robot_names):
        if i == len(robot_names) - 1:
            # Use a different launch file for the last robot
            package_name = "cola2_xiroi"
            launch_file = "sim_start.launch"
        else:
            package_name = "cola2_sparus2"
            launch_file = "sim_start.launch"

        if show_terminal:
            # Command to show terminal
            command = f'gnome-terminal -- bash -c "roslaunch {package_name} {launch_file} robot_name:={name}; exec bash"'
        else:
            # Command to hide terminal
            command = ["roslaunch", package_name, launch_file, f"robot_name:={name}"]

        if show_terminal:
            process = subprocess.Popen(command, shell=True)
        else:
            process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        processes.append(process)
        time.sleep(5)  # Wait 5 seconds before launching the next robot
        print(f"{name}: launched")

    return processes

def signal_handler(sig, frame):
    print("\nCtrl+C detected! Shutting down...")
    sys.exit(0)

if __name__ == "__main__":
    # Set to False to hide terminals
    show_terminal = False

    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, signal_handler)

    # Launch processes
    processes = launch_ros_processes(show_terminal=show_terminal)

    # Keep the script running until Ctrl+C
    try:
        while True:
            time.sleep(1)  # Keeps the script alive
    except KeyboardInterrupt:
        print("\nCtrl+C detected! Cleaning up processes...")
        for process in processes:
            process.terminate()
        print("All processes terminated. Exiting.")