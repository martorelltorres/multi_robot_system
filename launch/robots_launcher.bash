#!/bin/bash

# Function to launch the ROS processes
launch_ros_processes() {
    local show_terminal="$1"
    local robot_names=("robot0" "robot1" "robot2" "robot3" "robot4")
    local processes=()

    for i in "${!robot_names[@]}"; do
        local name="${robot_names[$i]}"
        if [ "$i" -eq "$((${#robot_names[@]} - 1))" ]; then
            # Use a different launch file for the last robot
            local package_name="cola2_xiroi"
            local launch_file="sim_start.launch"
        else
            local package_name="cola2_turbot"
            local launch_file="sim_start.launch"
        fi

        if [ "$show_terminal" = true ]; then
            # Command to show terminal
            gnome-terminal -- bash -c "roslaunch $package_name $launch_file robot_name:=$name; exec bash" &
            processes+=($!)
        else
            # Command to hide terminal
            roslaunch $package_name $launch_file robot_name:=$name >/dev/null 2>&1 &
            processes+=($!)
        fi

        sleep 3  # Wait n seconds before launching the next robot
        echo "$name: launched"
    done
}

# Trap Ctrl+C and clean up processes
trap_ctrlc() {
    echo -e "\nCtrl+C detected! Cleaning up processes..."
    for pid in "${processes[@]}"; do
        kill "$pid"
    done
    echo "All processes terminated. Exiting."
    exit 0
}

# Main logic
show_terminal=false  # Set to true to show terminals

# Trap the Ctrl+C signal
trap 'trap_ctrlc' SIGINT

# Launch the ROS processes
launch_ros_processes "$show_terminal"

# Keep the script running until Ctrl+C
while true; do
    sleep 1  # Keeps the script alive
done