#!/bin/bash

# Source your workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Create a dedicated folder for these data logs
mkdir -p topic_logs

echo "Intercepting ROS 2 topics and writing to files..."

# Echo each topic into its own text file in the background
ros2 topic echo /imu/data_raw > topic_logs/imu_data.txt &
IMU_PID=$!

ros2 topic echo /robot/state/pitch > topic_logs/pitch_data.txt &
PITCH_PID=$!

ros2 topic echo /motor/pwm > topic_logs/pwm_data.txt &
PWM_PID=$!

echo "Recording in progress."
echo "Let it run for a few seconds, then press [CTRL+C] to stop."

# Safely kill the background echoing when you cancel the script
trap "echo -e '\nStopping recording...'; kill $IMU_PID $PITCH_PID $PWM_PID; exit" INT
wait