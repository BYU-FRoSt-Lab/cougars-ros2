#!/bin/bash

##########################################################
# RECORDS A ROS2 BAG FILE
# - Run this after running the 'bash start.sh' script
##########################################################

cd ~/ros2_ws
source install/setup.bash

echo ""
echo "Enter a descriptive rosbag folder name: "
read FOLDER
echo ""

FOLDER=$FOLDER-$(date +"%Y-%m-%d-%H-%M-%S")
ros2 bag record -o /home/frostlab/ros2_ws/bag/$FOLDER -s mcap -a
