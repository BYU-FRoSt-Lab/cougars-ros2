#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Starts a ROS2 bag recording with a custom name
# - Run this after running the 'launch.sh', 'test.sh', and 'init.sh' scripts
# - Log files are saved in 'CoUGARs/bag' on the host machine running the docker container

echo ""
echo "Enter a descriptive folder name for the rosbag: "
read FOLDER
echo ""

bash ~/ros2_ws/dvl_tools/acoustics_on.sh true
sleep 5
bash ~/ros2_ws/init.sh
bash ~/ros2_ws/dvl_tools/reset_dr.sh

FOLDER=$FOLDER-$(date +"%Y-%m-%d-%H-%M-%S")
ros2 bag record -o ~/bag/$FOLDER -a
