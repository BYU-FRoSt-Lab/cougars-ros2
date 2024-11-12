#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Starts a ROS2 bag recording with a custom name
# - Run this after running the 'launch.sh', 'test.sh', and 'init.sh' scripts
# - Log files are saved in '../bag' on the host machine running the docker container

echo ""
echo "Enter a descriptive folder name for the rosbag: "
read FOLDER
echo ""

FOLDER=$FOLDER-$(date +"%Y-%m-%d-%H-%M-%S")
ros2 bag record -o ~/bag/$FOLDER -s mcap -a

cleanup $1
