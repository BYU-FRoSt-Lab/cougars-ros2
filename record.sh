#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Starts a ROS2 bag recording with a custom name
# - Run this after running the 'launch.sh', 'test.sh', and 'init.sh' scripts
# - Log files are saved in 'CoUGARs/bag' on the host machine running the docker container

case $1 in
  "on")
    echo ""
    bash ~/ros2_ws/dvl_tools/acoustics_on.sh true
    ;;
  "off")
    bash ~/ros2_ws/dvl_tools/acoustics_on.sh false
    ;;
  *)
    printError "No state specified for DVL acoustics"
    printError "Specify a state using either 'bash test.sh on' or 'bash test.sh off'"
    exit 1
    ;;
esac

echo ""
echo "Enter a descriptive folder name for the rosbag: "
read FOLDER
echo ""

sleep 5
bash ~/ros2_ws/init.sh
bash ~/ros2_ws/dvl_tools/reset_dr.sh

FOLDER=$FOLDER-$(date +"%Y-%m-%d-%H-%M-%S")
ros2 bag record -o ~/bag/$FOLDER -a
