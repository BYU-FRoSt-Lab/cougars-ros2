#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Starts a ROS2 bag recording with a custom name
# - Specify a DVL acoustics power state using 'bash record.sh <state>' (ex. 'bash record.sh off')
# - Run this after running the 'launch.sh', 'test.sh' scripts
# - Log files are saved in 'CoUGARs/bag' on the host machine running the docker container

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

case $1 in
  "on")
    bash ~/ros2_ws/dvl_tools/acoustics_on.sh true
    ;;
  "off")
    bash ~/ros2_ws/dvl_tools/acoustics_on.sh false
    ;;
  *)
    printError "No state specified for DVL acoustics"
    printError "Specify a state using either 'bash record.sh on' or 'bash record.sh off'"
    exit 1
    ;;
esac

echo ""
echo "IMPORTANT! Name the rosbag with the testing location combined with the test number (ex. 'utah_lake1.0')"
echo "-> If the mission fails, keep the same name and increment the second number for reruns (ex. 'utah_lake1.1')"
echo "-> If the mission is successful, increment the first number to indicate a new mission (ex. 'utah_lake2.0')"
echo ""
echo "Enter a folder name for the rosbag: "
read folder
echo ""

sleep 5
bash ~/ros2_ws/dvl_tools/reset_dr.sh
# The dead reckoning should be reset before we call init on the factor graph
bash ~/ros2_ws/init.sh

folder=$folder-$(date +"%Y-%m-%d-%H-%M-%S")
ros2 bag record -o ~/bag/$folder -a

#Copy moos behavior file
cp ~/ros2_ws/moos_tools/coug.bhv ~/bag/$folder/
cp ~/config/vehicle_params.yaml ~/bag/$folder/
