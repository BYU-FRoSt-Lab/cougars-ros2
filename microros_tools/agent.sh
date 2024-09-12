#!/bin/bash

##########################################################
# STARTS THE MICRO-ROS AGENT
##########################################################

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

if [ -z "$(tycmd list | grep Teensy)" ]; then
    echo ""
    printError "No Teensy boards avaliable to connect to"
    echo ""

else 
    cd ~/microros_ws
    source install/setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &
fi
