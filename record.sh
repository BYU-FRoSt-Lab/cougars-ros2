#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Starts a ROS2 bag recording and initializes the vehicle
# - Specify a start configuration using 'bash record.sh 
#   <launch>' (ex. 'bash record.sh moos')
# - Run this after running the 'start.sh' and 'test.sh'
#   scripts
# - Log files are saved in '../bag' on the host machine
#   running the docker container

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

cleanup() {

    case $1 in
        "moos")
            bash ~/ros2_ws/moos_tools/mission_kill.sh
            ;;
    esac

    exit 0
}
trap cleanup SIGINT

echo ""
echo "Enter a descriptive folder name for the rosbag: "
read FOLDER
echo ""

source ~/ros2_ws/install/setup.bash

# Reset dead reckoning and start factor graph, controls
bash ~/ros2_ws/dvl_tools/reset_dr.sh
ros2 topic pub /init std_msgs/msg/Empty -1

# Start the MOOS-IvP mission
case $1 in
    "moos")
        bash ~/ros2_ws/moos_tools/mission_start_processes.sh
        bash ~/ros2_ws/moos_tools/mission_deploy.sh
        ;;
esac

FOLDER=$FOLDER-$(date +"%Y-%m-%d-%H-%M-%S")
ros2 bag record -o ~/bag/$FOLDER -s mcap -a

cleanup $1
