#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Starts the micro-ROS agent and ROS 2 launch files
# - Specify a start configuration using 'bash start.sh 
#   <launch>' (ex. 'bash start.sh moos')

source ~/config/bash_params.sh

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[73m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

cleanup() {
    bash ~/gpio/strobe.sh off
    bash ~/ros2_ws/dvl_tools/acoustics_on.sh false
    echo "[COMPLETE] Acoustics successfully disabled"
    exit 0
}
trap cleanup SIGINT

echo ""
echo "######################################################################"
echo "# BYU FROST LAB - CONFIGURABLE UNDERWATER GROUP OF AUTONOMOUS ROBOTS #"
echo "######################################################################"
echo ""

source ~/config/bash_vars.sh

# Quick fix for daemon error (TODO: find a better solution)
source ~/ros2_ws/install/setup.bash
ros2 daemon stop
ros2 daemon start
sleep 5

# Start the strobe light and Teensy board
bash ~/gpio/strobe.sh on
bash ~/gpio/power.sh on

# Test for Teensy board connection
if [ -z "$(tycmd list | grep Teensy)" ]; then
    printError "No Teensy boards avaliable to connect to"
    exit 1
fi

# Start both workspaces
source ~/microros_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
case $1 in
    "manual")
        ros2 launch cougars_control manual_launch.py namespace:=$NAMESPACE config_file:=$VEHICLE_PARAMS_FILE
        ;;
    "moos")
        ros2 launch cougars_control moos_launch.py namespace:=$NAMESPACE config_file:=$VEHICLE_PARAMS_FILE
        ;;
    "sensors")
        ros2 launch cougars_localization sensors_launch.py namespace:=$NAMESPACE config_file:=$VEHICLE_PARAMS_FILE
        ;;
    *)
        printWarning "No start configuration specified"
        printWarning "Specify a start configuration using 'bash start.sh <config>' (ex. 'bash start.sh moos')"
        ;;
esac

cleanup
