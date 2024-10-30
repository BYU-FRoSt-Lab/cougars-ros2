#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Starts the micro-ROS agent and ROS 2 launch files
# - Specify a start configuration using 'bash start.sh 
#   <launch>' (ex. 'bash start.sh moos')

source ~/config/constants.sh

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
    
    sudo bash /home/$LABNAME/gpio/strobe.sh off
    bash ~/ros2_ws/dvl_tools/acoustics_on.sh false
    printInfo "Acoustics successfully disabled"

    # killall micro_ros_agent
    wait
    exit 0
}
trap cleanup SIGINT

echo ""
echo "######################################################################"
echo "# BYU FROST LAB - CONFIGURABLE UNDERWATER GROUP OF AUTONOMOUS ROBOTS #"
echo "######################################################################"
echo ""

# Quick fix for daemon error (TODO: find a better solution)
source ~/ros2_ws/install/setup.bash
ros2 daemon stop
ros2 daemon start
sleep 5

# Start the strobe light and Teensy board
sudo bash /home/$LABNAME/gpio/strobe.sh on
sudo bash /home/$LABNAME/gpio/power.sh on

# Start the micro-ROS agent
if [ -z "$(tycmd list | grep Teensy)" ]; then
    printError "No Teensy boards avaliable to connect to"
    exit 1
# else 
#     source ~/microros_ws/install/setup.bash
#     ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &
#     sleep 5
fi

# Start both workspaces
source ~/microros_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
case $1 in
    "manual")
        ros2 launch cougars_control manual_launch.py namespace:=$NAMESPACE
        ;;
    "moos")
        ros2 launch cougars_control moos_launch.py namespace:=$NAMESPACE
        ;;
    "sensors")
        ros2 launch cougars_localization sensors_launch.py namespace:=$NAMESPACE
        ;;
    *)
        printWarning "No start configuration specified"
        printWarning "Specify a start configuration using 'bash start.sh <config>' (ex. 'bash start.sh moos')"
        ;;
esac

cleanup
