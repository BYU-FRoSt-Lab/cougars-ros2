#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Tests each of the expected micro-ROS topics
# - Use this after setting up a new PCB to test the agent
#   and Teensy board connections

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

    killall micro_ros_agent
    wait
    
    exit 0
}
trap cleanup SIGINT

sudo bash ~/teensy_ws/power.sh on

if [ -z "$(tycmd list | grep Teensy)" ]; then
    echo ""
    printError "No Teensy boards avaliable to connect to"
    echo ""

    exit 1

else 
    source ~/microros_ws/install/setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &
fi

source ~/ros2_ws/install/setup.bash

echo ""
echo "LISTING FOUND TOPICS..."
ros2 topic list

echo ""
echo "LISTENING TO TOPIC 'PRESSURE/DATA'..."
ros2 topic echo --once /pressure/data

# echo ""
# echo "LISTENING TO TOPIC 'LEAK/DATA'..."
# ros2 topic echo --once /leak/data

# echo ""
# echo "LISTENING TO TOPIC 'BATTERY/DATA'..."
# ros2 topic echo --once /battery/data

echo ""
echo "TESTING TOP SERVO, PUBLISHING TO 'KINEMATICS/COMMAND'..."
ros2 topic pub -1 /kinematics/command frost_interfaces/msg/UCommand '{fin: [45, 0, 0, 0], thruster: 0}'

echo ""
echo "TESTING SIDE SERVOS, PUBLISHING TO 'KINEMATICS/COMMAND'..."
ros2 topic pub -1 /kinematics/command frost_interfaces/msg/UCommand '{fin: [0, 45, 45, 0], thruster: 0}'

echo ""
echo "TESTING THRUSTER (ON), PUBLISHING TO 'KINEMATICS/COMMAND'..."
ros2 topic pub -1 /kinematics/command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 10}'

echo ""
echo "TESTING THRUSTER (OFF), PUBLISHING TO 'KINEMATICS/COMMAND'..."
ros2 topic pub -1 /kinematics/command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 0}'

echo ""
echo "TEST COMPLETE"

cleanup
