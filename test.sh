#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Enables acoustics and tests vehicle sensors
# - Run this after running the 'start.sh' script

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

source ~/ros2_ws/install/setup.bash

# Turn on DVL acoustics
# Check if a parameter was provided
if [ -z "$1" ]; then
  echo "Error: You need to input a parameter ('true' or 'false') to enable or disable acoustics."
  exit 1
fi

# Check if the parameter is 'true' or 'false'
if [ "$1" == "true" ]; then
  echo "Turning on DVL acoustics..."
  # Add your command for enabling acoustics here
  bash ~/ros2_ws/dvl_tools/acoustics_on.sh true
elif [ "$1" == "false" ]; then
  echo "Turning off DVL acoustics..."
  # Add your command for disabling acoustics here
  bash ~/ros2_ws/dvl_tools/acoustics_on.sh false
else
  echo "Error: Invalid parameter. Please input either 'true' or 'false' to control acoustics."
  exit 1
fi

echo ""
echo "LISTING FOUND TOPICS..."
ros2 topic list

# echo ""
# echo "LISTENING TO TOPIC 'LEAK/DATA'..."
# ros2 topic echo --once /leak/data

# echo ""
# echo "LISTENING TO TOPIC 'BATTERY/DATA'..."
# ros2 topic echo --once /battery/data

echo ""
echo "LISTENING TO TOPIC 'PRESSURE/DATA'..."
ros2 topic echo --once /pressure/data

echo ""
echo "LISTENING TO TOPIC 'DEPTH_DATA'..."
ros2 topic echo --once /depth_data

echo ""
echo "LISTENING TO TOPIC 'MODEM_IMU'..."
ros2 topic echo --once /modem_imu

echo ""
echo "LISTENING TO TOPIC 'FIX'..."
ros2 topic echo --once /fix

echo ""
echo "LISTENING TO TOPIC 'GPS_ODOM'..."
ros2 topic echo --once /gps_odom

echo ""
echo "LISTENING TO TOPIC 'DVL/DATA'..."
ros2 topic echo --once /dvl/data

echo ""
echo "LISTENING TO TOPIC 'DVL/POSITION'..."
ros2 topic echo --once /dvl/position

echo ""
echo "TESTING TOP SERVO, PUBLISHING TO 'kinematics/COMMAND'..."
ros2 topic pub -1 /kinematics/command frost_interfaces/msg/UCommand '{fin: [45, 0, 0, 0], thruster: 0}'

echo ""
echo "TESTING SIDE SERVOS, PUBLISHING TO 'kinematics/COMMAND'..."
ros2 topic pub -1 /kinematics/command frost_interfaces/msg/UCommand '{fin: [0, 45, 45, 0], thruster: 0}'

echo ""
echo "TESTING THRUSTER (ON), PUBLISHING TO 'kinematics/COMMAND'..."
ros2 topic pub -1 /kinematics/command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 10}'

echo ""
echo "TESTING THRUSTER (OFF), PUBLISHING TO 'kinematics/COMMAND'..."
ros2 topic pub -1 /kinematics/command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 0}'

echo ""
echo "TEST COMPLETE"
