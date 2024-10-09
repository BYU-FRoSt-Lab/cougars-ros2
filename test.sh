#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Enables acoustics and tests vehicle sensors
# - Run this after running the 'start.sh' script

source ~/ros2_ws/install/setup.bash

# Turn on DVL acoustics
bash ~/ros2_ws/dvl_tools/acoustics_on.sh true

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
echo "TESTING TOP SERVO, PUBLISHING TO 'CONTROLS/COMMAND'..."
ros2 topic pub -1 /controls/command frost_interfaces/msg/UCommand '{fin: [45, 0, 0, 0], thruster: 0}'

echo ""
echo "TESTING SIDE SERVOS, PUBLISHING TO 'CONTROLS/COMMAND'..."
ros2 topic pub -1 /controls/command frost_interfaces/msg/UCommand '{fin: [0, 45, 45, 0], thruster: 0}'

echo ""
echo "TESTING THRUSTER (ON), PUBLISHING TO 'CONTROLS/COMMAND'..."
ros2 topic pub -1 /controls/command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 10}'

echo ""
echo "TESTING THRUSTER (OFF), PUBLISHING TO 'CONTROLS/COMMAND'..."
ros2 topic pub -1 /controls/command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 0}'

echo ""
echo "TEST COMPLETE"
