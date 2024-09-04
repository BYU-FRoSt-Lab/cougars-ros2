#!/bin/bash

##########################################################
# TESTS EACH OF THE EXPECTED ROS TOPICS
##########################################################

cd ~/ros2_ws
source install/setup.bash

# Turn on DVL acoustics
bash ~/ros2_ws/dvl_tools/acoustics_on.sh

echo ""
echo "LISTING FOUND TOPICS..."
ros2 topic list

# echo ""
# echo "LISTENING TO TOPIC 'LEAK_DATA'..."
# ros2 topic echo --once /leak_data

# echo ""
# echo "LISTENING TO TOPIC 'BATTERY_DATA'..."
# ros2 topic echo --once /battery_data

echo ""
echo "LISTENING TO TOPIC 'PRESSURE_DATA'..."
ros2 topic echo --once /pressure_data

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
echo "TESTING TOP SERVO, PUBLISHING TO 'CONTROL_COMMAND'..."
ros2 topic pub -1 /control_command frost_interfaces/msg/UCommand '{fin: [45, 0, 0, 0], thruster: 0}'

echo ""
echo "TESTING SIDE SERVOS, PUBLISHING TO 'CONTROL_COMMAND'..."
ros2 topic pub -1 /control_command frost_interfaces/msg/UCommand '{fin: [0, 45, 45, 0], thruster: 0}'

echo ""
echo "TESTING THRUSTER (ON), PUBLISHING TO 'CONTROL_COMMAND'..."
ros2 topic pub -1 /control_command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 10}'

echo ""
echo "TESTING THRUSTER (OFF), PUBLISHING TO 'CONTROL_COMMAND'..."
ros2 topic pub -1 /control_command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 0}'

echo ""
echo "TEST COMPLETE"
