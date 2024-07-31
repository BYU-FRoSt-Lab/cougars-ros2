#!/bin/bash

##########################################################
# TESTS EACH OF THE EXPECTED ROS TOPICS
# - Before running this script, run "agent.sh" in a 
#   different terminal
##########################################################

cd ~/ros2_ws
source install/setup.bash

echo ""
echo "LISTING FOUND TOPICS..."
ros2 topic list

echo ""
echo "PUBLISHING TO TOPIC 'DESIRED_DEPTH'..."
ros2 topic pub -1 /pid_request frost_interfaces/msg/DesiredDepth '{desired_depth: 0}'

echo ""
echo "PUBLISHING TO TOPIC 'DESIRED_HEADING'..."
ros2 topic pub -1 /pid_request frost_interfaces/msg/DesiredHeading '{desired_heading: 0}'

echo ""
echo "PUBLISHING TO TOPIC 'DESIRED_SPEED'..."
ros2 topic pub -1 /pid_request frost_interfaces/msg/DesiredSpeed '{desired_speed: 0}'

echo ""
echo "LISTENING TO TOPIC 'DVL_DATA'..."
ros2 topic echo --once /dvl_data

echo ""
echo "LISTENING TO TOPIC 'DEPTH_DATA'..."
ros2 topic echo --once /depth_data

echo ""
echo "LISTENING TO TOPIC 'LEAK_DATA'..."
ros2 topic echo --once /leak_data

echo ""
echo "LISTENING TO TOPIC 'BATTERY_DATA'..."
ros2 topic echo --once /battery_data

echo ""
echo "LISTENING TO TOPIC 'GPS_DATA'..."
ros2 topic echo --once /gps_data

# echo ""
# echo "LISTENING TO TOPIC 'ECHO_DATA'..."
# ros2 topic echo --once /echo_data

echo ""
echo "TEST COMPLETE"