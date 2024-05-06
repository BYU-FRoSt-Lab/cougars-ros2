#!/bin/bash

##########################################################
# STARTS THE MICRO-ROS AGENT
##########################################################

cd ~/microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent multiserial --devs "/dev/ttyACM0 /dev/ttyACM1" -b 6000000