#!/bin/bash

##########################################################
# STARTS THE MICRO-ROS AGENT
##########################################################

if [ -z "$(tycmd list)" ]; then
    echo ""
    echo "ERROR: No Teensy boards avaliable to connect to"
    echo ""

else 
    cd ~/microros_ws
    source install/setup.bash
    # ros2 run micro_ros_agent micro_ros_agent multiserial --devs "/dev/ttyACM0 /dev/ttyACM1" -b 6000000
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &
fi
