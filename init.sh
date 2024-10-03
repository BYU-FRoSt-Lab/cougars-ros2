#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Publishes an empty message to the /init topic to initialize the vehicle

cd ~/ros2_ws
source install/setup.bash
ros2 topic pub /init std_msgs/msg/Empty -1