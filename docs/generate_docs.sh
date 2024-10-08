#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Builds the ROS 2 documentation for the project

cd ~/ros2_ws
source install/setup.bash

cd ~/ros2_ws/docs
rosdoc2 build --package-path ../src/cougars_localization
rosdoc2 build --package-path ../src/cougars_control