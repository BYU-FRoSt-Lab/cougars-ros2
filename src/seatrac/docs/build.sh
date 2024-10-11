#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Builds documentation for the ROS2 seatrac package
# - The documentation can be accessed by opening the 'index.html'
#   file in the 'docs_output/seatrac' directory

source ~/ros2_ws/install/setup.bash
rosdoc2 build --package-path ~/ros2_ws/src/seatrac