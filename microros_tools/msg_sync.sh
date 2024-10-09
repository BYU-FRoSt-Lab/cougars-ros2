#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Syncs frost_interfaces changes with the CougarsTeensy repo
# - After running this script, recompile micro-ROS in 
#   PlatformIO using 'msg_update.sh'

rsync -avc --delete ~/ros2_ws/src/frost_interfaces ~/teensy_ws/cougars/extra_packages
