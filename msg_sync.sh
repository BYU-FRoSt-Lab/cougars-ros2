#!/bin/bash

##########################################################
# SYNCS FROST_INTERFACES CHANGES WITH THE TEENSY_WS
# - After running this script, recompile micro-ROS in 
#   PlatformIO using "msg_update.sh"
##########################################################

rsync -avc --delete ~/ros2_ws/src/frost_interfaces ~/teensy_ws/sensors/extra_packages
rsync -avc --delete ~/ros2_ws/src/frost_interfaces ~/teensy_ws/control/extra_packages
