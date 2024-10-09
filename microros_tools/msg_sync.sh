#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Syncs frost_interfaces changes with the CougarsTeensy repo
# - After running this script, recompile micro-ROS in 
#   PlatformIO using 'msg_update.sh'
# - Don't run this in the Docker image -- run it on your 
#   own machine or locally on the the Pi

rsync -avc --delete ~/CougarsRPi/src/frost_interfaces ~/CougarsTeensy/cougars/extra_packages
