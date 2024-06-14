#!/bin/bash

##########################################################
# SYNCS FROST_INTERFACES CHANGES WITH THE TEENSY_WS
# - After running this script, recompile micro-ROS in 
#   PlatformIO using "msg_update.sh"
##########################################################

rsync -avc --delete ~/CougarsRPi/src/frost_interfaces ~/CougarsTeensy/sensors/extra_packages
rsync -avc --delete ~/CougarsRPi/src/frost_interfaces ~/CougarsTeensy/control/extra_packages
