#!/bin/bash

##########################################################
# SYNCS FROST_INTERFACES CHANGES WITH THE TEENSY_WS
# - After running this script, recompile micro-ROS in 
#   PlatformIO using "msg_update.sh"
# - Don't run this on the Pi -- run it on your own 
#   machine and pull the changes to the Pi
##########################################################

rsync -avc --delete ~/CougarsRPi/src/frost_interfaces ~/CougarsTeensy/sensors/extra_packages
rsync -avc --delete ~/CougarsRPi/src/frost_interfaces ~/CougarsTeensy/control/extra_packages
