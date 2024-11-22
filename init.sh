#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Initializes the vehicle nodes
# - This bash script is called by the 'record.sh' script already

# TODO: Make the init message a service instead? Doesn't seem to be incredibly reliable.

source ~/config/bash_vars.sh

source ~/ros2_ws/install/setup.bash
ros2 topic pub $NAMESPACE/init std_msgs/msg/Empty -1

ros2 service call $NAMESPACE/arm_thruster std_srvs/srv/SetBool data:\ true\ 
