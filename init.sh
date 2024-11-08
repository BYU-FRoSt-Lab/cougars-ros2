#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Publishes an empty message to the /init topic to initialize the vehicle

source ~/config/bash_params.sh

source ~/ros2_ws/install/setup.bash
ros2 topic pub $NAMESPACE/init std_msgs/msg/Empty -1