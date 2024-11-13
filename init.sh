#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Publishes an empty message to the <namespace>/init topic to initialize the vehicle

source ~/config/bash_vars.sh

source ~/ros2_ws/install/setup.bash
ros2 topic pub $NAMESPACE/init std_msgs/msg/Empty -1



ros2 service call $NAMESPACE/holoocean_reset std_srvs/srv/Empty
