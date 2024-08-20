#!/bin/bash

# Check if a parameter is provided
if [ -z "$1" ]; then
    echo "Usage: $0 <true|false>"
    exit 1
fi

# Check if the parameter is either 'true' or 'false'
if [ "$1" != "true" ] && [ "$1" != "false" ]; then
    echo "Error: Parameter must be 'true' or 'false'"
    exit 1
fi

# Set the command variables
COMMAND="set_config"
PARAMETER_NAME="acoustic_enabled"
PARAMETER_VALUE="$1"

# Replace 'my_package' with the actual package name containing the ConfigCommand.msg
PACKAGE_NAME="dvl_msgs"

echo ${PARAMETER_VALUE}

# Publish the message to the ROS 2 topic

echo running:  ros2 topic pub /dvl/config/command ${PACKAGE_NAME}/ConfigCommand "{command: '${COMMAND}', parameter_name: '${PARAMETER_NAME}', parameter_value: '${PARAMETER_VALUE}'}" 

ros2 topic pub /dvl/config/command ${PACKAGE_NAME}/ConfigCommand "{command: '${COMMAND}', parameter_name: '${PARAMETER_NAME}', parameter_value: '${PARAMETER_VALUE}'}"
