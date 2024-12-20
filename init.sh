#!/bin/bash
# Created by Nelson Durrant, Sep 2024
# Modified by Braden Meyers, Dec 2024
# Initializes the vehicle nodes
# - This bash script is called by the 'record.sh' script already

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

source ~/config/bash_vars.sh

source ~/ros2_ws/install/setup.bash
# ros2 topic pub $NAMESPACE/init std_msgs/msg/Empty -1

# Function to handle service call responses
function handleServiceCall {
  local service_output
  service_output=$(timeout 5s ros2 service call "$1" "$2" "$3" 2>&1)

  if echo "$service_output" | grep -q "success=True"; then
    local message
    message=$(echo "$service_output" | grep "message=" | sed -e 's/^.*message= //')
    printInfo "Service call to $1 succeeded. $message"
  elif echo "$service_output" | grep -q "success=False"; then
    local message
    message=$(echo "$service_output" | grep "message=" | sed -e 's/^.*message= //')
    printWarning "Service call to $1 failed with message: $message"
  else
    printError "Unexpected response from service $1: $service_output"
  fi
}

# Call services and handle responses
handleServiceCall "$NAMESPACE/arm_thruster" "std_srvs/srv/SetBool" "data: true"
handleServiceCall "$NAMESPACE/init_factor_graph" "std_srvs/srv/SetBool" "data: true"
handleServiceCall "$NAMESPACE/init_controls" "std_srvs/srv/SetBool" "data: true"
handleServiceCall "$NAMESPACE/init_manual" "std_srvs/srv/SetBool" "data: true"


# TODO: Add mission deploy for moos

# TODO: Stop the script if a service does not return a proper response
# TODO: Handle stop command and a restart
