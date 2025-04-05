#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Starts the micro-ROS agent and ROS 2 launch files
# - Specify a launch configuration using 'bash launch.sh <launch>' (ex. 'bash launch.sh moos')

source ~/config/bash_vars.sh

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

cleanup() {
  echo ""
  
  if [ "$(uname -m)" == "aarch64" ]; then
    bash ~/gpio/strobe.sh off
    bash ~/ros2_ws/dvl_tools/acoustics_on.sh false
  fi
  
  exit 0
}
trap cleanup SIGINT

echo ""
echo -e "\033[0m\033[36m######################################################################\033[0m"
echo -e "\033[0m\033[36m#\033[0m BYU FROST LAB - CONFIGURABLE UNDERWATER GROUP OF AUTONOMOUS ROBOTS \033[0m\033[36m#\033[0m"
echo -e "\033[0m\033[36m######################################################################\033[0m"
echo ""

# Parse options
SIM_PARAM="false" # Default value for sim
VERBOSE="false"
GPS="false"
FINS="false"
while getopts "svgf" opt; do
  case $opt in
    s)
      SIM_PARAM="true"
      VEHICLE_PARAMS_FILE=/home/frostlab/config/sim_params.yaml
      echo "Using param file $VEHICLE_PARAMS_FILE"
      ;;
    v)
      VERBOSE="true"
      ;;
    g)
      GPS="true"
      ;;
    f)
      FINS="true"
      ;;
    *)
      printError "Invalid option"
      exit 1
      ;;
  esac
done
shift $((OPTIND - 1)) # Shift positional arguments


source ~/ros2_ws/install/setup.bash

echo ""

if [ "$(uname -m)" == "aarch64" ]; then
  # Quick fix for daemon error (TODO: find a better solution)
  ros2 daemon stop
  ros2 daemon start
  sleep 3
  # Start the strobe light and Teensy board
  bash ~/gpio/strobe.sh on
  bash ~/gpio/power.sh on

  # Test for Teensy board connection
  if [ -z "$(tycmd list | grep Teensy)" ]; then
      printError "No Teensy boards avaliable to connect to"
      exit 1
  fi

  echo ""
fi


#TODO demo option that launchs the teensy controller fins even with sim?
#TODO just make a parameter in yaml for moos GPS Only

# Start both workspaces
source ~/ros2_ws/install/setup.bash
case $1 in
    "manual")
        ros2 launch cougars_control manual_launch.py namespace:=$NAMESPACE param_file:=$VEHICLE_PARAMS_FILE sim:=$SIM_PARAM verbose:=$VERBOSE fins:=$FINS
        ;;
    "moos")
        ros2 launch cougars_control moos_launch.py namespace:=$NAMESPACE param_file:=$VEHICLE_PARAMS_FILE sim:=$SIM_PARAM verbose:=$VERBOSE GPS:=$GPS
        ;;
    "sensors")
        ros2 launch cougars_localization sensors_launch.py namespace:=$NAMESPACE param_file:=$VEHICLE_PARAMS_FILE
        ;;
    "demo")
        ros2 launch cougars_localization demo_launch.py namespace:=$NAMESPACE param_file:=$VEHICLE_PARAMS_FILE
        ;;
    *)
        printError "No start configuration specified"
        printError "Specify a launch configuration using 'bash launch.sh <config>' (ex. 'bash launch.sh moos')"
        ;;
esac

cleanup
