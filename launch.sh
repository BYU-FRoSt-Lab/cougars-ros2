#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# MODIFIED: This script has been updated to pass arguments to ROS 2 launch files
# using the standard 'argument_name:=value' syntax for better compatibility.
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
    # bash ~/gpio/strobe.sh off
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

# Quick fix for daemon error (TODO: find a better solution)
source ~/ros2_ws/install/setup.bash
ros2 daemon stop
ros2 daemon start
sleep 3

echo ""

# Parse options
SIM_PARAM="false" # Default value for sim
VERBOSE="false"
GPS="false"
FINS="false"
# Set a default vehicle parameter file. It can be overridden by the -s flag.
VEHICLE_PARAMS_FILE="/home/frostlab/config/vehicle_params.yaml"

while getopts "svgfb" opt; do
  case $opt in
    s)
      SIM_PARAM="true"
      VEHICLE_PARAMS_FILE="/home/frostlab/config/sim_params.yaml"
      printInfo "Using SIM param file: $VEHICLE_PARAMS_FILE"
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

# Start both workspaces
source ~/microros_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash

# The following 'ros2 launch' commands now use the standard 'argument:=value' syntax.
# This is the correct way to pass parameters to a ROS 2 launch file.
case $1 in
    "manual")
        ros2 launch cougars_control manual_launch.py \
          namespace:="$NAMESPACE" \
          param_file:="$VEHICLE_PARAMS_FILE" \
          sim:="$SIM_PARAM" \
          verbose:="$VERBOSE" \
          fins:="$FINS"
        ;;
    "moos")
        ros2 launch cougars_control moos_launch.py \
          namespace:="$NAMESPACE" \
          param_file:="$VEHICLE_PARAMS_FILE" \
          sim:="$SIM_PARAM" \
          verbose:="$VERBOSE" \
          GPS:="$GPS"
        ;;
    "sensors")
        ros2 launch cougars_localization sensors_launch.py \
          namespace:="$NAMESPACE" \
          param_file:="$VEHICLE_PARAMS_FILE"
        ;;

    "demo")
        ros2 launch cougars_localization demo_launch.py \
          namespace:="$NAMESPACE" \
          param_file:="$VEHICLE_PARAMS_FILE"
        ;;

    "bluerov")
        ros2 launch cougars_control bluerov_launch.py \
          namespace:="$NAMESPACE" \
          param_file:="$VEHICLE_PARAMS_FILE" \
          BLUEROV:="true" \
          verbose:="$VERBOSE"
        ;;

    *)
        printError "No start configuration specified"
        printError "Specify a launch configuration using 'bash launch.sh <config>' (ex. 'bash launch.sh moos')"
        ;;
esac

cleanup