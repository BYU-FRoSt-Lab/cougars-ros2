#!/bin/bash
#
# Starts the ROS 2 launch files
# - Specify a launch configuration using 'bash launch.sh <launch>' (ex. 'bash launch.sh moos')


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
case $1 in
    "full")
        ros2 launch cougars_bringup persistant_launch.py namespace:=$NAMESPACE param_file:=$VEHICLE_PARAMS_FILE fleet_param:=$FLEET_PARAMS_FILE sim:=$SIM_PARAM verbose:=$VERBOSE fins:=$FINS
        ;;
    "manual")
        ros2 launch cougars_control manual_launch.py namespace:=$NAMESPACE param_file:=$VEHICLE_PARAMS_FILE fleet_param:=$FLEET_PARAMS_FILE sim:=$SIM_PARAM verbose:=$VERBOSE fins:=$FINS
        ;;
    "moos")
        ros2 launch cougars_control moos_launch.py namespace:=$NAMESPACE param_file:=$VEHICLE_PARAMS_FILE fleet_param:=$FLEET_PARAMS_FILE sim:=$SIM_PARAM verbose:=$VERBOSE GPS:=$GPS
        ;;
    "sensors")
        ros2 launch cougars_localization sensors_launch.py namespace:=$NAMESPACE param_file:=$VEHICLE_PARAMS_FILE fleet_param:=$FLEET_PARAMS_FILE
        ;;
    "demo")
        ros2 launch cougars_localization demo_launch.py namespace:=$NAMESPACE param_file:=$VEHICLE_PARAMS_FILE fleet_param:=$FLEET_PARAMS_FILE
        ;;
    *)
        printError "No start configuration specified"
        printError "Specify a launch configuration using 'bash launch.sh <config>' (ex. 'bash launch.sh moos')"
        ;;
esac

cleanup
