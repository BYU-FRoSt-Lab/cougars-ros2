#!/bin/bash
#
# Starts the ROS 2 launch files
# - Specify a launch configuration using 'bash launch.sh <launch>' (ex. 'bash launch.sh moos')
source ~/config/cougarsrc.sh

# cleanup() {
#   echo ""
  
#   if [ "$(uname -m)" == "aarch64" ]; then
#     bash ~/gpio/strobe.sh off
#     bash ~/ros2_ws/dvl_tools/acoustics_on.sh false
#   fi
  
#   exit 0
# }
# trap cleanup SIGINT

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
    "waypoint")
        # --- Start Waypoint Selection Logic ---
        # MISSIONS_DIR=$(ros2 pkg prefix cougars_control)/share/cougars_control/waypoint_missions

        # if [ ! -d "$MISSIONS_DIR" ]; then
        #     printError "Missions directory not found at $MISSIONS_DIR"
        #     exit 1
        # fi

        # MISSIONS=("$MISSIONS_DIR"/*.yaml)

        # if [ ${#MISSIONS[@]} -eq 1 ] && [ ! -e "${MISSIONS[0]}" ]; then
        #     printError "No mission files (.yaml) found in $MISSIONS_DIR"
        #     exit 1
        # fi

        # # Use 'basename' to show only filenames in the list
        # MISSIONS_BASENAMES=()
        # for mission in "${MISSIONS[@]}"; do
        #     MISSIONS_BASENAMES+=("$(basename "$mission")")
        # done

        # MISSIONS_BASENAMES+=("Cancel")

        # printInfo "Please select a mission to launch:"
        # PS3="Enter number: " # Set the prompt for the select command

        # select MISSION_NAME in "${MISSIONS_BASENAMES[@]}"; do
        #     if [ "$MISSION_NAME" == "Cancel" ]; then
        #         printInfo "Launch cancelled."
        #         exit 0
        #     elif [ -n "$MISSION_NAME" ]; then
        #         printInfo "You selected mission: $MISSION_NAME"
        #         break
        #     else
        #         printWarning "Invalid selection. Please try again."
        #     fi
        # done

        # if [ -z "$MISSION_NAME" ]; then
        #     printError "No mission selected."
        #     exit 1
        # fi
        # --- End Waypoint Selection Logic ---

        # Launch with the selected mission file
        ros2 launch cougars_control waypoint_launch.py namespace:=$NAMESPACE param_file:=$VEHICLE_PARAMS_FILE sim:=$SIM_PARAM verbose:=$VERBOSE mission_file:=/home/frostlab/config/test_mission.yaml GPS:=$GPS
        ;;
    *)
        printError "No start configuration specified"
        printError "Specify a launch configuration using 'bash launch.sh <config>' (ex. 'bash launch.sh moos')"
        ;;
esac

cleanup
