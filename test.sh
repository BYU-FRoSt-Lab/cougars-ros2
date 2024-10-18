#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Tests vehicle sensors and actuators
# - Run this after running the 'launch.sh' script

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

function printSuccess {
  echo -e "\033[0m\033[32m[SUCCESS] $1\033[0m"
}

function printFailure {
  echo -e "\033[0m\033[31m[FAIL] $1\033[0m"
}

source ~/ros2_ws/install/setup.bash

# Turn on DVL acoustics
# Check if a parameter was provided
# if [ -z "$1" ]; then
#   echo "Error: You need to input a parameter ('true' or 'false') to enable or disable acoustics."
#   exit 1
# fi

# # Check if the parameter is 'true' or 'false'
# if [ "$1" == "true" ]; then
#   echo "Turning on DVL acoustics..."
#   # Add your command for enabling acoustics here
#   bash ~/ros2_ws/dvl_tools/acoustics_on.sh true
# elif [ "$1" == "false" ]; then
#   echo "Turning off DVL acoustics..."
#   # Add your command for disabling acoustics here
#   bash ~/ros2_ws/dvl_tools/acoustics_on.sh false
# else
#   echo "Error: Invalid parameter. Please input either 'true' or 'false' to control acoustics."
#   exit 1
# fi

echo ""

# TODO: Test this implementation with a string
leak_data=$(timeout 3 ros2 topic echo --once --no-arr $NAMESPACE/leak/data 2>/dev/null | grep -oP '(?<=leak: )\d+')
if [ -z "$leak_data" ]; then
  printFailure "No leak sensor connection found."
else
  if [[ $(echo "$leak_data" | awk '{if ($1 == false) print 1; else print 0}') -eq 0 ]]; then
    printSuccess "Leak sensor connected!"
    echo "  leak: $leak_data"
  else
    printFailure "Leak sensor may not be working."
    echo "  leak: $leak_data"
  fi
fi

battery_data=$(timeout 3 ros2 topic echo --once --no-arr $NAMESPACE/battery/data 2>/dev/null | grep -oP '(?<=voltage: )\d+(\.\d+)?')
if [ -z "$battery_data" ]; then
  printFailure "No battery monitor connection found."
else
  if [[ $(echo "$battery_data" | awk '{if ($1 == 0.0) print 1; else print 0}') -eq 0 ]]; then
    printSuccess "Battery monitor connected!"
    echo "  voltage: $battery_data"
  else
    printFailure "Battery monitor may not be working."
    echo "  voltage: $battery_data"
  fi
fi

pressure_data=$(timeout 3 ros2 topic echo --once --no-arr $NAMESPACE/pressure/data 2>/dev/null | grep -oP '(?<=fluid_pressure: )\d+(\.\d+)?')
if [ -z "$pressure_data" ]; then
  printFailure "No pressure sensor connection found."
else
  if [[ $(echo "$pressure_data" | awk '{if ($1 == 0.0) print 1; else print 0}') -eq 0 ]]; then
    printSuccess "Pressure sensor connected!"
    echo "  fluid_pressure: $pressure_data"
  else
    printFailure "Pressure sensor may not be working."
    echo "  fluid_pressure: $pressure_data"
  fi
fi

# TODO: Update this with the right values
modem_imu_data=$(timeout 3 ros2 topic echo --once --no-arr $NAMESPACE/modem_imu 2>/dev/null | grep -oP '(?<=orientation: \[)\d+')
if [ -z "$modem_imu_data" ]; then
  printFailure "No modem IMU connection found."
else
  if [[ $(echo "$modem_imu_data" | awk '{if ($1 == 0.0) print 1; else print 0}') -eq 0 ]]; then
    printSuccess "Modem IMU connected! (orientation: $modem_imu_data)"
  else
    printFailure "Modem IMU may not be working. (orientation: $modem_imu_data)"
  fi
fi

gps_data=$(timeout 3 ros2 topic echo --once --no-arr $NAMESPACE/fix 2>/dev/null | grep -oP '(?<=latitude: )\d+(\.\d+)?')
if [ -z "$gps_data" ]; then
  printFailure "No GPS connection found."
else
  if [[ $(echo "$gps_data" | awk '{if ($1 == 0.0) print 1; else print 0}') -eq 0 ]]; then
    printSuccess "GPS connected!"
    echo "  latitude: $gps_data"
  else
    printFailure "GPS may not be working."
    echo "  latitude: $gps_data"
  fi
fi

# TODO: Update this with the right values
dvl_data=$(timeout 3 ros2 topic echo --once --no-arr $NAMESPACE/dvl/data 2>/dev/null | grep -oP '(?<=velocity: \[)\d+')
if [ -z "$dvl_data" ]; then
  printFailure "No DVL connection (data) found."
else
  if [[ $(echo "$dvl_data" | awk '{if ($1 == 0.0) print 1; else print 0}') -eq 0 ]]; then
    printSuccess "DVL (data) connected! (velocity: $dvl_data)"
  else
    printFailure "DVL (data) may not be working. (velocity: $dvl_data)"
  fi
fi
  
dvl_position_data=$(timeout 3 ros2 topic echo --once --no-arr $NAMESPACE/dvl/position 2>/dev/null | grep -A 3 position: | grep -oP '(?<=z: )\d+(\.\d+)?')
if [ -z "$dvl_position_data" ]; then
  printFailure "No DVL connection (position) found."
else
  if [[ $(echo "$dvl_position_data" | awk '{if ($1 == 0.0) print 1; else print 0}') -eq 0 ]]; then
    printSuccess "DVL (position) connected!"
    echo "  x: $dvl_position_data"
  else
    printFailure "DVL (position) may not be working."
    echo "  x: $dvl_position_data"
  fi
fi

echo ""

printInfo "Testing top servo, publishing to 'kinematics/command'..."
timeout 5 ros2 topic pub -1 $NAMESPACE/kinematics/command frost_interfaces/msg/UCommand '{fin: [45, 0, 0, 0], thruster: 0}' 2>/dev/null

printInfo "Testing side servos, publishing to 'kinematics/command'..."
timeout 5 ros2 topic pub -1 $NAMESPACE/kinematics/command frost_interfaces/msg/UCommand '{fin: [0, 45, 45, 0], thruster: 0}' 2>/dev/null

printInfo "Testing thruster (ON), publishing to 'kinematics/command'..."
timeout 5 ros2 topic pub -1 $NAMESPACE/kinematics/command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 10}' 2>/dev/null

printInfo "Testing thruster (OFF), publishing to 'kinematics/command'..."
timeout 5 ros2 topic pub -1 $NAMESPACE/kinematics/command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 0}' 2>/dev/null

