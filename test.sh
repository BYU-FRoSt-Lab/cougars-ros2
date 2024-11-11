#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Enables acoustics and tests vehicle sensors
# - Run this after running the 'start.sh' script

source ~/config/bash_vars.sh

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARN] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

function printSuccess {
  echo -e "\033[0m\033[32m[SUCCESS] $1\033[0m"
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
printInfo "Testing vehicle sensors..."

# get leak as a bool from the output of the topic echo
leak_data=$(timeout 3 ros2 topic echo --once --no-arr $NAMESPACE/leak/data 2>/dev/null | grep -oP '(?<=leak: )\d+')
if [ -z "$leak_data" ]; then
  printError "No leak sensor connection found."
else
  if [ $leak_data -eq "false" ]; then
    printSuccess "Leak sensor connected! (leak: $leak_data)"
  else
    printWarning "Leak sensor may not be working. (leak: $leak_data)"
  fi
fi

battery_data=$(timeout 3 ros2 topic echo --once --no-arr $NAMESPACE/battery/data 2>/dev/null | grep -oP '(?<=voltage: )\d+')
if [ -z "$battery_data" ]; then
  printError "No battery monitor connection found."
else
  if [ $battery_data -eq 0 ]; then
    printSuccess "Battery monitor connected! (voltage: $battery_data)"
  else
    printWarning "Battery monitor may not be working. (voltage: $battery_data)"
  fi
fi

pressure_data=$(timeout 3 ros2 topic echo --once --no-arr $NAMESPACE/pressure/data 2>/dev/null | grep -oP '(?<=fluid_pressure: )\d+(\.\d+)?')
if [ -z "$pressure_data" ]; then
  printError "No pressure sensor connection found."
else
  if [ $(bc <<< "$pressure_data != 0.0") ]; then
    printSuccess "Pressure sensor connected! (fluid_pressure: $pressure_data)"
  else
    printWarning "Pressure sensor may not be working. (fluid_pressure: $pressure_data)"
  fi
fi

depth_data=$(timeout 3 ros2 topic echo --once --no-arr $NAMESPACE/depth_data 2>/dev/null | grep -oP '(?<=z: )\d+(\.\d+)?')
if [ -z "$depth_data" ]; then
  printError "No depth sensor connection found."
else
  if [ $(bc <<< "$pressure_data != 0.0") ]; then
    printSuccess "Depth sensor connected! (z: $depth_data)"
  else
    printWarning "Depth sensor may not be working. (z: $depth_data)"
  fi
fi

# modem_imu_data=$(timeout 3 ros2 topic echo --no-arr $NAMESPACE/modem_imu 2>/dev/null | grep -oP '(?<=orientation: \[)\d+')
# if [ -z "$modem_imu_data" ]; then
#   printError "No modem IMU connection found."
# else
#   if [ $modem_imu_data -eq 0 ]; then
#     printSuccess "Modem IMU connected! (orientation: $modem_imu_data)"
#   else
#     printWarning "Modem IMU may not be working. (orientation: $modem_imu_data)"
#   fi
# fi

# fix_data=$(timeout 3 ros2 topic echo --no-arr $NAMESPACE/fix 2>/dev/null | grep -oP '(?<=latitude: )\d+')
# if [ -z "$fix_data" ]; then
#   printError "No GPS fix connection found."
# else
#   if [ $fix_data -eq 0 ]; then
#     printSuccess "GPS fix connected! (latitude: $fix_data)"
#   else
#     printWarning "GPS fix may not be working. (latitude: $fix_data)"
#   fi
# fi

# gps_odom_data=$(timeout 3 ros2 topic echo --no-arr $NAMESPACE/gps_odom 2>/dev/null | grep -oP '(?<=latitude: )\d+')
# if [ -z "$gps_odom_data" ]; then
#   printError "No GPS odom connection found."
# else
#   if [ $gps_odom_data -eq 0 ]; then
#     printSuccess "GPS odom connected! (latitude: $gps_odom_data)"
#   else
#     printWarning "GPS odom may not be working. (latitude: $gps_odom_data)"
#   fi
# fi

# dvl_data=$(timeout 3 ros2 topic echo --no-arr $NAMESPACE/dvl/data 2>/dev/null | grep -oP '(?<=velocity: \[)\d+')
# if [ -z "$dvl_data" ]; then
#   printError "No DVL data connection found."
# else
#   if [ $dvl_data -eq 0 ]; then
#     printSuccess "DVL data connected! (velocity: $dvl_data)"
#   else
#     printWarning "DVL data may not be working. (velocity: $dvl_data)"
#   fi
# fi

# dvl_position_data=$(timeout 3 ros2 topic echo --no-arr $NAMESPACE/dvl/position 2>/dev/null | grep -oP '(?<=position: \[)\d+')
# if [ -z "$dvl_position_data" ]; then
#   printError "No DVL position connection found."
# else
#   if [ $dvl_position_data -eq 0 ]; then
#     printSuccess "DVL position connected! (position: $dvl_position_data)"
#   else
#     printWarning "DVL position may not be working. (position: $dvl_position_data)"
#   fi
# fi

echo ""

printInfo "Testing top servo, publishing to 'kinematics/command'..."
timeout 3 ros2 topic pub -1 $NAMESPACE/kinematics/command frost_interfaces/msg/UCommand '{fin: [45, 0, 0, 0], thruster: 0}' 2>/dev/null

printInfo "Testing side servos, publishing to 'kinematics/command'..."
timeout 3 ros2 topic pub -1 $NAMESPACE/kinematics/command frost_interfaces/msg/UCommand '{fin: [0, 45, 45, 0], thruster: 0}' 2>/dev/null

printInfo "Testing thruster (ON), publishing to 'kinematics/command'..."
timeout 3 ros2 topic pub -1 $NAMESPACE/kinematics/command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 10}' 2>/dev/null

printInfo "Testing thruster (OFF), publishing to 'kinematics/command'..."
timeout 3 ros2 topic pub -1 $NAMESPACE/kinematics/command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 0}' 2>/dev/null
