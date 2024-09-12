#!/bin/bash

##########################################################
# TESTS EACH OF THE EXPECTED MICROROS TOPICS
# - Use this after setting up a new PCB to test the agent
#   and Teensy board connections
##########################################################

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

    killall micro_ros_agent
    wait
    
    exit 0
}
trap cleanup SIGINT

if [ -z "$(tycmd list)" ]; then
    echo ""
    printError "No Teensy boards avaliable to connect to"
    echo ""

    exit 1

else 
    cd ~/microros_ws
    source install/setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &
fi

cd ~/ros2_ws
source install/setup.bash

echo ""
echo "LISTING FOUND TOPICS..."
ros2 topic list

echo ""
echo "LISTENING TO TOPIC 'PRESSURE_DATA'..."
ros2 topic echo --once /pressure_data

echo ""
echo "LISTENING TO TOPIC 'LEAK_DATA'..."
ros2 topic echo --once /leak_data

echo ""
echo "LISTENING TO TOPIC 'BATTERY_DATA'..."
ros2 topic echo --once /battery_data

echo ""
echo "TESTING TOP SERVO, PUBLISHING TO 'CONTROL_COMMAND'..."
ros2 topic pub -1 /control_command frost_interfaces/msg/UCommand '{fin: [45, 0, 0, 0], thruster: 0}'

echo ""
echo "TESTING SIDE SERVOS, PUBLISHING TO 'CONTROL_COMMAND'..."
ros2 topic pub -1 /control_command frost_interfaces/msg/UCommand '{fin: [0, 45, 45, 0], thruster: 0}'

echo ""
echo "TESTING THRUSTER (ON), PUBLISHING TO 'CONTROL_COMMAND'..."
ros2 topic pub -1 /control_command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 10}'

echo ""
echo "TESTING THRUSTER (OFF), PUBLISHING TO 'CONTROL_COMMAND'..."
ros2 topic pub -1 /control_command frost_interfaces/msg/UCommand '{fin: [0, 0, 0, 0], thruster: 0}'

echo ""
echo "TEST COMPLETE"

cleanup
