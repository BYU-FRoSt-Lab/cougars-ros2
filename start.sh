#!/bin/bash

##########################################################
# STARTS THE MICRO-ROS AGENT AND RUNS THE LAUNCH FILE
# - Log files are saved in ~/bag on the host machine
#   running the docker container
##########################################################

echo ""
echo -e "\e[38;5;6m  ██████          ██    ██  ██████   █████  ██████  ███████ \e[0m"
echo -e "\e[38;5;6m ██       ██████  ██    ██ ██       ██   ██ ██   ██ ██      \e[0m" 
echo -e "\e[38;5;6m ██      ██    ██ ██    ██ ██   ███ ███████ ██████  ███████ \e[0m" 
echo -e "\e[38;5;6m ██      ██    ██ ██    ██ ██    ██ ██   ██ ██   ██      ██ \e[0m" 
echo -e "\e[38;5;6m  ██████  ██████   ██████   ██████  ██   ██ ██   ██ ███████ \e[0m" 
echo ""
echo -e "BYU FROST LAB - CONFIGURABLE UNDERWATER GROUP OF AUTONOMOUS ROBOTS"
echo ""

cd ~/teensy_ws
bash strobe_on.sh

echo ""

cd ~/microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent multiserial --devs "/dev/ttyACM0 /dev/ttyACM1" -b 6000000 &
sleep 5

echo ""

cd ~/ros2_ws
source install/setup.bash

# ADD CALIBRATION SCRIPT w ROS TOPIC PUB, ROS PARAMS

cd ~/ros2_ws/bag
ros2 launch cougars cougars_launch.py

killall micro_ros_agent
wait