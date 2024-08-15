#!/bin/bash

##########################################################
# STARTS THE MICRO-ROS AGENT AND RUNS THE LAUNCH FILE
# - Log files are saved in "~/bag" on the host machine
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

cd ~/microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent multiserial --devs "/dev/ttyACM0 /dev/ttyACM1" -b 6000000 &
sleep 5

echo ""

cd ~/ros2_ws
source install/setup.bash
cd ~/ros2_ws/bag

case $1 in
    manual)
        ros2 launch cougars_py manual_launch.py
        ;;
    moos)
        ros2 launch cougars_py moos_launch.py
        ;;
    *)
        echo ""
        echo -e "\e[38;5;6mALERT: No start configuration specified, defaulting to 'manual'\e[0m"
        echo -e "\e[38;5;6mSpecify a start configuration using 'bash start.sh <launch>' (ex. 'bash start.sh moos')\e[0m"
        echo ""

        ros2 launch cougars_py manual_launch.py
        ;;
esac

killall micro_ros_agent
wait
