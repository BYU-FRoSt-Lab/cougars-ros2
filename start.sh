#!/bin/bash

##########################################################
# STARTS THE AGENT AND RUNS A SPECIFIED LAUNCH FILE
# - Specify a start configuration using 'bash start.sh 
#   <launch>' (ex. 'bash start.sh moos')
# - Log files are saved in "~/bag" on the host machine
#   running the docker container
##########################################################

# Prompt for sudo password
sudo echo ""

echo -e "\e[38;5;6m  ██████          ██    ██  ██████   █████  ██████  ███████ \e[0m"
echo -e "\e[38;5;6m ██       ██████  ██    ██ ██       ██   ██ ██   ██ ██      \e[0m" 
echo -e "\e[38;5;6m ██      ██    ██ ██    ██ ██   ███ ███████ ██████  ███████ \e[0m" 
echo -e "\e[38;5;6m ██      ██    ██ ██    ██ ██    ██ ██   ██ ██   ██      ██ \e[0m" 
echo -e "\e[38;5;6m  ██████  ██████   ██████   ██████  ██   ██ ██   ██ ███████ \e[0m" 
echo ""
echo -e "BYU FROST LAB - CONFIGURABLE UNDERWATER GROUP OF AUTONOMOUS ROBOTS"
echo ""

bash ~/teensy_ws/strobe.sh on

cd ~/microros_ws
source install/setup.bash
# ros2 run micro_ros_agent micro_ros_agent multiserial --devs "/dev/ttyACM0 /dev/ttyACM1" -b 6000000 &
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &
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
        cd ~/ros2_ws/moos_tools
        bash mission_start_processes.sh
        bash mission_deploy.sh
        
        cd ~/ros2_ws/bag
        ros2 launch cougars_py moos_launch.py

        bash ~/ros2_ws/moos_tools/mission_kill.sh
        ;;
    *)
        echo ""
        echo "ALERT: No start configuration specified, defaulting to 'manual'"
        echo "Specify a start configuration using 'bash start.sh <config>' (ex. 'bash start.sh moos')"
        echo ""

        ros2 launch cougars_py manual_launch.py
        ;;
esac

bash ~/teensy_ws/strobe.sh off

killall micro_ros_agent
wait
