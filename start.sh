#!/bin/bash

##########################################################
# STARTS THE AGENT AND RUNS A SPECIFIED LAUNCH FILE
# - Specify a start configuration using 'bash start.sh 
#   <launch>' (ex. 'bash start.sh moos')
# - Log files are saved in "~/bag" on the host machine
#   running the docker container
##########################################################

cleanup() {

    case $1 in
        moos)
            bash ~/ros2_ws/moos_tools/mission_kill.sh
            ;;
    esac

    bash ~/teensy_ws/strobe.sh off

    killall micro_ros_agent
    wait

    exit 0
}
trap cleanup SIGINT

sudo echo "" # Prompt for sudo password (bug fix for strobe.sh)
echo "BYU FROST LAB - CONFIGURABLE UNDERWATER GROUP OF AUTONOMOUS ROBOTS"
echo ""

# Start the strobe light
bash ~/teensy_ws/strobe.sh on

# Start the micro-ROS agent
if [ -z "$(tycmd list)" ]; then
    echo ""
    echo "ERROR: No Teensy boards avaliable to connect to"
    echo ""

else 
    cd ~/microros_ws
    source install/setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &

    sleep 5
fi

echo ""

# Start the ROS 2 launch files
cd ~/ros2_ws
source install/setup.bash

case $1 in
    manual)
        ros2 launch cougars_py manual_launch.py
        ;;
    moos)
        bash ~/ros2_ws/moos_tools/mission_start_processes.sh
        bash ~/ros2_ws/moos_tools/mission_deploy.sh
        
        ros2 launch cougars_py moos_launch.py
        ;;
    sensors)
        ros2 launch cougars_py sensors_launch.py
        ;;
    *)
        echo ""
        echo "ALERT: No start configuration specified, defaulting to 'manual'"
        echo "Specify a start configuration using 'bash start.sh <config>' (ex. 'bash start.sh moos')"
        echo ""

        ros2 launch cougars_py manual_launch.py
        ;;
esac
