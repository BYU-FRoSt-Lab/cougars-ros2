#!/bin/bash

##########################################################
# RESETS DR, STARTS HLC, AND RECORDS A ROS2 BAG FILE
# - Specify a start configuration using 'bash start.sh 
#   <launch>' (ex. 'bash start.sh moos')
# - Run this after running the 'start.sh' and 'test.sh'
#   scripts
# - Log files are saved in "~/bag" on the host machine
#   running the docker container
##########################################################

cleanup() {

    case $1 in
        moos)
            bash ~/ros2_ws/moos_tools/mission_kill.sh
            ;;
    esac

    exit 0
}
trap cleanup SIGINT

echo ""
echo "Enter a descriptive folder name for the rosbag: "
read FOLDER
echo ""

cd ~/ros2_ws
source install/setup.bash

# Reset dead reckoning and start factor graph, high-level controls
bash ~/ros2_ws/dvl_tools/reset_dr.sh
ros2 topic pub /init std_msgs/msg/Empty -1

# Start the MOOS-IvP mission
case $1 in
    moos)
        bash ~/ros2_ws/moos_tools/mission_start_processes.sh
        bash ~/ros2_ws/moos_tools/mission_deploy.sh
        ;;
esac

FOLDER=$FOLDER-$(date +"%Y-%m-%d-%H-%M-%S")
ros2 bag record -o /home/frostlab/ros2_ws/bag/$FOLDER -s mcap -a
