#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Initializes the vehicle, resets dead reckoning, and starts the MOOS-IvP mission
# - Specify a start configuration using 'bash record.sh <launch>' (ex. 'bash record.sh moos')
# - Run this after running the 'launch.sh' and 'test.sh' scripts

source ~/config/bash_vars.sh

cleanup() {
    case $1 in
        "moos")
            bash ~/ros2_ws/moos_tools/mission_kill.sh
            ;;
    esac
    exit 0
}
trap cleanup SIGINT

source ~/ros2_ws/install/setup.bash
ros2 topic pub $NAMESPACE/init std_msgs/msg/Empty -1

# bash ~/ros2_ws/dvl_tools/reset_dr.sh
# echo "[COMPLETE] Reset DVL dead reckoning"

# Start the MOOS-IvP mission
case $1 in
    "moos")
        echo ""
        bash ~/ros2_ws/moos_tools/mission_start_processes.sh
        bash ~/ros2_ws/moos_tools/mission_deploy.sh
        ;;
esac

cleanup $1
