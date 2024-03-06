#!/bin/bash

cd ~/microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent multiserial --devs "/dev/ttyACM0 /dev/ttyACM1" -b 6000000 &
sleep 5

cd ~/ros2_ws
source install/setup.bash

echo ""
echo "LISTING FOUND TOPICS..."
ros2 topic list

echo ""
echo "PUBLISHING TO TOPIC 'PID_REQUEST'..."
ros2 topic pub -1 /pid_request frost_interfaces/msg/PID '{velocity: 0.0, yaw: 90.0, pitch: 0.0, roll: 0.0, depth: 0.0}'

echo ""
echo "LISTENING TO TOPIC 'IMU_DATA'..."
ros2 topic echo --once /imu_data

echo ""
echo "LISTENING TO TOPIC 'DEPTH_DATA'..."
ros2 topic echo --once /depth_data

echo ""
echo "LISTENING TO TOPIC 'LEAK_DETECTED'..."
ros2 topic echo --once /leak_detected

echo ""
echo "LISTENING TO TOPIC 'VOLTAGE'..."
ros2 topic echo --once /voltage

echo ""
echo "LISTENING TO TOPIC 'ECHO_DATA'..."
ros2 topic echo --once /echo_data

echo ""
echo "TEST COMPLETE"

killall micro_ros_agent
wait