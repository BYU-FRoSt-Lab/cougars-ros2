#!/bin/bash

cd ~/microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &
sleep 5

cd ~/ros2_ws
source install/setup.bash

echo ""
echo "TESTING PID REQUEST..."
ros2 topic pub -1 /pid_request frost_interfaces/msg/PID '{velocity: 0.0, yaw: 90.0, pitch: 0.0, roll: 0.0, depth: 0.0}'

echo ""
echo "CALLING GPS SERVICE..."
ros2 service call /gps_service frost_interfaces/srv/GetGPS "{test: True}"

echo ""
echo "CALLING ECHO SERVICE..."
ros2 service call /echo_service frost_interfaces/srv/GetEcho "{test: True}"

echo ""
echo "READING LEAK DATA (TRIGGER LEAK SENSOR)..."
ros2 topic echo --once /leak_detected

echo ""
echo "READING HUMIDITY DATA (TRIGGER HUMIDITY SENSOR)..."
ros2 topic echo --once /humidity

echo ""
echo "READING VOLTAGE DATA (TRIGGER VOLTAGE SENSOR)..."
ros2 topic echo --once /voltage

echo ""
echo "TEST COMPLETE"

killall micro_ros_agent
wait