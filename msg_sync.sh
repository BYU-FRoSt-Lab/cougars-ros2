#!/bin/bash

# syncs changes made in the ros2 frost_interfaces package with the teensy_ws
rsync -avu --delete ~/ros2_ws/src/frost_interfaces ~/teensy_ws/sensors/extra_packages
rsync -avu --delete ~/ros2_ws/src/frost_interfaces ~/teensy_ws/control/extra_packages
rsync -avu --delete ~/ros2_ws/src/frost_interfaces ~/teensy_ws/test_ros/extra_packages