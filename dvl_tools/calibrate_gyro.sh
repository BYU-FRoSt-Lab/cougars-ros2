#!/bin/bash
# Created by Matthew McMurray, Sep 2024
#
# ADD HERE

echo -n '{"command": "calibrate_gyro"}' | nc -q 0 192.168.194.95 16171

#TODO: Calibrate gyro verify calibration script 