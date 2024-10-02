#!/bin/bash
# Created by Matthew McMurray, Sep 2024
#
# ADD HERE

echo -n '{"command": "reset_dead_reckoning"}' | nc -q 0 192.168.194.95 16171


