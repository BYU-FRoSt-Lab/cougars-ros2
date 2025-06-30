#!/bin/bash
# Created by Matthew McMurray, Sep 2024
#
# ADD HERE

echo -n '{"command": "get_config"}' | nc -q 0 192.168.2.95 16171

