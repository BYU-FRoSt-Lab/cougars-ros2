#!/bin/bash
# Created by Matthew McMurray, Sep 2024
#
# ADD HERE

source ~/config/cougarsrc.sh

echo -n '{"command": "get_config"}' | nc -q 0 $DVL_IP_ADDRESS 16171

