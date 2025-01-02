#!/bin/bash
# Created by Matthew McMurray, Sep 2024
#

# Check if the correct number of arguments is provided
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <speed_of_sound>"
  exit 1
fi

# Assign the first argument to the variable
SPEED_OF_SOUND=$1

# Build the JSON string
JSON_STRING='"set_config","parameters":{"speed_of_sound":'"$SPEED_OF_SOUND"'}'

source send_command.sh

# Main execution
MAX_ATTEMPTS=3
TIMEOUT=3

execute_with_retry $JSON_STRING "Set speed of sound" $TIMEOUT || exit 1

# Send the JSON string to the specified IP and port using netcat
# echo -n "$JSON_STRING" | nc -q 0 192.168.194.95 16171

