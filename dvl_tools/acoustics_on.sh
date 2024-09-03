#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <true|false>"
  exit 1
fi

# Assign the first argument to the variable
ACOUSTIC_ENABLED=$1

# Validate the input to ensure it's either "true" or "false"
if [[ "$ACOUSTIC_ENABLED" != "true" && "$ACOUSTIC_ENABLED" != "false" ]]; then
  echo "Error: Argument must be 'true' or 'false'."
  exit 1
fi

# Build the JSON string
JSON_STRING='{"command":"set_config","parameters":{"acoustic_enabled":'"$ACOUSTIC_ENABLED"'}}'

# Send the JSON string to the specified IP and port using netcat
echo -n "$JSON_STRING" | nc -q 0 192.168.194.95 16171

