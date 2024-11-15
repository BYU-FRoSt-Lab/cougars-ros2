#!/bin/bash
# Created by Matthew McMurray, Sep 2024
#
# ADD HERE

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

# Function to send set_config request
send_set_config() {
    JSON_STRING='{"command":"set_config","parameters":{"acoustic_enabled":'"$ACOUSTIC_ENABLED"'}}'
    echo -n "$JSON_STRING" | nc -w 5 -q 0 192.168.194.95 16171
    # Add a small delay to allow the change to take effect
    sleep 1
}

# Function to send get_config request and check response
check_config() {
    response=$(echo -n '{"command": "get_config"}' | nc -w 5 -q 0 192.168.194.95 16171)

    if [ -z "$response" ]; then
        echo "Error: No response received."
        exit 1
    fi

    success=$(echo "$response" | grep -o '"success":true' | wc -l)
    acoustic_status=$(echo "$response" | grep -o '"acoustic_enabled":'"$ACOUSTIC_ENABLED" | wc -l)

    if [ "$success" -eq 1 ] && [ "$acoustic_status" -eq 1 ]; then
        echo "Success: acoustic_enabled is set to $ACOUSTIC_ENABLED as requested."
    elif [ "$success" -eq 1 ]; then
        echo "Error: Request was successful, but acoustic_enabled value doesn't match the requested value."
        echo "Response: $response"
    else
        echo "Error: Failed to retrieve configuration."
        echo "Response: $response"
    fi
}

# Main execution
send_set_config
check_config
