#!/bin/bash
# Created by Matthew McMurray, Sep 2024
# Modified by Braden Meyers, Nov 2024
# Modified by AI Assistant, Nov 2024

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

function printSuccess {
  echo -e "\033[0m\033[32m[SUCCESS] $1\033[0m"
}

function printFailure {
  echo -e "\033[0m\033[31m[FAIL] $1\033[0m"
}

# Check if the correct number of arguments is provided
if [ "$#" -ne 1 ]; then
  printError "Usage: $0 <true|false>"
  exit 1
fi

# Assign the first argument to the variable
ACOUSTIC_ENABLED=$1

# Validate the input to ensure it's either "true" or "false"
if [[ "$ACOUSTIC_ENABLED" != "true" && "$ACOUSTIC_ENABLED" != "false" ]]; then
  printError "DVL acoustics argument must be 'true' or 'false'."
  exit 1
fi

IPADDRESS=192.168.194.95

# Function to send set_config request
send_set_config() {
    JSON_STRING='{"command":"set_config","parameters":{"acoustic_enabled":'"$ACOUSTIC_ENABLED"'}}'
    echo -n "$JSON_STRING" | nc -w 5 -q 0 $IPADDRESS 16171
    # Add a small delay to allow the change to take effect
    sleep 1
}

# Function to send get_config request and check response
check_config() {
    response=$(echo -n '{"command": "get_config"}' | nc -w 5 -q 0 $IPADDRESS 16171)

    if [ -z "$response" ]; then
        printError "No response received from DVL."
        return 1
    fi

    success=$(echo "$response" | grep -o '"success":true' | wc -l)
    acoustic_status=$(echo "$response" | grep -o '"acoustic_enabled":'"$ACOUSTIC_ENABLED" | wc -l)

    if [ "$success" -eq 1 ] && [ "$acoustic_status" -eq 1 ]; then
        printSuccess "Acoustic_enabled is set to $ACOUSTIC_ENABLED as requested."
        return 0
    elif [ "$success" -eq 1 ]; then
        printError "DVL Request was successful, but acoustic_enabled value doesn't match the requested value."
        echo "Response: $response"
        return 1
    else
        printError "Failed to retrieve DVL configuration."
        echo "Response: $response"
        return 1
    fi
}

# Main execution with retry logic
MAX_ATTEMPTS=5
attempt=1
success=false

while [ $attempt -le $MAX_ATTEMPTS ] && [ "$success" = false ]; do
    
    send_set_config
    if check_config; then
        success=true
    else
        if [ $attempt -lt $MAX_ATTEMPTS ]; then
            printInfo "Attempt $attempt of $MAX_ATTEMPTS"
            printWarning "DVL config attempt failed. Retrying in 5 seconds..."
            sleep 5
        fi
    fi
    
    ((attempt++))
done

if [ "$success" = false ]; then
    printFailure "Failed to set and verify acoustic_enabled after $MAX_ATTEMPTS attempts."
    exit 1
fi