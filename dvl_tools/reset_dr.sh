#!/bin/bash
# Created by Matthew McMurray, Sep 2024
#
# ADD HERE
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

IPADDRESS=192.168.194.95

# Function to send set_config request
send_set_config() {
    JSON_STRING='{"command": "reset_dead_reckoning"}'
    response=$(echo -n "$JSON_STRING" | nc -w 3 $IPADDRESS 16171)

    if [ -z "$response" ]; then
        printError "No response received from DVL."
        return 1
    fi

    success=$(echo "$response" | grep -o '"success":true' | wc -l)

    if [ "$success" -eq 1 ]; then
        printSuccess "Dead Reckoning Report is reset as requested."
        return 0
    else
        printError "Failed to retrieve DVL response"
        echo "Response: $response"
        return 1
    fi
}

# Main execution with retry logic
MAX_ATTEMPTS=3
attempt=1
success=false

while [ $attempt -le $MAX_ATTEMPTS ] && [ "$success" = false ]; do
    
    if send_set_config; then
        success=true
    else
        if [ $attempt -lt $MAX_ATTEMPTS ]; then
            printInfo "Attempt $attempt of $MAX_ATTEMPTS"
            printWarning "DVL config attempt failed. Retrying ..."
        fi
    fi
    
    ((attempt++))
done

if [ "$success" = false ]; then
    printFailure "Failed to set and verify acoustic_enabled after $MAX_ATTEMPTS attempts."
    exit 1
fi

