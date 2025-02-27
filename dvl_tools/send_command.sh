#!/bin/bash

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

# Function to send command and check for success
send_command() {
    local command=$1
    local description=$2
    local timeout=$3
    local JSON_STRING="{\"command\": $command}"
    local response=$(echo -n "$JSON_STRING" | nc -w $timeout -q 0 $IPADDRESS 16171)
    
    if [ -z "$response" ]; then
        printError "No response received from DVL."
        return 1
    fi

    local success=$(echo "$response" | grep -o '"success":true' | wc -l)

    if [ "$success" -eq 1 ]; then
        # printSuccess "$description successful."
        echo "Response: $response"
        return 0
    else
        printError "Failed to $description"
        echo "Response: $response"
        return 1
    fi
}

# Function to execute command with retry logic
execute_with_retry() {
    local command=$1
    local description=$2
    local timeout=$3
    local attempt=1
    local success=false

    while [ $attempt -le $MAX_ATTEMPTS ] && [ "$success" = false ]; do
        printInfo "$description: Attempt $attempt of $MAX_ATTEMPTS"
        if send_command "$command" "$description" "$timeout"; then
            printSuccess "$description completed successfully"
            success=true
        else
            if [ $attempt -lt $MAX_ATTEMPTS ]; then
                printWarning "$description failed. Retrying ..."
            fi
        fi
        
        ((attempt++))

    done

    if [ "$success" = false ]; then
        printFailure "Failed to $description after $MAX_ATTEMPTS attempts."
        return 1
    fi

    return 0
}

