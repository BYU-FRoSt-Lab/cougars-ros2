source ~/config/bash_vars.sh
source ~/ros2_ws/install/setup.bash

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

# Function to handle service call responses
function handleServiceCall {
  local service_output
  service_output=$(timeout 5s ros2 service call "$1" "$2" "$3" 2>&1)

  if echo "$service_output" | grep -q "success=True"; then
    local message
    message=$(echo "$service_output" | grep "message=" | sed -e 's/^.*message= //')
    printInfo "Service call to $1 succeeded. $message"
  elif echo "$service_output" | grep -q "success=False"; then
    local message
    message=$(echo "$service_output" | grep "message=" | sed -e 's/^.*message= //')
    printWarning "Service call to $1 failed with message: $message"
  else
    printError "Unexpected response from service $1: $service_output"
  fi
}

#TODO: Take the functions used in many scripts and put them into a utils script or something 

handleServiceCall "$NAMESPACE/calibrate_depth" "std_srvs/srv/Trigger" "{}"

bash ~/ros2_ws/dvl_tools/calibrate_gyro.sh
