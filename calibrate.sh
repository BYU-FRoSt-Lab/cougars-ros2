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

# Depth Sensor calibration:
echo "Calibrate depth sensor (y/n) - depth converter node needs to be running"

read -r choice

# If y then run this script:
if [[ "$choice" =~ ^[Yy]$ ]]; then
    handleServiceCall "$NAMESPACE/calibrate_depth" "std_srvs/srv/Trigger" "{}"
fi


#TEMP FIX - Only works inside the container
cd ~/ros2_ws/dvl_tools

# Calibrate Gyro:
echo "Calibrate gyro for DVL (y/n)"

read -r choice

if [[ "$choice" =~ ^[Yy]$ ]]; then
    bash ~/ros2_ws/dvl_tools/calibrate_gyro.sh
fi


echo "Set speed of sound for DVL (y/n)"
# Read user input
read -r choice

# If choice is yes then ask for input of speed of sound, if no skip
if [[ "$choice" =~ ^[Yy]$ ]]; then
    while true; do
        echo "Enter speed of sound (should be an int between 1400 and 1600):"
        read -r SPEED
        if [[ "$SPEED" =~ ^[0-9]+$ ]] && [ "$SPEED" -ge 1400 ] && [ "$SPEED" -le 1600 ]; then
            bash ~/ros2_ws/dvl_tools/set_speed_sound.sh "$SPEED"
            break
        else
            echo "Invalid input. Please enter an integer between 1400 and 1600."
        fi
    done
fi

# Set ntp server:
echo "Set ntp server for DVL (y/n)"

read -r choice

# If y then run this script:
if [[ "$choice" =~ ^[Yy]$ ]]; then
    bash ~/ros2_ws/dvl_tools/set_ntp.sh
fi