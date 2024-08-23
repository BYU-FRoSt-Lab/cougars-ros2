
# instructions to prepare DVL for dead reckoning
# https://waterlinked.github.io/dvl/dead-reckoning/ :

# Introduction

# The DVL-A50 and DVL-A125 run a dead reckoning algorithm which estimates the orientation and position of the DVL. This uses both the velocities which the DVL calculates and its integrated IMU (Inertial Measurement Unit).
# Starting dead reckoning

#     Calibrate the gyroscope by pressing More -> Calibrate gyro in the GUI whilst the DVL is stationary.
#     Click the 'Reset' button in the GUI, or send a reset command over the TCP or serial protocol.

# Failure to perform gyro calibration will result in less accurate dead reckoning.


# this python script fulfills this requirement
import subprocess

def run_ros2_commands():
    # Define the commands with the given parameter value
    command1 = "ros2 topic pub /dvl/config/command dvl_msgs/ConfigCommand \"{command: 'calibrate_gyro'}\" -1"
    command2 = "ros2 topic pub /dvl/config/command dvl_msgs/ConfigCommand \"{command: 'reset_dead_reckoning'}\" -1"
    

    try:
        # Run the first command
        print("calibrating gyro")
        result1 = subprocess.run(command1, shell=True, check=True)
        
        # Run the second command if the first one succeeds
        print("resetting dead reckoning")
        result2 = subprocess.run(command2, shell=True, check=True)
        
        print("Commands executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")

if __name__ == "__main__":
    # Run the commands with the provided parameter value
    print("hold dvl still in water")
    run_ros2_commands()
