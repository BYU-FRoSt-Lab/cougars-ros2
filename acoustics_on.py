import subprocess
import argparse

def run_ros2_commands(parameter_value):
    # Define the commands with the given parameter value
    command1 = f"ros2 topic pub /dvl/config/command dvl_msgs/ConfigCommand \"{{command: 'set_config', parameter_name: 'acoustic_enabled', parameter_value: '{parameter_value}'}}\" -1"
    command2 = "ros2 topic pub /dvl/config/command dvl_msgs/ConfigCommand \"{command: 'get_config'}\" -1"

    try:
        # Run the first command
        result1 = subprocess.run(command1, shell=True, check=True)
        
        # Run the second command if the first one succeeds
        result2 = subprocess.run(command2, shell=True, check=True)
        
        print("Commands executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")

if __name__ == "__main__":
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Run ROS 2 commands with a specified parameter value.")
    parser.add_argument("parameter_value", choices=["true", "false"], help="The parameter value to set (true/false) - keep lowercase.")
    
    # Parse the arguments
    args = parser.parse_args()

    
    # Run the commands with the provided parameter value
    run_ros2_commands(args.parameter_value)
