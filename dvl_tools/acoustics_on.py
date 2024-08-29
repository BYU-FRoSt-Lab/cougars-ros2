import subprocess
import argparse

def main():
    # Set up argument parser

    parser = argparse.ArgumentParser(description="argument: true or false, dark mode enabled")
    parser.add_argument("parameter_value",  help="The parameter value to set (true/false)")

    # Parse the arguments
    args = parser.parse_args()


    # Parse arguments
    args = parser.parse_args()

    # Create the JSON data string
    speed_of_sound = "1475"
    dark_mode_enabled = "false"
    mounting_rotation_offset = 0
    url = "http://192.168.194.95/api/v1/config"

    data = f'{{"speed_of_sound":{speed_of_sound},"acoustic_enabled":{str(args.parameter_value).lower()},"dark_mode_enabled":{dark_mode_enabled},"mounting_rotation_offset":{mounting_rotation_offset}}}'

    # Define the curl command
    command = [
        "curl",
        "-X", "PUT",
        "-d", data,
        "-H", "Content-Type: application/json",
        url
    ]

    # Execute the curl command
    result = subprocess.run(command, capture_output=True, text=True)

    # Print the result
    if result.returncode == 0:
        print("Success:")
        print(result.stdout)
    else:
        print("Failed with status code:", result.returncode)
        print("Error message:", result.stderr)

if __name__ == "__main__":
    main()