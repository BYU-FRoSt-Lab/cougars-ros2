import argparse
import yaml

def update_yaml(file_path, param_name, param_value):
    """Updates or adds a parameter value in the YAML file."""
    try:
        # Load the YAML file
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file) or {}

        # Update the parameter value
        node_name = next(iter(data), None)  # Assumes a single top-level node
        if node_name:
            data.setdefault(node_name, {})
            data[node_name].setdefault("ros__parameters", {})
            data[node_name]["ros__parameters"][param_name] = float(param_value)
        else:
            print("Error: Invalid YAML structure.")
            return

        # Save the updated YAML file
        with open(file_path, 'w') as file:
            yaml.safe_dump(data, file)

        print(f"Parameter '{param_name}' updated to '{param_value}' in '{file_path}'.")

    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Update a parameter in a YAML file.")
    parser.add_argument("--file", required=True, help="Path to the YAML file.")
    parser.add_argument("--param", required=True, help="Parameter name to update.")
    parser.add_argument("--value", required=True, help="New value for the parameter.")

    args = parser.parse_args()

    # Call the update function
    update_yaml(args.file, args.param, args.value)
