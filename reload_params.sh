#!/bin/bash

# Script to reload parameters for a list of ROS2 nodes from a parameter file
# Usage: ./reload_params.sh <param_file> [node1 node2 node3...]
# If no nodes specified, it will attempt to reload for all nodes found in the param file

set -e

# Default parameter file
PARAM_FILE=${1:-"~/config/deploy_tmp/coug${VEHICLE_ID}_params.yaml"}

# Check if parameter file exists
if [ ! -f "$PARAM_FILE" ]; then
    echo "Error: Parameter file '$PARAM_FILE' not found!"
    echo "Usage: $0 <param_file> [node1 node2 node3...]"
    exit 1
fi

# If nodes are specified as arguments, use those. Otherwise extract from param file
if [ $# -gt 1 ]; then
    NODES=("${@:2}")  # All arguments after the first one
else
    # Extract node names from YAML file (assumes standard ROS2 param file format)
    NODES=($(grep -E "^[[:space:]]*[a-zA-Z_][a-zA-Z0-9_]*:" "$PARAM_FILE" | grep -v "ros__parameters" | sed 's/:.*$//' | tr -d ' '))
fi

echo "Reloading parameters from: $PARAM_FILE"
echo "Target nodes: ${NODES[*]}"
echo "----------------------------------------"

# Function to check if a node is running
check_node_running() {
    local node_name=$1
    ros2 node list | grep -q "/$node_name" || ros2 node list | grep -q "$node_name"
}

# Function to reload parameters for a single node
reload_node_params() {
    local node_name=$1
    
    echo "Reloading parameters for node: $node_name"
    
    # Check if node is running
    if ! check_node_running "$node_name"; then
        echo "  Warning: Node '$node_name' is not currently running"
        return 1
    fi
    
    # Load parameters from file
    if ros2 param load "/$node_name" "$PARAM_FILE" 2>/dev/null || ros2 param load "$node_name" "$PARAM_FILE" 2>/dev/null; then
        echo "  ✓ Successfully reloaded parameters for $node_name"
    else
        echo "  ✗ Failed to reload parameters for $node_name"
        return 1
    fi
}

# Main execution
success_count=0
total_count=${#NODES[@]}

for node in "${NODES[@]}"; do
    if reload_node_params "$node"; then
        ((success_count++))
    fi
    echo
done

echo "----------------------------------------"
echo "Parameter reload complete: $success_count/$total_count nodes updated successfully"

if [ $success_count -eq $total_count ]; then
    echo "All parameters reloaded successfully!"
    exit 0
else
    echo "Some nodes failed to reload parameters. Check the output above."
    exit 1
fi