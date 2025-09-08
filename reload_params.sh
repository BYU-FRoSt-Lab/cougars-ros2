#!/bin/bash

# Script to reload parameters for a list of ROS2 nodes from a parameter file
# Usage: ./reload_params.sh <param_file> [node1 node2 node3...]
# If no nodes specified, it will attempt to reload for all nodes found in the param file

# Note: We don't use 'set -e' here because we want to continue trying other nodes
# even if one fails to reload parameters

# Default parameter file
PARAM_FILE=${1:-"/home/frostlab/config/deploy_tmp/coug${VEHICLE_ID}_params.yaml"}

# Check if parameter file exists
if [ ! -f "$PARAM_FILE" ]; then
    echo "Error: Parameter file '$PARAM_FILE' not found!"
    echo "Usage: $0 <param_file> [node1 node2 node3...]"
    exit 1
fi

# If nodes are specified as arguments, use those. Otherwise get all running nodes
if [ $# -gt 1 ]; then
    NODES=("${@:2}")  # All arguments after the first one
else
    # Get all currently running ROS2 nodes
    echo "Discovering running ROS2 nodes..."
    
    # Check if ROS2 is available
    if ! command -v ros2 &> /dev/null; then
        echo "Error: ros2 command not found. Make sure ROS2 is sourced."
        exit 1
    fi
    
    # Get node list and clean up the names
    if ! ros2_nodes=$(ros2 node list 2>/dev/null); then
        echo "Error: Could not get ROS2 node list. Make sure ROS2 is running."
        exit 1
    fi
    
    # Clean node names (remove leading slash and vehicle namespace)
    NODES=()
    while IFS= read -r node; do
        # Remove leading slash
        node=${node#/}
        # Remove vehicle namespace if present
        node=${node#coug$VEHICLE_ID/}
        # Trim whitespace
        node=$(echo "$node" | xargs)
        # Add to array if not empty
        if [[ -n "$node" ]]; then
            NODES+=("$node")
        fi
    done <<< "$ros2_nodes"
    
    if [ ${#NODES[@]} -eq 0 ]; then
        echo "Warning: No ROS2 nodes found running. Make sure ROS2 is running and nodes are active."
        exit 1
    fi
fi

echo "Reloading parameters from: $PARAM_FILE"
echo "Target nodes: ${NODES[*]}"
echo "Total nodes to process: ${#NODES[@]}"
echo "----------------------------------------"

# Function to reload parameters for a single node (designed for parallel execution)
reload_node_params() {
    local node_name=$1
    local temp_file="/tmp/reload_${node_name}_$$"
    
    {
        echo "node:$node_name"
        
        # Check if node is running
        if ! (ros2 node list | grep -q "/$node_name" || ros2 node list | grep -q "/coug$VEHICLE_ID/$node_name" || ros2 node list | grep -q "$node_name"); then
            echo "status:warning"
            echo "message:Node '$node_name' is not currently running"
            exit 1
        fi
        
        # Load parameters from file
        if ros2 param load "/coug$VEHICLE_ID/$node_name" "$PARAM_FILE" 2>/dev/null || ros2 param load "$node_name" "$PARAM_FILE" 2>/dev/null; then
            echo "status:success"
            echo "message:Successfully reloaded parameters for $node_name"
            exit 0
        else
            echo "status:failure"
            echo "message:Failed to reload parameters for $node_name"
            exit 1
        fi
    } > "$temp_file"
}

# Function to process results from parallel execution
process_results() {
    local pids=("$@")
    local success_count=0
    local total_count=${#pids[@]}
    
    # Wait for all background processes to complete
    for pid in "${pids[@]}"; do
        wait "$pid"
    done
    
    # Process results from temp files
    for node in "${NODES[@]}"; do
        local temp_file="/tmp/reload_${node}_$$"
        if [[ -f "$temp_file" ]]; then
            local node_name status message
            while IFS=':' read -r key value; do
                case "$key" in
                    "node") node_name="$value" ;;
                    "status") status="$value" ;;
                    "message") message="$value" ;;
                esac
            done < "$temp_file"
            
            echo "Reloading parameters for node: $node_name"
            case "$status" in
                "success")
                    echo "  ✓ $message"
                    ((success_count++))
                    ;;
                "warning")
                    echo "  ⚠ Warning: $message"
                    ;;
                "failure")
                    echo "  ✗ $message"
                    ;;
            esac
            
            # Clean up temp file
            rm -f "$temp_file"
        else
            echo "Reloading parameters for node: $node"
            echo "  ✗ Failed to get result"
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
}

# Main execution - run parameter reloads in parallel
echo "Starting parallel parameter reload for ${#NODES[@]} nodes..."
pids=()

# Start all reload operations in parallel
for node in "${NODES[@]}"; do
    reload_node_params "$node" &
    pids+=($!)
done

echo "Waiting for all reload operations to complete..."

# Process all results
process_results "${pids[@]}"
