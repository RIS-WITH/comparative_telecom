#!/bin/bash

# Display help message
display_help() {
    echo "Usage: ./Loki.sh <middleware>"
    echo "Middleware options:"
    echo "  default         Default middleware"
    echo "  fastdds         FastDDS middleware"
    echo "  cyclone         Cyclone middleware"
    echo "  zenoh           Zenoh middleware"
}

# Exit if no arguments are passed
if [ $# -eq 0 ]; then
    display_help
    exit 1
fi

# Validate middleware argument
middleware=$1
# if help message is asked
valid_help=("help" "-h" "--help" "h" "-help")
if [[ " ${valid_help[@]} " =~ " ${middleware} " ]]; then
    display_help
    exit 0
fi
valid_middlewares=("default" "fastdds" "cyclone" "zenoh")
if [[ ! " ${valid_middlewares[@]} " =~ " ${middleware} " ]]; then
    echo "Invalid middleware argument. Please provide a valid option."
    display_help
    exit 1
fi
cd 
# Check for ROS2 and workspace setup files
if [ ! -f /opt/ros/iron/setup.bash ]; then
    echo "ROS2 environment not found. Please install and source ROS2 Iron."
    exit 1
fi

if [ ! -f ros2_ws/install/setup.bash ]; then
    echo "ROS2 workspace not found. Please build the workspace first."
    exit 1
fi

# Function to set middleware-specific environment
source_middleware() {
    case $1 in
        "default")
            echo "Using default middleware."
            ;; # Default setup
        "fastdds")
            echo "Setting up FastDDS middleware."
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            ;;
        "cyclone")
            echo "Setting up Cyclone middleware."
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            ;;
        "zenoh")
            echo "Setting up Zenoh middleware."
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp
            source_zenoh
            ;;
    esac
}

# Open the first terminal for the ROS bridge
gnome-terminal -- bash -c "
echo 'Setting up environment for ROS bridge with middleware: $middleware';
source /opt/ros/iron/setup.bash;
source ros2_ws/install/setup.bash;
$(declare -f source_middleware); source_middleware $middleware;
ros2 launch rosbridge_server rosbridge_websocket_launch.xml;
exec bash
"

# Open the second terminal for the Loki node
gnome-terminal -- bash -c "
echo 'Setting up environment for Loki node with middleware: $middleware';
source /opt/ros/iron/setup.bash;
source ros2_ws/install/setup.bash;
$(declare -f source_middleware); source_middleware $middleware;
ros2 run loki loki_node;
exec bash
"
