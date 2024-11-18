#!/bin/bash
cd
# Display help message
display_help() {
    echo "Usage: ./Yuonobo.sh <middleware> <robot_IP_address>"
    echo "Middleware options:"
    echo "  default         Default middleware"
    echo "  fastdds         FastDDS middleware"
    echo "  cyclone         Cyclone middleware"
    echo "  zenoh           Zenoh middleware"

    echo "Robot IP address:"
    echo "  <robot_IP_address>  IP address of the robot"
    echo "  example: 127.0.0.1"
}
# Exit if no arguments are passed
if [ $# -eq 0 ]; then
    display_help
    exit 1
fi

# Check for help arguments
middleware=$1
if [[ "$middleware" =~ ^(-h|--help|help)$ ]]; then
    display_help
    exit 0
fi

robot_IP_address=$2
if [[ "$robot_IP_address" =~ ^(-h|--help|help)$ ]]; then
    display_help
    exit 0
fi
# Validate middleware argument
valid_middlewares=("default" "fastdds" "cyclone" "zenoh")
if [[ ! " ${valid_middlewares[@]} " =~ " ${middleware} " ]]; then
    echo "Invalid middleware argument. Please provide a valid option."
    display_help
    exit 1
fi

# Validate robot IP address
if [[ ! $robot_IP_address =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Invalid robot IP address. Please provide a valid IP address."
    display_help
    exit 1
fi

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

# Source ROS2 and workspace environments
source /opt/ros/iron/setup.bash || { echo "Failed to source ROS2 environment."; exit 1; }
source ros2_ws/install/setup.bash || { echo "Failed to source workspace environment."; exit 1; }

# Set middleware environment
source_middleware $middleware

echo "Environment setup complete. Starting Yunobo node with middleware: $middleware"
ros2 run yunobo yunobo_node --ros-args -p robot_ip:=$robot_IP_address
