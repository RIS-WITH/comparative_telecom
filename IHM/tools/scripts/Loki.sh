#!/bin/bash
# start a new bash and go to the home directory
cd
# Display help message
display_help() {
    echo "Usage: ./Loki.sh <middleware>"
    echo "Middleware options:"
    echo "  default         Default middleware"
    echo "  fastdds         FastDDS middleware"
    echo "  cyclone         Cyclone middleware"
    echo "  zenoh           Zenoh middleware"

    echo "If you would like to run zenoh without rostools websocket bridge, please run the following command:"
    echo "  ./Loki.sh zenoh --no-rosbridge"
}

# Exit if no arguments are passed
if [ $# -eq 0 ]; then
    display_help
    exit 1
fi

# Validate middleware argument
middleware=$1
# If help message is asked
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
            echo "Using alias source_zenoh to source Zenoh environment."
            source $zenoh_ws/install/setup.bash
            ;;
    esac
}

# Source ROS2 and workspace environments
source /opt/ros/iron/setup.bash || { echo "Failed to source ROS2 environment."; exit 1; }
source $test_ws/install/setup.bash || { echo "Failed to source workspace environment."; exit 1; }

# Set middleware environment
source_middleware $middleware

# Check if ROS bridge is enabled
if [ "$2" == "--no-rosbridge" ]; then
    echo "ROS bridge is disabled."
    echo "Starting Loki node..."
    ros2 run loki loki_node
    exit 0
fi
# Run both commands in the same terminal with feedback
echo "Starting ROS bridge and Loki node..."
(
    echo "Starting ROS bridge..."
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
) &

(
    echo "Starting Loki node..."
    ros2 run loki loki_node
) &

# Wait for both processes to complete
wait
