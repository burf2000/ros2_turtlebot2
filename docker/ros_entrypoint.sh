#!/bin/bash
set -e

# Install xacro if not present (for older Docker images)
if ! command -v xacro &> /dev/null; then
    pip3 install xacro -q 2>/dev/null || true
fi

# Source ROS2 setup
source /opt/ros/humble/install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true

# Source workspace if it exists
if [ -f "/root/turtlebot2_ws/install/setup.bash" ]; then
    source /root/turtlebot2_ws/install/setup.bash
fi

# Set ROS domain ID (change if needed for multi-robot)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Execute command
exec "$@"
