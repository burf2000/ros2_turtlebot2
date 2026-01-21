#!/bin/bash
# Build ROS2 workspace inside Docker container
# Run this script inside the turtlebot2 container

set -e

echo "=== Building TurtleBot 2 ROS2 Workspace ==="

# Source ROS2
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd /root/turtlebot2_ws

# Install dependencies
echo "Installing rosdep dependencies..."
rosdep update --rosdistro=humble || true
rosdep install --from-paths src --ignore-src -r -y || true

# Build workspace
echo "Building workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash

echo ""
echo "=== Build Complete ==="
echo ""
echo "Workspace built successfully!"
echo "Source the workspace with: source install/setup.bash"
