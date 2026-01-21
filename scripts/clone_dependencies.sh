#!/bin/bash
# Clone all required dependencies for TurtleBot 2 ROS2
# Run this inside the Docker container before building

set -e

echo "=== Cloning TurtleBot 2 Dependencies ==="

cd /root/turtlebot2_ws/src

# Kobuki packages
echo "Cloning Kobuki packages..."
if [ ! -d "kobuki_core" ]; then
    git clone https://github.com/kobuki-base/kobuki_core.git
fi
if [ ! -d "kobuki_ros" ]; then
    git clone https://github.com/kobuki-base/kobuki_ros.git
fi
if [ ! -d "velocity_smoother" ]; then
    git clone https://github.com/kobuki-base/velocity_smoother.git
fi
if [ ! -d "cmd_vel_mux" ]; then
    git clone https://github.com/kobuki-base/cmd_vel_mux.git
fi

# ECL dependencies
echo "Cloning ECL dependencies..."
if [ ! -d "ecl_tools" ]; then
    git clone https://github.com/stonier/ecl_tools.git
fi
if [ ! -d "ecl_lite" ]; then
    git clone https://github.com/stonier/ecl_lite.git
fi
if [ ! -d "ecl_core" ]; then
    git clone https://github.com/stonier/ecl_core.git
fi

# Sophus (geometry library)
echo "Cloning Sophus..."
if [ ! -d "Sophus" ]; then
    git clone https://github.com/strasdat/Sophus.git
fi

# OrbbecSDK for Astra camera
echo "Cloning OrbbecSDK ROS2..."
if [ ! -d "OrbbecSDK_ROS2" ]; then
    git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
    cd OrbbecSDK_ROS2
    git checkout main
    cd ..
fi

echo ""
echo "=== Dependencies Cloned ==="
echo ""
echo "All dependencies have been cloned to /root/turtlebot2_ws/src"
echo "Next: Run ./scripts/build_workspace.sh to build everything"
