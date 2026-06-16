#!/bin/bash
# Quick run script for TurtleBot 2
# Usage: ./scripts/run.sh [command]
# Commands: bash, bringup, teleop, slam, nav

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

# Detect if running on Jetson
if [ -f /etc/nv_tegra_release ]; then
    COMPOSE_FILES="-f docker-compose.yml -f docker-compose.jetson.yml"
    echo "Detected Jetson platform"
else
    COMPOSE_FILES="-f docker-compose.yml"
    echo "Running on non-Jetson platform"
fi

COMMAND=${1:-bash}

case $COMMAND in
    bash)
        echo "Starting interactive shell..."
        docker-compose $COMPOSE_FILES run --rm turtlebot2 bash
        ;;
    bringup)
        echo "Starting TurtleBot 2 bringup..."
        docker-compose $COMPOSE_FILES run --rm turtlebot2 \
            ros2 launch turtlebot2_bringup turtlebot2.launch.py
        ;;
    kobuki)
        echo "Starting Kobuki base only..."
        docker-compose $COMPOSE_FILES run --rm turtlebot2 \
            ros2 launch turtlebot2_bringup kobuki.launch.py
        ;;
    camera)
        echo "Starting Astra camera only..."
        docker-compose $COMPOSE_FILES run --rm turtlebot2 \
            ros2 launch turtlebot2_bringup astra.launch.py
        ;;
    teleop)
        echo "Starting teleop keyboard..."
        docker-compose $COMPOSE_FILES run --rm turtlebot2 \
            ros2 run teleop_twist_keyboard teleop_twist_keyboard
        ;;
    slam)
        # SLAM runs INSIDE the live bringup container (shares DDS / sensors).
        # Includes the odom_tf_bridge (kobuki doesn't broadcast odom TF).
        echo "Starting SLAM (slam_toolbox + odom_tf_bridge)..."
        docker exec -d turtlebot2_bringup bash -c \
            'source /opt/ros/humble/setup.bash && source /root/turtlebot2_ws/install/setup.bash && ros2 launch turtlebot2_bringup slam.launch.py'
        echo "Launched. Verify: docker exec turtlebot2_bringup bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic hz /map'"
        ;;
    burf)
        # Burf Platform driver (+ raw->compressed camera republish), in the live container.
        echo "Starting Burf Platform driver (turtlebot2-01)..."
        docker exec -d turtlebot2_bringup bash -c \
            'source /opt/ros/humble/setup.bash && source /root/turtlebot2_ws/install/setup.bash && ros2 launch turtlebot2_bringup burf.launch.py'
        echo "Launched -> turtlebot2-01 on https://platform.burf.co (run 'slam' first for map + waypoint)"
        ;;
    nav)
        echo "Starting Nav2..."
        docker-compose $COMPOSE_FILES run --rm turtlebot2 \
            ros2 launch turtlebot2_bringup nav2.launch.py
        ;;
    desktop)
        echo "Starting desktop (RViz + teleop)..."
        docker-compose $COMPOSE_FILES run --rm turtlebot2 \
            ros2 launch turtlebot2_bringup desktop.launch.py
        ;;
    build)
        echo "Building Docker image..."
        docker-compose $COMPOSE_FILES build
        ;;
    dev)
        echo "Starting development container..."
        docker-compose $COMPOSE_FILES --profile dev run --rm turtlebot2-dev bash
        ;;
    *)
        echo "Usage: $0 {bash|bringup|kobuki|camera|teleop|desktop|slam|burf|nav|build|dev}"
        echo ""
        echo "Commands:"
        echo "  bash    - Start interactive shell"
        echo "  bringup - Launch full robot stack"
        echo "  kobuki  - Launch Kobuki base only"
        echo "  camera  - Launch Astra camera only"
        echo "  teleop  - Start keyboard teleop"
        echo "  desktop - Launch RViz and teleop (desktop side)"
        echo "  slam    - Start SLAM mapping (slam_toolbox + odom_tf_bridge)"
        echo "  burf    - Start Burf Platform driver (+ camera republish)"
        echo "  nav     - Start Nav2 navigation"
        echo "  build   - Build Docker image"
        echo "  dev     - Start development container"
        exit 1
        ;;
esac
