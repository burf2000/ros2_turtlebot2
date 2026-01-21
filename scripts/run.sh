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
        echo "Starting SLAM..."
        docker-compose $COMPOSE_FILES run --rm turtlebot2 \
            ros2 launch turtlebot2_bringup slam.launch.py
        ;;
    nav)
        echo "Starting Nav2..."
        docker-compose $COMPOSE_FILES run --rm turtlebot2 \
            ros2 launch turtlebot2_bringup nav2.launch.py
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
        echo "Usage: $0 {bash|bringup|kobuki|camera|teleop|slam|nav|build|dev}"
        echo ""
        echo "Commands:"
        echo "  bash    - Start interactive shell"
        echo "  bringup - Launch full robot stack"
        echo "  kobuki  - Launch Kobuki base only"
        echo "  camera  - Launch Astra camera only"
        echo "  teleop  - Start keyboard teleop"
        echo "  slam    - Start SLAM mapping"
        echo "  nav     - Start Nav2 navigation"
        echo "  build   - Build Docker image"
        echo "  dev     - Start development container"
        exit 1
        ;;
esac
