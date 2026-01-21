#!/bin/bash
# Save a map from SLAM
# Usage: ./scripts/save_map.sh [map_name]

set -e

MAP_NAME=${1:-map}
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
MAP_FILE="${MAP_NAME}_${TIMESTAMP}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

# Detect Jetson
if [ -f /etc/nv_tegra_release ]; then
    COMPOSE_FILES="-f docker-compose.yml -f docker-compose.jetson.yml"
else
    COMPOSE_FILES="-f docker-compose.yml"
fi

echo "Saving map as: ${MAP_FILE}"

docker-compose $COMPOSE_FILES run --rm turtlebot2 \
    ros2 run nav2_map_server map_saver_cli -f /root/maps/${MAP_FILE}

echo ""
echo "Map saved to: data/maps/${MAP_FILE}.yaml"
echo "              data/maps/${MAP_FILE}.pgm"
