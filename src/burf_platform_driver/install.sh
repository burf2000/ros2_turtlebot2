#!/bin/bash
#
# The Platform ROS2 Driver - Easy Install Script
#
# Run with: bash <(curl -sSL https://platform.burf.co/install/ros2)
#
# This script will:
# 1. Install the ROS2 driver
# 2. Configure your robot's topics
# 3. Set up your API key
# 4. Optionally enable auto-start on boot
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
BOLD='\033[1m'

# Print banner
echo -e "${CYAN}"
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                                                               ║"
echo "║   ${BOLD}The Platform - ROS2 Driver Installer${NC}${CYAN}                        ║"
echo "║                                                               ║"
echo "║   Connect your ROS2 robot to The Platform                    ║"
echo "║   https://platform.burf.co                                    ║"
echo "║                                                               ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo -e "${YELLOW}Warning: Running as root. The driver will be installed system-wide.${NC}"
fi

# Check for ROS2
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}Error: ROS2 is not installed or not in PATH.${NC}"
    echo "Please install ROS2 first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# Detect ROS2 distro
ROS_DISTRO=${ROS_DISTRO:-$(ls /opt/ros/ 2>/dev/null | head -1)}
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: Could not detect ROS2 distribution.${NC}"
    exit 1
fi
echo -e "${GREEN}Detected ROS2 distribution: ${BOLD}$ROS_DISTRO${NC}"

# Detect ROS2 workspace
DEFAULT_WS="$HOME/ros2_ws"
if [ -d "$HOME/colcon_ws" ]; then
    DEFAULT_WS="$HOME/colcon_ws"
elif [ -d "$HOME/robot_ws" ]; then
    DEFAULT_WS="$HOME/robot_ws"
fi

echo ""
echo -e "${BOLD}Step 1: ROS2 Workspace${NC}"
echo -e "Enter your ROS2 workspace path [${CYAN}$DEFAULT_WS${NC}]: "
read -r ROS2_WS
ROS2_WS=${ROS2_WS:-$DEFAULT_WS}

# Create workspace if it doesn't exist
if [ ! -d "$ROS2_WS/src" ]; then
    echo -e "${YELLOW}Workspace not found. Creating $ROS2_WS...${NC}"
    mkdir -p "$ROS2_WS/src"
fi

# ─────────────────────────────────────────────────────────────────────────────
# Robot Configuration
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}Step 2: Robot Configuration${NC}"

# Robot ID (unique identifier)
DEFAULT_ROBOT_ID="ros2-rover-$(hostname | tr '[:upper:]' '[:lower:]' | tr -cd '[:alnum:]-')"
echo -e "Enter a unique Robot ID [${CYAN}$DEFAULT_ROBOT_ID${NC}]: "
read -r ROBOT_ID
ROBOT_ID=${ROBOT_ID:-$DEFAULT_ROBOT_ID}

# Robot Name (friendly display name)
DEFAULT_ROBOT_NAME="ROS2 Rover"
echo -e "Enter a display name for your robot [${CYAN}$DEFAULT_ROBOT_NAME${NC}]: "
read -r ROBOT_NAME
ROBOT_NAME=${ROBOT_NAME:-$DEFAULT_ROBOT_NAME}

# ─────────────────────────────────────────────────────────────────────────────
# Topic Configuration
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}Step 3: ROS2 Topic Configuration${NC}"
echo -e "${YELLOW}Press Enter to accept defaults, or type your custom topic name.${NC}"
echo ""

# Camera topic
DEFAULT_CAMERA="/camera/image_raw"
echo -e "Camera topic (Image or CompressedImage) [${CYAN}$DEFAULT_CAMERA${NC}]: "
read -r CAMERA_TOPIC
CAMERA_TOPIC=${CAMERA_TOPIC:-$DEFAULT_CAMERA}

# Odometry topic
DEFAULT_ODOM="/odom"
echo -e "Odometry topic (nav_msgs/Odometry) [${CYAN}$DEFAULT_ODOM${NC}]: "
read -r ODOM_TOPIC
ODOM_TOPIC=${ODOM_TOPIC:-$DEFAULT_ODOM}

# IMU topic
DEFAULT_IMU="/imu/data"
echo -e "IMU topic (sensor_msgs/Imu) [${CYAN}$DEFAULT_IMU${NC}]: "
read -r IMU_TOPIC
IMU_TOPIC=${IMU_TOPIC:-$DEFAULT_IMU}

# LaserScan topic
DEFAULT_SCAN="/scan"
echo -e "LaserScan topic (sensor_msgs/LaserScan) [${CYAN}$DEFAULT_SCAN${NC}]: "
read -r SCAN_TOPIC
SCAN_TOPIC=${SCAN_TOPIC:-$DEFAULT_SCAN}

# Cmd_vel topic
DEFAULT_CMD_VEL="/cmd_vel"
echo -e "Velocity command topic (geometry_msgs/Twist) [${CYAN}$DEFAULT_CMD_VEL${NC}]: "
read -r CMD_VEL_TOPIC
CMD_VEL_TOPIC=${CMD_VEL_TOPIC:-$DEFAULT_CMD_VEL}

# ─────────────────────────────────────────────────────────────────────────────
# API Key
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}Step 4: API Key${NC}"
echo -e "Get your API key from: ${CYAN}https://platform.burf.co/profile${NC}"
echo ""
echo -e "Enter your API Key: "
read -r API_KEY

if [ -z "$API_KEY" ]; then
    echo -e "${YELLOW}Warning: No API key provided. Your robot won't be linked to your account.${NC}"
    echo -e "You can add it later in the config file."
fi

# ─────────────────────────────────────────────────────────────────────────────
# Install the driver
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}Step 5: Installing ROS2 Driver${NC}"

# Download the driver
DRIVER_DIR="$ROS2_WS/src/burf_platform_driver"
echo -e "Downloading driver from The Platform..."

# Download and extract driver tarball
curl -sSL https://platform.burf.co/install/ros2_driver.tar.gz -o /tmp/ros2_driver.tar.gz
if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Failed to download driver. Check your internet connection.${NC}"
    exit 1
fi

# Extract to workspace
cd "$ROS2_WS/src"
tar -xzf /tmp/ros2_driver.tar.gz
rm /tmp/ros2_driver.tar.gz

# Rename extracted folder if needed
if [ -d "ros2_driver" ] && [ ! -d "burf_platform_driver" ]; then
    mv ros2_driver burf_platform_driver
fi

DRIVER_DIR="$ROS2_WS/src/burf_platform_driver"
echo -e "${GREEN}Driver downloaded to: $DRIVER_DIR${NC}"

# Install Python dependencies
echo -e "Installing Python dependencies..."

# Required dependencies (always install)
pip3 install -q "websockets>=12.0,<14.0" "psutil>=5.9.0,<7.0" 2>/dev/null || \
pip3 install --user -q "websockets>=12.0,<14.0" "psutil>=5.9.0,<7.0"

# Optional dependencies for camera/WebRTC (may fail on some systems)
echo -e "Attempting to install camera dependencies (opencv, aiortc, av)..."
if pip3 install -q "opencv-python-headless>=4.8.0,<5.0" "aiortc>=1.6.0,<2.0" "av>=10.0.0,<14.0" "numpy<2.0" 2>/dev/null || \
   pip3 install --user -q "opencv-python-headless>=4.8.0,<5.0" "aiortc>=1.6.0,<2.0" "av>=10.0.0,<14.0" "numpy<2.0" 2>/dev/null; then
    echo -e "${GREEN}Camera/WebRTC support installed successfully${NC}"
else
    echo -e "${YELLOW}Note: Camera dependencies could not be installed.${NC}"
    echo -e "${YELLOW}The driver will work but without video streaming.${NC}"
    echo -e "${YELLOW}You can try installing them manually later with:${NC}"
    echo -e "${CYAN}  pip3 install opencv-python-headless aiortc av \"numpy<2.0\"${NC}"
fi

# Create configuration file
CONFIG_FILE="$DRIVER_DIR/config/burf_driver.yaml"
mkdir -p "$DRIVER_DIR/config"

cat > "$CONFIG_FILE" << EOF
/**:
  ros__parameters:
    # Server configuration
    server_url: "wss://platform.burf.co/ws/robot"
    robot_id: "$ROBOT_ID"
    robot_name: "$ROBOT_NAME"
    api_key: "$API_KEY"

    # ROS2 topic names
    camera_topic: "$CAMERA_TOPIC"
    use_compressed: false
    odom_topic: "$ODOM_TOPIC"
    imu_topic: "$IMU_TOPIC"
    scan_topic: "$SCAN_TOPIC"
    cmd_vel_topic: "$CMD_VEL_TOPIC"

    # Performance tuning
    telemetry_rate: 5.0
    video_fps: 10
EOF

echo -e "${GREEN}Configuration saved to: $CONFIG_FILE${NC}"

# Build the package
echo -e "Building the ROS2 package..."
cd "$ROS2_WS"
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select burf_platform_driver --symlink-install 2>&1 | tail -5

if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed. Check the error messages above.${NC}"
    exit 1
fi

echo -e "${GREEN}Build successful!${NC}"

# ─────────────────────────────────────────────────────────────────────────────
# Auto-start configuration
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}Step 6: Auto-start Configuration${NC}"
echo -e "Would you like the driver to start automatically on boot? [y/N]: "
read -r AUTOSTART

LAUNCH_CMD="ros2 launch burf_platform_driver burf_driver.launch.py"

if [[ "$AUTOSTART" =~ ^[Yy]$ ]]; then
    # Create systemd service
    SERVICE_FILE="/etc/systemd/system/burf-platform.service"

    # Check if we can write to systemd
    if [ "$EUID" -eq 0 ] || sudo -n true 2>/dev/null; then
        sudo tee "$SERVICE_FILE" > /dev/null << EOF
[Unit]
Description=The Platform ROS2 Driver
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER
Environment="ROS_DISTRO=$ROS_DISTRO"
ExecStart=/bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && source $ROS2_WS/install/setup.bash && $LAUNCH_CMD'
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

        sudo systemctl daemon-reload
        sudo systemctl enable burf-platform.service

        echo -e "${GREEN}Auto-start enabled!${NC}"
        echo -e "The driver will start automatically on boot."
        echo ""
        echo -e "Service commands:"
        echo -e "  ${CYAN}sudo systemctl start burf-platform${NC}   - Start now"
        echo -e "  ${CYAN}sudo systemctl stop burf-platform${NC}    - Stop"
        echo -e "  ${CYAN}sudo systemctl status burf-platform${NC}  - Check status"
        echo -e "  ${CYAN}journalctl -u burf-platform -f${NC}       - View logs"
    else
        echo -e "${YELLOW}Cannot create systemd service without sudo access.${NC}"
        echo -e "Run this script with sudo to enable auto-start, or manually create the service."
        AUTOSTART="n"
    fi
fi

if [[ ! "$AUTOSTART" =~ ^[Yy]$ ]]; then
    echo ""
    echo -e "${BOLD}To start the driver manually:${NC}"
    echo ""
    echo -e "  ${CYAN}source /opt/ros/$ROS_DISTRO/setup.bash${NC}"
    echo -e "  ${CYAN}source $ROS2_WS/install/setup.bash${NC}"
    echo -e "  ${CYAN}$LAUNCH_CMD${NC}"
fi

# ─────────────────────────────────────────────────────────────────────────────
# Summary
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                                                               ║${NC}"
echo -e "${GREEN}║   ${BOLD}Installation Complete!${NC}${GREEN}                                     ║${NC}"
echo -e "${GREEN}║                                                               ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BOLD}Summary:${NC}"
echo -e "  Robot Name:   ${CYAN}$ROBOT_NAME${NC}"
echo -e "  Robot ID:     ${CYAN}$ROBOT_ID${NC}"
echo -e "  Workspace:    ${CYAN}$ROS2_WS${NC}"
echo -e "  Config file:  ${CYAN}$CONFIG_FILE${NC}"
echo ""
echo -e "${BOLD}Topics configured:${NC}"
echo -e "  Camera:       ${CYAN}$CAMERA_TOPIC${NC}"
echo -e "  Odometry:     ${CYAN}$ODOM_TOPIC${NC}"
echo -e "  IMU:          ${CYAN}$IMU_TOPIC${NC}"
echo -e "  LaserScan:    ${CYAN}$SCAN_TOPIC${NC}"
echo -e "  Cmd Vel:      ${CYAN}$CMD_VEL_TOPIC${NC}"
echo ""
echo -e "${BOLD}Next steps:${NC}"
echo -e "  1. Make sure your robot's sensors are publishing to the configured topics"
echo -e "  2. Start the driver (see commands above)"
echo -e "  3. Go to ${CYAN}https://platform.burf.co/robots${NC} to see your robot!"
echo ""
echo -e "Need help? Visit ${CYAN}https://platform.burf.co/faq${NC}"
echo ""
