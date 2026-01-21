#!/bin/bash
# Setup script for Jetson Nano
# Run this on the Jetson Nano after flashing JetPack

set -e

echo "=== TurtleBot 2 Jetson Nano Setup ==="

# Update system
echo "Updating system packages..."
sudo apt-get update
sudo apt-get upgrade -y

# Install Docker if not present
if ! command -v docker &> /dev/null; then
    echo "Installing Docker..."
    sudo apt-get install -y docker.io
    sudo systemctl enable docker
    sudo systemctl start docker
    sudo usermod -aG docker $USER
    echo "Docker installed. You may need to log out and back in for group changes."
fi

# Install docker-compose
if ! command -v docker-compose &> /dev/null; then
    echo "Installing docker-compose..."
    sudo apt-get install -y python3-pip
    sudo pip3 install docker-compose
fi

# Clone jetson-containers for building ROS2 images
if [ ! -d "$HOME/jetson-containers" ]; then
    echo "Cloning jetson-containers..."
    cd $HOME
    git clone https://github.com/dusty-nv/jetson-containers
    cd jetson-containers
    bash install.sh
fi

# Setup udev rules for Kobuki
echo "Setting up Kobuki udev rules..."
sudo tee /etc/udev/rules.d/99-kobuki.rules > /dev/null << 'EOF'
# Kobuki base (FTDI USB-Serial)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", SYMLINK+="kobuki"
EOF

# Setup udev rules for Orbbec Astra
echo "Setting up Orbbec Astra udev rules..."
sudo tee /etc/udev/rules.d/99-orbbec.rules > /dev/null << 'EOF'
# Orbbec Astra cameras
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="1d27", MODE="0666", GROUP="plugdev"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Create data directories
echo "Creating data directories..."
mkdir -p ~/turtlebot2/data/maps
mkdir -p ~/turtlebot2/data/logs

# Set permissions
echo "Setting permissions..."
sudo chmod 666 /dev/ttyUSB* 2>/dev/null || true

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Next steps:"
echo "1. Reboot or log out/in for Docker group changes"
echo "2. Connect Kobuki base via USB"
echo "3. Connect Orbbec Astra camera via USB"
echo "4. Navigate to your turtlebot2 project directory"
echo "5. Run: docker-compose -f docker-compose.yml -f docker-compose.jetson.yml build"
echo "6. Run: docker-compose -f docker-compose.yml -f docker-compose.jetson.yml up"
