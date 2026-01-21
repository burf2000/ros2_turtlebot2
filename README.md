# TurtleBot 2 - ROS2 Humble on Jetson Nano

This project provides a complete ROS2 Humble setup for TurtleBot 2 running on NVIDIA Jetson Nano via Docker.

## Hardware

- **Compute**: NVIDIA Jetson Nano (4GB)
- **Mobile Base**: Kobuki (Yujin Robot)
- **Camera**: Orbbec Astra
- **Previous**: Upgraded from NVIDIA TK1 + ROS1

## Quick Start

### 1. On the Jetson Nano

```bash
# Clone this repository
git clone <your-repo-url> ~/turtlebot2
cd ~/turtlebot2

# Run the Jetson setup script
./scripts/setup_jetson.sh

# Reboot or log out/in for Docker permissions
```

### 2. Build the Docker Image

```bash
# Build for Jetson
docker-compose -f docker-compose.yml -f docker-compose.jetson.yml build

# Or use the helper script
./scripts/run.sh build
```

### 3. Run the Robot

```bash
# Start interactive shell
./scripts/run.sh bash

# Or launch the full robot stack
./scripts/run.sh bringup
```

## Project Structure

```
turtlebot2/
├── docker/
│   ├── Dockerfile           # Full image with all dependencies
│   ├── Dockerfile.dev       # Development image (lighter)
│   └── ros_entrypoint.sh    # Container entrypoint
├── docker-compose.yml       # Main compose file
├── docker-compose.jetson.yml # Jetson-specific overrides
├── scripts/
│   ├── setup_jetson.sh      # Initial Jetson setup
│   ├── clone_dependencies.sh # Clone ROS2 packages
│   ├── build_workspace.sh   # Build ROS2 workspace
│   ├── run.sh               # Helper run script
│   └── save_map.sh          # Save SLAM maps
├── src/
│   ├── turtlebot2_description/  # URDF and meshes
│   │   ├── urdf/
│   │   │   ├── turtlebot2.urdf.xacro
│   │   │   ├── kobuki.urdf.xacro
│   │   │   └── sensors/astra.urdf.xacro
│   │   └── launch/
│   └── turtlebot2_bringup/      # Launch files and configs
│       ├── launch/
│       │   ├── turtlebot2.launch.py
│       │   ├── kobuki.launch.py
│       │   ├── astra.launch.py
│       │   ├── slam.launch.py
│       │   └── nav2.launch.py
│       └── config/
│           ├── kobuki.yaml
│           ├── astra.yaml
│           ├── slam_params.yaml
│           └── nav2_params.yaml
└── data/
    ├── maps/                # Saved maps
    └── logs/                # ROS logs
```

## Available Commands

Using the helper script `./scripts/run.sh`:

| Command | Description |
|---------|-------------|
| `bash` | Interactive shell in container |
| `bringup` | Launch full robot stack |
| `kobuki` | Launch Kobuki base only |
| `camera` | Launch Astra camera only |
| `teleop` | Keyboard teleoperation |
| `slam` | Start SLAM mapping |
| `nav` | Start Nav2 navigation |
| `build` | Build Docker image |
| `dev` | Development container |

## Usage Examples

### Teleoperation

```bash
# Terminal 1: Launch robot
./scripts/run.sh bringup

# Terminal 2: Teleop
./scripts/run.sh teleop
```

### SLAM Mapping

```bash
# Terminal 1: Launch robot
./scripts/run.sh bringup

# Terminal 2: Launch SLAM
./scripts/run.sh slam

# Terminal 3: Teleop to drive around
./scripts/run.sh teleop

# When done mapping:
./scripts/save_map.sh mymap
```

### Autonomous Navigation

```bash
# Launch robot with Nav2 (requires a saved map)
docker-compose run --rm turtlebot2 \
    ros2 launch turtlebot2_bringup nav2.launch.py map:=/root/maps/mymap.yaml
```

## Development

### Building Inside Container

```bash
# Enter development container
./scripts/run.sh dev

# Inside container:
source /opt/ros/humble/setup.bash
cd /root/turtlebot2_ws

# Clone dependencies if not already done
/root/turtlebot2_ws/src/local/scripts/clone_dependencies.sh

# Build
colcon build --symlink-install
source install/setup.bash
```

### Modifying URDF

The robot description is in `src/turtlebot2_description/urdf/`. After modifying:

```bash
# Rebuild description package
colcon build --packages-select turtlebot2_description

# Restart robot_state_publisher
ros2 launch turtlebot2_description robot_state_publisher.launch.py
```

### Viewing TF Tree

```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf
```

## Troubleshooting

### Kobuki Not Detected

```bash
# Check USB connection
lsusb | grep -i "FTDI"

# Check device
ls -la /dev/ttyUSB*

# Fix permissions
sudo chmod 666 /dev/ttyUSB0
```

### Astra Camera Issues

```bash
# Check USB
lsusb | grep -i "Orbbec"

# Reinstall udev rules
cd /root/turtlebot2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger

# Replug USB cable
```

### Performance on Jetson Nano

The original Jetson Nano may be slower. Tips:
- Reduce camera resolution in `config/astra.yaml`
- Use `ros-base` instead of `ros-desktop`
- Monitor with `jtop` (install: `sudo pip3 install jetson-stats`)
- Disable unused components

### Docker Permission Denied

```bash
sudo usermod -aG docker $USER
# Log out and back in
```

## Dependencies

The Dockerfile clones these repositories:

- [kobuki-base/kobuki_ros](https://github.com/kobuki-base/kobuki_ros)
- [kobuki-base/kobuki_core](https://github.com/kobuki-base/kobuki_core)
- [stonier/ecl_core](https://github.com/stonier/ecl_core)
- [stonier/ecl_lite](https://github.com/stonier/ecl_lite)
- [orbbec/OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2)
- [dusty-nv/jetson-containers](https://github.com/dusty-nv/jetson-containers)

## References

- [TurtleBot 2 on ROS2](https://github.com/idorobotics/turtlebot2_ros2)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Jetson Containers](https://github.com/dusty-nv/jetson-containers)
