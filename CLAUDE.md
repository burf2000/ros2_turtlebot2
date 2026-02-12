# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Humble TurtleBot 2 stack for NVIDIA Jetson Nano. Runs entirely in Docker using standard Ubuntu 22.04 (no GPU acceleration needed). Hardware: Kobuki mobile base + ASUS Xtion Pro depth camera (OpenNI2).

## Common Commands

All commands use the helper script from the project root:

```bash
# Build Docker image (auto-detects Jetson vs non-Jetson)
./scripts/run.sh build

# Interactive shell in container
./scripts/run.sh bash

# Launch full robot (Kobuki + Xtion + TF + laser scan + point cloud)
./scripts/run.sh bringup

# Development container (lighter weight)
./scripts/run.sh dev
```

### Inside Container

```bash
# Source the workspace
source /opt/ros/humble/setup.bash
source /root/turtlebot2_ws/install/setup.bash

# Build local packages only
colcon build --symlink-install --packages-select turtlebot2_description turtlebot2_bringup

# Build all packages from source
colcon build --symlink-install

# View TF tree
ros2 run tf2_tools view_frames
```

## Architecture

### Docker Setup
- `docker/Dockerfile` - Full image with all dependencies (Kobuki, ECL, openni2_camera, depthimage_to_laserscan)
- `docker/Dockerfile.dev` - Lighter development image
- `docker-compose.yml` - Main compose file with service definitions
- `docker-compose.jetson.yml` - Jetson-specific overrides (same base image, auto-detected by run.sh)

The container mounts `./src` to `/root/turtlebot2_ws/src/local` for live development.

### ROS2 Packages

**turtlebot2_description** - Robot model
- `urdf/turtlebot2.urdf.xacro` - Main robot description, includes Kobuki base and Xtion camera
- `urdf/kobuki.urdf.xacro` - Kobuki mobile base geometry and joints
- `urdf/sensors/xtion.urdf.xacro` - ASUS Xtion Pro depth camera frames

**turtlebot2_bringup** - Launch files and configs
- `launch/turtlebot2.launch.py` - Main bringup (Kobuki + Xtion + robot_state_publisher + laser scan + point cloud)
- `launch/kobuki.launch.py` - Kobuki base only
- `launch/astra.launch.py` - Camera only
- `launch/desktop.launch.py` - RViz + teleop (desktop side)
- `launch/slam.launch.py` - SLAM Toolbox integration
- `launch/nav2.launch.py` - Nav2 navigation stack
- `config/` - YAML configs for each component

### Launch Arguments

The main launch file (`turtlebot2.launch.py`) accepts:
- `use_sim_time` (default: false) - Use simulation clock
- `launch_kobuki` (default: true) - Enable Kobuki base driver
- `launch_camera` (default: true) - Enable Xtion camera
- `launch_robot_state_publisher` (default: true) - Enable URDF publisher

### TF Frame Hierarchy

```
odom
  └── base_footprint
        └── base_link
              ├── wheel_left_link
              ├── wheel_right_link
              └── plate_bottom_link
                    └── ... → plate_top_link
                                 └── camera_mount_link
                                       └── camera_link (Xtion)
```

### External Dependencies (cloned in Dockerfile)

- kobuki_core, kobuki_ros - Kobuki driver (built from source)
- ecl_core, ecl_lite, ecl_tools - ECL utilities required by Kobuki (built from source)
- velocity_smoother, cmd_vel_mux - Kobuki utilities (built from source)

### Pre-built Dependencies (installed via apt)

- openni2_camera - ASUS Xtion Pro camera driver
- depthimage_to_laserscan - Converts depth image to virtual `/scan` topic
- depth_image_proc - Point cloud generation from depth + RGB

## Hardware Notes

- Kobuki connects via `/dev/ttyUSB0` (FTDI serial)
- Xtion Pro requires udev rules for USB permissions
- On Ubuntu 22.04, the `snd-usb-audio` kernel driver may conflict with the Xtion - see troubleshooting in README
- Container runs privileged with full `/dev` access for hardware
