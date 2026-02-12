# TurtleBot 2 Upgrade Plan: TK1 → Jetson Nano + ROS2 Humble

> **Note**: This is a historical planning document from before the upgrade. The actual
> implementation kept the ASUS Xtion Pro camera (using openni2_camera driver) rather than
> switching to Orbbec Astra, and uses standard Ubuntu 22.04 `ros:humble-ros-base` as the
> Docker base image instead of Jetson-specific images. See README.md for current setup.

## Overview

Upgrade an existing TurtleBot 2 from:
- **Current**: Nvidia TK1 + ROS1 + Asus Xtion Pro
- **Target**: Jetson Nano + ROS2 Humble (via Docker)

### Hardware Retained
- Kobuki mobile base (Yujin Robot)
- TurtleBot 2 frame/structure
- ASUS Xtion Pro depth camera

### Hardware Changed
- **Compute**: TK1 → Jetson Nano (4GB)

---

## Phase 1: Jetson Nano Setup

### 1.1 Flash Jetson Nano
- Download and flash JetPack 4.6.x (last supported for Nano)
- This provides Ubuntu 18.04 with CUDA support
- Configure WiFi, SSH access

### 1.2 Install Docker + Container Tools
```bash
# Install jetson-containers tooling
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh
```

### 1.3 Build ROS2 Humble Container
```bash
# Build container with ROS2 Humble
jetson-containers build --name=turtlebot2_humble ros:humble-desktop
```

**Alternative**: Use pre-built image from dusty-nv:
```bash
jetson-containers run $(autotag ros:humble-ros-base)
```

### 1.4 USB Device Passthrough
Ensure Docker can access USB devices for Kobuki and Astra:
```bash
docker run --privileged \
  -v /dev:/dev \
  --device=/dev/ttyUSB0 \
  ...
```

---

## Phase 2: Kobuki Base Driver

### 2.1 Clone Required Repositories
Inside the Docker container, create a workspace:
```bash
mkdir -p ~/turtlebot2_ws/src
cd ~/turtlebot2_ws/src

# Kobuki driver and ROS2 wrapper
git clone https://github.com/kobuki-base/kobuki_ros.git
git clone https://github.com/kobuki-base/kobuki_core.git

# ECL dependencies (required)
git clone https://github.com/stonier/ecl_core.git
git clone https://github.com/stonier/ecl_lite.git

# Velocity smoother and mux
git clone https://github.com/kobuki-base/kobuki_velocity_smoother.git
git clone https://github.com/kobuki-base/cmd_vel_mux.git

# OR use the all-in-one community repo:
git clone https://github.com/idorobotics/turtlebot2_ros2.git
```

### 2.2 Install Dependencies
```bash
cd ~/turtlebot2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2.3 Build
```bash
colcon build --symlink-install
source install/setup.bash
```

### 2.4 Test Kobuki Connection
```bash
# Check USB connection (typically /dev/ttyUSB0)
ls -la /dev/ttyUSB*

# Launch Kobuki node
ros2 launch kobuki_node kobuki_node.launch.py
```

### 2.5 Verify Topics
```bash
ros2 topic list
# Should see:
#   /odom
#   /cmd_vel
#   /joint_states
#   /mobile_base/sensors/imu_data
#   /mobile_base/sensors/bumper_pointcloud
```

---

## Phase 3: Orbbec Astra Camera

### 3.1 Clone OrbbecSDK ROS2 Wrapper
```bash
cd ~/turtlebot2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2
git checkout v2-main  # or 'main' if Astra not in v2-main
```

### 3.2 Install udev Rules (Critical)
```bash
cd ~/turtlebot2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3.3 Build Camera Package
```bash
cd ~/turtlebot2_ws
colcon build --packages-select orbbec_camera
source install/setup.bash
```

### 3.4 Test Camera
```bash
ros2 launch orbbec_camera astra.launch.py
```

### 3.5 Verify Camera Topics
```bash
ros2 topic list
# Should see:
#   /camera/color/image_raw
#   /camera/depth/image_raw
#   /camera/depth/points
#   /camera/camera_info
```

---

## Phase 4: URDF and TF Setup

### 4.1 Get TurtleBot2 Description
```bash
cd ~/turtlebot2_ws/src

# Option A: From turtlebot2_ros2 community repo (if not already cloned)
git clone https://github.com/idorobotics/turtlebot2_ros2.git

# Option B: From official ROS2 demo
git clone https://github.com/ros2/turtlebot2_demo.git
```

### 4.2 Modify URDF for Orbbec Astra
The default URDF may reference Kinect or Xtion. Update the sensor mount:

File: `turtlebot_description/urdf/sensors/astra.urdf.xacro`
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:macro name="sensor_astra" params="parent">
    <joint name="camera_rgb_joint" type="fixed">
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      <parent link="${parent}"/>
      <child link="camera_rgb_frame"/>
    </joint>
    <link name="camera_rgb_frame"/>

    <joint name="camera_rgb_optical_joint" type="fixed">
      <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
      <parent link="camera_rgb_frame"/>
      <child link="camera_rgb_optical_frame"/>
    </joint>
    <link name="camera_rgb_optical_frame"/>

    <joint name="camera_depth_joint" type="fixed">
      <origin xyz="0.0 0.025 0.0" rpy="0 0 0"/>
      <parent link="camera_rgb_frame"/>
      <child link="camera_depth_frame"/>
    </joint>
    <link name="camera_depth_frame"/>

    <joint name="camera_depth_optical_joint" type="fixed">
      <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
      <parent link="camera_depth_frame"/>
      <child link="camera_depth_optical_frame"/>
    </joint>
    <link name="camera_depth_optical_frame"/>
  </xacro:macro>
</robot>
```

### 4.3 Build and Launch Robot State Publisher
```bash
colcon build --packages-select turtlebot_description
source install/setup.bash

ros2 launch turtlebot_description robot_state_publisher.launch.py
```

### 4.4 Verify TF Tree
```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf showing transform tree
```

Expected TF tree:
```
odom
  └── base_footprint
        └── base_link
              ├── wheel_left_link
              ├── wheel_right_link
              └── camera_rgb_frame
                    └── camera_rgb_optical_frame
                    └── camera_depth_frame
                          └── camera_depth_optical_frame
```

---

## Phase 5: Integration Bringup

### 5.1 Create Combined Launch File
File: `turtlebot2_bringup/launch/turtlebot2_bringup.launch.py`

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    kobuki_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('kobuki_node'),
            '/launch/kobuki_node.launch.py'
        ])
    )

    astra_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('orbbec_camera'),
            '/launch/astra.launch.py'
        ])
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('turtlebot_description'),
            '/launch/robot_state_publisher.launch.py'
        ])
    )

    return LaunchDescription([
        kobuki_launch,
        astra_launch,
        robot_state_publisher,
    ])
```

### 5.2 Test Full Bringup
```bash
ros2 launch turtlebot2_bringup turtlebot2_bringup.launch.py
```

### 5.3 Test Teleop
```bash
# In another terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Phase 6: Navigation (Nav2)

### 6.1 Install Nav2
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### 6.2 SLAM (Mapping)
```bash
sudo apt install ros-humble-slam-toolbox

# Launch SLAM
ros2 launch slam_toolbox online_async_launch.py
```

### 6.3 Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 6.4 Navigation with Saved Map
```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  map:=/path/to/my_map.yaml
```

---

## Docker Compose Setup (Optional)

For easier management, create `docker-compose.yml`:

```yaml
version: '3.8'
services:
  turtlebot2:
    image: turtlebot2_humble:latest
    privileged: true
    network_mode: host
    volumes:
      - /dev:/dev
      - ./turtlebot2_ws:/root/turtlebot2_ws
      - /tmp/.X11-unix:/tmp/.X11-unix
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
    command: ros2 launch turtlebot2_bringup turtlebot2_bringup.launch.py
```

---

## Troubleshooting

### Kobuki Not Detected
```bash
# Check USB
lsusb | grep -i "Yujin\|Kobuki\|FTDI"

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Add user to dialout group
sudo usermod -a -G dialout $USER
```

### Astra Not Detected
```bash
# Check USB
lsusb | grep -i "Orbbec"

# Re-run udev rules
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger

# Replug USB cable
```

### Performance Issues on Jetson Nano
- Reduce camera resolution in launch file
- Use `ros-base` instead of `ros-desktop` image
- Monitor with `jtop` (Jetson stats tool)
- Consider disabling GUI components

---

## Key Resources

- [kobuki-base/kobuki_ros](https://github.com/kobuki-base/kobuki_ros) - Kobuki ROS2 driver
- [idorobotics/turtlebot2_ros2](https://github.com/idorobotics/turtlebot2_ros2) - Community TurtleBot2 ROS2 support
- [orbbec/OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) - Astra camera driver
- [dusty-nv/jetson-containers](https://github.com/dusty-nv/jetson-containers) - Docker for Jetson
- [ros2/turtlebot2_demo](https://github.com/ros2/turtlebot2_demo) - Official ROS2 demo
- [ROS2 Essentials Kobuki](https://j3soon.github.io/ros2-essentials/kobuki-ws/) - Docker-based Kobuki setup

---

## Summary

| Component | Solution | Confidence |
|-----------|----------|------------|
| Compute | Jetson Nano + Docker + jetson-containers | High |
| Kobuki Base | kobuki_ros / turtlebot2_ros2 (build from source) | High |
| Orbbec Astra | OrbbecSDK_ROS2 (official support) | High |
| URDF | turtlebot2_ros2 / turtlebot2_demo (may need sensor swap) | Medium |
| Navigation | Nav2 + slam_toolbox | High |

**Estimated complexity**: Moderate - the main work is getting all packages built in the Docker environment and ensuring USB passthrough works correctly.
