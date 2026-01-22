# TurtleBot 2 - ROS2 Humble on Jetson Nano

This project provides a complete ROS2 Humble setup for TurtleBot 2 running on NVIDIA Jetson Nano via Docker.

## Hardware

- **Compute**: NVIDIA Jetson Nano (4GB)
- **Mobile Base**: Kobuki (Yujin Robot)
- **Camera**: ASUS Xtion Pro (OpenNI2)
- **Previous**: Upgraded from NVIDIA TK1 + ROS1

## Cold Start Guide

Follow these steps to run the robot from a completely powered-off state.

### Step 1: Hardware Setup

1. Connect the Kobuki base to the Jetson Nano via USB (FTDI cable)
2. Connect the ASUS Xtion Pro camera to a USB port
3. Power on the Kobuki base (green LED should light up)
4. Power on the Jetson Nano

### Step 2: SSH into Jetson Nano

From your development machine:

```bash
ssh burf2000@jetson.local
# Or use the IP address: ssh burf2000@<jetson-ip>
```

### Step 3: Verify USB Devices

```bash
# Check Kobuki is detected (FTDI USB serial)
lsusb | grep -i "0403:6001"

# Check Xtion camera is detected
lsusb | grep -i "1d27:0601"

# Should see both devices
ls /dev/ttyUSB*  # Kobuki serial port
```

### Step 4: Launch the Robot

```bash
cd ~/turtlebot2

# Run the full robot stack
docker run --rm -it --privileged --network host \
  -v /dev:/dev \
  -v ~/turtlebot2/src/turtlebot2_bringup:/root/turtlebot2_ws/src/turtlebot2_bringup \
  -v ~/turtlebot2/src/turtlebot2_description:/root/turtlebot2_ws/src/turtlebot2_description \
  turtlebot2_humble bash -c '
    source /opt/ros/humble/install/setup.bash && \
    source /root/turtlebot2_ws/install/setup.bash && \
    colcon build --symlink-install --packages-select turtlebot2_bringup && \
    source install/setup.bash && \
    ros2 launch turtlebot2_bringup turtlebot2.launch.py'
```

### Step 5: Verify Everything is Working

In a second SSH terminal:

```bash
# Enter the running container
docker exec -it $(docker ps -q) bash

# Source ROS2
source /opt/ros/humble/install/setup.bash
source /root/turtlebot2_ws/install/setup.bash

# List all topics
ros2 topic list

# Check odometry is publishing
ros2 topic hz /odom

# Check camera is streaming
ros2 topic hz /camera/depth/image_raw

# Check battery status
ros2 topic echo /sensors/battery_state --once

# Make the robot beep (confirms commands work)
ros2 topic pub --once /commands/sound kobuki_ros_interfaces/msg/Sound "{value: 0}"
```

## Quick Reference Commands

### Start Robot (Minimal)

```bash
docker run --rm -it --privileged --network host -v /dev:/dev \
  -v ~/turtlebot2/src/turtlebot2_bringup:/root/turtlebot2_ws/src/turtlebot2_bringup \
  turtlebot2_humble bash -c '
    source /opt/ros/humble/install/setup.bash && \
    source /root/turtlebot2_ws/install/setup.bash && \
    ros2 launch turtlebot2_bringup turtlebot2.launch.py'
```

### Kobuki Base Only

```bash
docker run --rm -it --privileged --network host -v /dev:/dev \
  turtlebot2_humble bash -c '
    source /opt/ros/humble/install/setup.bash && \
    source /root/turtlebot2_ws/install/setup.bash && \
    ros2 run kobuki_node kobuki_ros_node --ros-args -p device_port:=/dev/ttyUSB0'
```

### Camera Only

```bash
docker run --rm -it --privileged --network host -v /dev:/dev \
  turtlebot2_humble bash -c '
    source /opt/ros/humble/install/setup.bash && \
    source /root/turtlebot2_ws/install/setup.bash && \
    ros2 run openni2_camera openni2_camera_driver'
```

### Interactive Shell

```bash
docker run --rm -it --privileged --network host -v /dev:/dev \
  -v ~/turtlebot2/src/turtlebot2_bringup:/root/turtlebot2_ws/src/turtlebot2_bringup \
  -v ~/turtlebot2/src/turtlebot2_description:/root/turtlebot2_ws/src/turtlebot2_description \
  turtlebot2_humble bash
```

## ROS2 Topics

### Kobuki Base Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | nav_msgs/Odometry | Wheel odometry |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/joint_states` | sensor_msgs/JointState | Wheel joint states |
| `/tf` | tf2_msgs/TFMessage | Transform tree |
| `/sensors/battery_state` | sensor_msgs/BatteryState | Battery voltage/percentage |
| `/sensors/imu_data` | sensor_msgs/Imu | IMU data |
| `/sensors/core` | kobuki_ros_interfaces/SensorState | Raw sensor data |
| `/events/bumper` | kobuki_ros_interfaces/BumperEvent | Bumper events |
| `/events/cliff` | kobuki_ros_interfaces/CliffEvent | Cliff sensor events |
| `/commands/sound` | kobuki_ros_interfaces/Sound | Play sounds (0-6) |
| `/commands/led1` | kobuki_ros_interfaces/Led | LED 1 control |
| `/commands/led2` | kobuki_ros_interfaces/Led | LED 2 control |

### Camera Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/depth/image_raw` | sensor_msgs/Image | Depth image (640x480 @ 30fps) |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | Depth camera calibration |
| `/camera/rgb/image_raw` | sensor_msgs/Image | RGB image (640x480 @ 30fps) |
| `/camera/rgb/camera_info` | sensor_msgs/CameraInfo | RGB camera calibration |
| `/ir/image` | sensor_msgs/Image | Infrared image |

## Launch Arguments

The main launch file supports these arguments:

```bash
ros2 launch turtlebot2_bringup turtlebot2.launch.py \
  launch_kobuki:=true \
  launch_camera:=true \
  launch_robot_state_publisher:=false \
  use_sim_time:=false
```

| Argument | Default | Description |
|----------|---------|-------------|
| `launch_kobuki` | true | Launch Kobuki base driver |
| `launch_camera` | true | Launch Xtion camera driver |
| `launch_robot_state_publisher` | false | Launch robot URDF publisher |
| `use_sim_time` | false | Use simulation time |

## Project Structure

```
turtlebot2/
├── docker/
│   ├── Dockerfile           # Full image with all dependencies
│   └── ros_entrypoint.sh    # Container entrypoint
├── docker-compose.yml       # Main compose file
├── docker-compose.jetson.yml # Jetson-specific overrides
├── src/
│   ├── turtlebot2_description/  # URDF and meshes
│   │   ├── urdf/
│   │   │   ├── turtlebot2.urdf.xacro
│   │   │   ├── kobuki.urdf.xacro
│   │   │   └── sensors/xtion.urdf.xacro
│   │   └── launch/
│   │       └── robot_state_publisher.launch.py
│   └── turtlebot2_bringup/      # Launch files and configs
│       ├── launch/
│       │   └── turtlebot2.launch.py
│       └── config/
│           ├── kobuki.yaml      # Kobuki parameters
│           └── xtion.yaml       # Camera parameters
└── data/
    └── maps/                    # Saved maps
```

## Building the Docker Image

If you need to rebuild the Docker image:

```bash
cd ~/turtlebot2

# Build for Jetson Nano (takes ~1-2 hours)
docker build -t turtlebot2_humble \
  --build-arg BASE_IMAGE=dustynv/ros:humble-ros-base-l4t-r32.7.1 \
  -f docker/Dockerfile .
```

## Troubleshooting

### Kobuki Not Detected

```bash
# Check USB connection
lsusb | grep -i "FTDI"
# Should show: Future Technology Devices International

# Check device exists
ls -la /dev/ttyUSB*

# If permission denied:
sudo chmod 666 /dev/ttyUSB0
```

### Xtion Camera Issues

```bash
# Check USB connection
lsusb | grep -i "1d27"
# Should show: ASUS Xtion Pro

# Test camera detection inside container
ros2 run openni2_camera list_devices

# If "USB transfer timeout" errors occur, the standalone driver
# works better than ComposableNodeContainer (already configured)
```

### Container Can't Access USB Devices

Make sure to run with `--privileged` and `-v /dev:/dev`:

```bash
docker run --rm -it --privileged -v /dev:/dev turtlebot2_humble bash
```

### Check Running Topics

```bash
# List all active topics
ros2 topic list

# Check publishing rate
ros2 topic hz /odom
ros2 topic hz /camera/depth/image_raw

# View topic data
ros2 topic echo /sensors/battery_state --once
```

### View Camera Images (from remote machine)

On a machine with ROS2 and display:

```bash
# Make sure ROS_DOMAIN_ID matches
export ROS_DOMAIN_ID=0

# View depth image
ros2 run rqt_image_view rqt_image_view /camera/depth/image_raw

# View RGB image
ros2 run rqt_image_view rqt_image_view /camera/rgb/image_raw
```

## Dependencies

The Docker image includes:

- [kobuki-base/kobuki_ros](https://github.com/kobuki-base/kobuki_ros) - Kobuki ROS2 driver
- [kobuki-base/kobuki_core](https://github.com/kobuki-base/kobuki_core) - Kobuki core library
- [stonier/ecl_core](https://github.com/stonier/ecl_core) - ECL libraries
- [ros-drivers/openni2_camera](https://github.com/ros-drivers/openni2_camera) - OpenNI2 camera driver
- [dusty-nv/jetson-containers](https://github.com/dusty-nv/jetson-containers) - Base Jetson image

## References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Kobuki Documentation](https://kobuki.readthedocs.io/)
- [OpenNI2 Camera](https://github.com/ros-drivers/openni2_camera)
- [Jetson Containers](https://github.com/dusty-nv/jetson-containers)
