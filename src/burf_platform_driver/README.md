# BurfPlatform ROS2 Driver

ROS2 driver for integrating mobile robots with the BurfPlatform WebSocket/WebRTC server.

## Overview

This ROS2 package bridges standard ROS2 topics to the BurfPlatform server infrastructure, enabling:
- Camera streaming from ROS2 to web browser
- Odometry and IMU sensor data transmission
- Remote teleoperation via web interface joystick
- Real-time system monitoring

## Architecture

```
ROS2 Robot Topics              BurfPlatform Driver              WebSocket Server
─────────────────              ───────────────────              ────────────────
/camera/image_raw    ────>    JPEG Encoding        ────>      Video Stream
/odom                ────>    Telemetry            ────>      Position Data
/imu/data            ────>    Telemetry            ────>      IMU Data

Web Interface        <────    Movement Commands    <────      /cmd_vel
```

## Features

- **WebRTC Video Streaming**: Real-time camera streaming using WebRTC peer connections
- **Camera Telemetry**: Subscribes to ROS2 camera topics (Image or CompressedImage)
- **Sensor Telemetry**: Publishes odometry, IMU, and LaserScan data at configurable rate (default 5Hz)
- **Movement Control**: Receives movement commands from web interface and publishes to `/cmd_vel`
- **System Monitoring**: Reports CPU, memory, disk usage, and hostname
- **Auto-Reconnect**: Automatically reconnects to server with exponential backoff
- **Configurable**: All topics and parameters configurable via YAML or launch arguments

## Prerequisites

- ROS2 (Humble or Jazzy)
- Python 3.8+
- Required Python packages (see `requirements.txt`):
  - `websockets` - WebSocket client library
  - `opencv-python` - Image processing and encoding
  - `psutil` - System monitoring (CPU, memory, disk)
  - `aiortc` - WebRTC peer connection support
  - `av` - Video frame processing for WebRTC

## Installation

### 1. Install Python Dependencies

```bash
cd /path/to/ros2_driver
pip3 install -r requirements.txt
```

### 2. Build the Package

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Copy or symlink this package
ln -s /path/to/BurfPlatform/Drivers/ros2_driver burf_platform_driver

# Build
cd ~/ros2_ws
colcon build --packages-select burf_platform_driver

# Source the workspace
source install/setup.bash
```

## Configuration

Edit `config/burf_driver.yaml` to match your robot's topics and server configuration:

```yaml
/**:
  ros__parameters:
    # Server configuration
    server_url: "wss://platform.burf.co/ws/robot"
    robot_id: "ros2-rover-01"  # Unique ID for your robot

    # ROS2 topic names
    camera_topic: "/camera/image_raw"
    use_compressed: false  # Set to true for compressed images
    odom_topic: "/odom"
    imu_topic: "/imu/data"
    cmd_vel_topic: "/cmd_vel"

    # Performance tuning
    telemetry_rate: 5.0  # Hz
    video_fps: 10        # Target video FPS
```

## Usage

### Run with Launch File

```bash
ros2 launch burf_platform_driver burf_driver.launch.py
```

### Local Development

For local testing with a development server:

```bash
ros2 launch burf_platform_driver burf_driver.launch.py \
    local:=true \
    robot_id:=ros2-test-01
```

This automatically connects to `ws://192.168.0.50:8081/ws/robot` instead of the production server.

### Run with Custom Parameters

```bash
ros2 launch burf_platform_driver burf_driver.launch.py \
    server_url:=wss://platform.burf.co/ws/robot \
    robot_id:=my-robot-01
```

### Run Directly

```bash
ros2 run burf_platform_driver burf_driver \
    --ros-args \
    -p server_url:=wss://platform.burf.co/ws/robot \
    -p robot_id:=my-robot-01 \
    -p camera_topic:=/camera/image_raw \
    -p odom_topic:=/odom \
    -p imu_topic:=/imu/data

# Or for local development
ros2 run burf_platform_driver burf_driver \
    --ros-args \
    -p local:=true \
    -p robot_id:=ros2-test-01
```

## Topic Mapping

The driver expects the following standard ROS2 topics:

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` or `CompressedImage` | Camera feed (raw or compressed) |
| `/odom` | `nav_msgs/Odometry` | Robot odometry (position/velocity) |
| `/imu/data` | `sensor_msgs/Imu` | IMU data (orientation/acceleration) |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data (optional) |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (published by driver) |

If your robot uses different topic names, you can remap them:

```bash
ros2 run burf_platform_driver burf_driver \
    --ros-args \
    -r /camera/image_raw:=/my_robot/camera \
    -r /odom:=/my_robot/odom \
    -r /imu/data:=/my_robot/imu
```

## Server Protocol

The driver implements the BurfPlatform WebSocket protocol:

### Messages Sent to Server

**robot_hello** (on connection):
```json
{
  "type": "robot_hello",
  "timestamp": "2025-12-29T10:30:00.123Z",
  "payload": {
    "robotId": "ros2-rover-01",
    "sdkVersion": "1.0.0",
    "capabilities": ["camera", "movement", "odometry", "imu"]
  }
}
```

**telemetry** (periodic, 5Hz by default):
```json
{
  "type": "telemetry",
  "timestamp": "2025-12-29T10:30:00.123Z",
  "payload": {
    "robotId": "ros2-rover-01",
    "sensors": {
      "imu": {
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
        "angular_velocity": {"x": 0, "y": 0, "z": 0},
        "linear_acceleration": {"x": 0, "y": 0, "z": 9.81}
      },
      "odometry": {
        "position": {"x": 1.23, "y": 4.56, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
        "linear_velocity": {"x": 0.5, "y": 0, "z": 0},
        "angular_velocity": {"x": 0, "y": 0, "z": 0.1}
      }
    },
    "system": {
      "cpuUsage": 45.2,
      "memoryUsage": 62.1,
      "diskUsage": 38.5,
      "hostname": "ros2-rover"
    }
  }
}
```

### Messages Received from Server

**cmd_movement**:
```json
{
  "type": "cmd_movement",
  "payload": {
    "action": "forward"  // forward, backward, left, right, stop
  }
}
```

The driver translates these to `geometry_msgs/Twist` messages on `/cmd_vel`:
- `forward`: linear.x = 0.2 m/s
- `backward`: linear.x = -0.2 m/s
- `left`: angular.z = 0.4 rad/s
- `right`: angular.z = -0.4 rad/s
- `stop`: all zeros

Commands are published continuously at 10Hz until a new command is received.

## Map persistence & localization (SLAM robots)

When the driver runs with `enable_map_relay:=true` on a robot running
`slam_toolbox`, it advertises the `map_save` capability and can serialize the
live map to disk for reload in a later session.

**Save a map** — `cmd_map_save` (relayed from the server / UI):
```json
{ "type": "cmd_map_save", "payload": { "name": "ground-floor" } }
```
The driver calls `/slam_toolbox/serialize_map`, writing
`<map_storage_dir>/ground-floor.posegraph` + `.data` on the robot
(`map_storage_dir` defaults to `/home/burf2000/maps`). It replies with a
`map_save_result` (`ok` + map metadata, or `ok:false` + `error`). The server
persists the `SavedMap` row only on `ok:true`, so a failed serialize is never
recorded.

**Load a map (relocalize)** — fully server-driven, no relaunch and no CLI.
The server pushes the stored map bytes to the driver via `cmd_map_load`:
```json
{ "type": "cmd_map_load",
  "payload": { "name": "ground-floor",
               "posegraph_b64": "…", "data_b64": "…" } }
```
The driver writes `<map_storage_dir>/ground-floor.posegraph`/`.data` to disk,
then calls `/slam_toolbox/deserialize_map` with `match_type=LOCALIZE_AT_POSE`
on the **already-running** slam_toolbox node. slam_toolbox loads the graph,
switches to scan-match-only localization against it, and publishes `map→odom`
TF. It replies with a `map_load_result` (`ok`, or `ok:false` + `error`).

The driver stays a thin bridge — it never starts, stops, or relaunches SLAM;
SLAM runs as its own separate process. The driver's existing pose lookup, map
relay, and goto all work unchanged against the loaded map — including
`goto`-by-named-location (the server resolves a name to `{x, y, theta}` and
issues the normal `cmd_goal`).

> **Sensor-geometry note — all current SLAM robots run a 360° LiDAR.**
> Localization quality scales with the scan's angular FOV, and a 360° LiDAR
> (LD06/LD19: wheelchair, gwiz, **and the TurtleBot2**, now fitted with a real
> LD06) gives a full ring to correlate — so the robot can relocalize from
> anywhere in the mapped space, at any heading, with no need to park near the
> original mapping start pose. Because `deserialize_map` hands the saved
> posegraph to the already-running slam_toolbox node (no relaunch, no sensor
> restart), the live 360° scan immediately overlaps the saved graph and
> converges. (The old narrow-FOV caveat applied only to the TurtleBot's former
> Xtion depth→`depthimage_to_laserscan` ~58° "scan" — no longer used.)

**Named locations carry heading.** Each named spot stores `(x, y, theta)`;
`theta` is the final heading the robot rotates to on arrival. The driver applies
it on the last segment of the planned path (see `_goto_tick` /
`GoToController.set_goal`).

## Testing

### Test with TurtleSim

```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Run the driver (will fail on camera, but cmd_vel works)
ros2 run burf_platform_driver burf_driver \
    --ros-args \
    -p robot_id:=turtlesim-test \
    -p cmd_vel_topic:=/turtle1/cmd_vel

# Now control the turtle from the web interface!
```

### Test with Gazebo Simulation

```bash
# Launch your robot in Gazebo
ros2 launch my_robot_description gazebo.launch.py

# Run the driver
ros2 launch burf_platform_driver burf_driver.launch.py \
    robot_id:=gazebo-robot-01
```

## Performance Tuning

### Video Quality vs Bandwidth

Adjust JPEG quality in `burf_driver.py` line ~120:
```python
encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]  # 0-100
```

### Frame Rate

Lower values reduce CPU and network usage:
```yaml
video_fps: 5  # 5 FPS instead of 10
```

### Telemetry Rate

Reduce for lower bandwidth:
```yaml
telemetry_rate: 2.0  # 2 Hz instead of 5 Hz
```

## Troubleshooting

### No camera stream in web interface

1. Check that your camera topic is publishing:
   ```bash
   ros2 topic echo /camera/image_raw --no-arr
   ```

2. Verify topic name matches configuration

3. Check driver logs for errors:
   ```bash
   ros2 run burf_platform_driver burf_driver --ros-args --log-level debug
   ```

### Robot not responding to commands

1. Verify `/cmd_vel` topic:
   ```bash
   ros2 topic echo /cmd_vel
   ```

2. Check that your robot subscribes to `/cmd_vel`

3. Verify WebSocket connection in driver logs

### Connection issues

1. Test WebSocket URL:
   ```bash
   wscat -c wss://platform.burf.co/ws/robot
   ```

2. Check firewall/network settings

3. Verify robot_id is unique

## Future Enhancements

- [ ] Data channel for low-latency commands
- [ ] Support for multiple cameras
- [ ] Battery level reporting (via ROS2 battery_state topic)
- [ ] Map visualization (publish map to server)
- [ ] Recording/playback of robot sessions
- [ ] Support for depth cameras (PointCloud2)

## License

MIT

## Contributing

Issues and pull requests welcome at: https://github.com/yourusername/BurfPlatform
