# Deploying to pi4-ros Robot

Step-by-step guide for deploying the ROS2 driver to your robot.

## Robot Details

- **Hostname:** `pi4-ros.local`
- **Username:** `burf2000`
- **Password:** ``
- **ROS2 Workspace:** `~/robot_ws`

## Step 1: Copy Package to Robot

From your development machine (Mac):

```bash
# Copy the ROS2 driver to the robot
scp -r /Users/snrb/Documents/Git/BurfPlatform/ros2_driver \
    burf2000@pi4-ros.local:~/robot_ws/src/burf_platform_driver
```

When prompted, enter password: ``

## Step 2: SSH into Robot

```bash
ssh burf2000@pi4-ros.local
# Password: 
```

## Step 3: Install Dependencies

On the robot:

```bash
cd ~/robot_ws/src/burf_platform_driver
pip3 install -r requirements.txt
```

This installs:
- `websockets` - WebSocket client library
- `opencv-python` - Image processing and encoding
- `psutil` - System monitoring (CPU, memory, disk)
- `aiortc` - WebRTC peer connection support
- `av` - Video frame processing for WebRTC

## Step 4: Build the Package

```bash
cd ~/robot_ws
colcon build --packages-select burf_platform_driver
```

You should see:
```
Starting >>> burf_platform_driver
Finished <<< burf_platform_driver [X.XXs]

Summary: 1 package finished
```

## Step 5: Source the Workspace

```bash
source ~/robot_ws/install/setup.bash
```

**Important:** Add this to your `~/.bashrc` so it's automatic:

```bash
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
```

## Step 6: Configure for Your Robot

Edit the configuration file to match your robot's topics:

```bash
nano ~/robot_ws/src/burf_platform_driver/config/burf_driver.yaml
```

Update the topic names to match your robot:

```yaml
/**:
  ros__parameters:
    server_url: "wss://platform.burf.co/ws/robot"
    robot_id: "ros2-pi4-01"  # Unique ID for this robot

    # Update these topic names to match YOUR robot
    camera_topic: "/camera/image_raw"  # Check with: ros2 topic list
    odom_topic: "/odom"
    imu_topic: "/imu/data"
    cmd_vel_topic: "/cmd_vel"

    use_compressed: false  # Set true if using /camera/image_raw/compressed
    telemetry_rate: 5.0    # Hz
    video_fps: 10          # Target FPS
```

### Find Your Robot's Topics

```bash
# List all topics on your robot
ros2 topic list

# Check if topics are publishing
ros2 topic hz /camera/image_raw
ros2 topic hz /odom
ros2 topic hz /imu/data

# See topic details
ros2 topic info /camera/image_raw
```

## Step 7: Run the Driver

### Production Run (Cloud Server)

```bash
ros2 launch burf_platform_driver burf_driver.launch.py \
    robot_id:=ros2-pi4-01
```

### Local Testing

For local development/testing with your local server:

```bash
ros2 launch burf_platform_driver burf_driver.launch.py \
    local:=true \
    robot_id:=ros2-pi4-test
```

This automatically connects to `ws://192.168.0.50:8081/ws/robot`.

Alternatively, you can specify a custom server URL:

```bash
ros2 launch burf_platform_driver burf_driver.launch.py \
    robot_id:=ros2-pi4-test \
    server_url:=ws://192.168.0.50:8081/ws/robot
```

### Run with Custom Parameters

```bash
ros2 run burf_platform_driver burf_driver \
    --ros-args \
    -p robot_id:=ros2-pi4-01 \
    -p server_url:=wss://platform.burf.co/ws/robot \
    -p camera_topic:=/camera/rgb/image_raw \
    -p odom_topic:=/odom \
    -p imu_topic:=/imu/data
```

## Step 8: Verify It's Working

You should see output like:

```
[INFO] [burf_driver]: The Platform Driver initialized
[INFO] [burf_driver]: Robot ID: ros2-pi4-01
[INFO] [burf_driver]: Server URL: wss://platform.burf.co/ws/robot
[INFO] [burf_driver]: Connecting to wss://platform.burf.co/ws/robot...
[INFO] [burf_driver]: WebSocket connected!
[INFO] [burf_driver]: Sent robot_hello for ros2-pi4-01
[INFO] [burf_driver]: Received hello_ack from server
```

## Step 9: Test from Web Interface

1. Open browser: `https://platform.burf.co`
2. Login (if required)
3. Find your robot: `ros2-pi4-01`
4. Click to view camera feed
5. Use joystick to control movement

## Auto-Start on Boot (Optional)

To automatically start the driver when the robot boots:

### Create systemd service:

```bash
sudo nano /etc/systemd/system/burf-driver.service
```

Add:

```ini
[Unit]
Description=The Platform ROS2 Driver
After=network.target

[Service]
Type=simple
User=burf2000
WorkingDirectory=/home/burf2000
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/burf2000/robot_ws/install/setup.bash && ros2 launch burf_platform_driver burf_driver.launch.py robot_id:=ros2-pi4-01'
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable burf-driver.service
sudo systemctl start burf-driver.service

# Check status
sudo systemctl status burf-driver.service

# View logs
journalctl -u burf-driver.service -f
```

## Updating the Driver

When you make changes to the driver on your development machine:

```bash
# On development machine: Copy updated files
scp -r /Users/snrb/Documents/Git/BurfPlatform/ros2_driver/burf_platform_driver/*.py \
    burf2000@pi4-ros.local:~/robot_ws/src/burf_platform_driver/burf_platform_driver/

# SSH to robot
ssh burf2000@pi4-ros.local

# Rebuild
cd ~/robot_ws
colcon build --packages-select burf_platform_driver
source install/setup.bash

# Restart service (if using systemd)
sudo systemctl restart burf-driver.service
```

## Troubleshooting

### Cannot connect to server

```bash
# Test WebSocket connection
python3 -c "import websockets, asyncio; asyncio.run(websockets.connect('wss://platform.burf.co/ws/robot'))"
```

### Topics not found

```bash
# List all available topics
ros2 topic list

# Check topic type
ros2 topic info /camera/image_raw

# Update config.yaml with correct topic names
```

### Permission denied

```bash
# Make sure you're in the burf2000 user
whoami  # Should output: burf2000

# Check file permissions
ls -la ~/robot_ws/src/burf_platform_driver/
```

### Package not found after build

```bash
# Source the workspace
source ~/robot_ws/install/setup.bash

# Verify package is installed
ros2 pkg list | grep burf
```

## Quick Reference

### SSH to Robot
```bash
ssh burf2000@pi4-ros.local
# Password: 
```

### Rebuild Package
```bash
cd ~/robot_ws
colcon build --packages-select burf_platform_driver
source install/setup.bash
```

### Run Driver
```bash
ros2 launch burf_platform_driver burf_driver.launch.py robot_id:=ros2-pi4-01
```

### Stop Driver
```bash
# If running in terminal: Ctrl+C
# If running as service:
sudo systemctl stop burf-driver.service
```

### View Logs (systemd)
```bash
journalctl -u burf-driver.service -f
```

### Check Topics
```bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic echo /cmd_vel
```
