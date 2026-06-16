# Quick Start Guide

Get your ROS2 robot connected to The Platform in 5 minutes!

## 1. Install Dependencies

```bash
pip3 install -r requirements.txt
```

## 2. Build the Package

```bash
# Copy to your ROS2 workspace
cd ~/ros2_ws/src
ln -s /path/to/BurfPlatform/ros2_driver burf_platform_driver

# Build
cd ~/ros2_ws
colcon build --packages-select burf_platform_driver
source install/setup.bash
```

## 3. Configure Your Robot

Edit `config/burf_driver.yaml`:

```yaml
robot_id: "my-robot-01"  # Give your robot a unique name
camera_topic: "/camera/image_raw"  # Your camera topic
odom_topic: "/odom"  # Your odometry topic
imu_topic: "/imu/data"  # Your IMU topic (optional)
```

## 4. Run the Driver

```bash
ros2 launch burf_platform_driver burf_driver.launch.py
```

## 5. View in Web Browser

1. Open https://platform.burf.co
2. Login (if required)
3. Find your robot in the list
4. Click to view camera feed
5. Use joystick to control!

## Example: TurtleSim Test

```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Run driver
ros2 run burf_platform_driver burf_driver \
    --ros-args \
    -p robot_id:=turtlesim \
    -p cmd_vel_topic:=/turtle1/cmd_vel \
    -p camera_topic:=/fake_camera
```

Now control the turtle from the web interface!

## Troubleshooting

**"Failed to connect to server"**
- Check internet connection
- Verify server URL: `wss://platform.burf.co/ws/robot`
- Check firewall settings

**"No camera feed"**
- Verify camera topic: `ros2 topic list | grep camera`
- Check topic name in config matches your robot
- Look for errors in driver logs

**"Robot not moving"**
- Check /cmd_vel topic: `ros2 topic echo /cmd_vel`
- Verify your robot subscribes to /cmd_vel
- Test manually: `ros2 topic pub /cmd_vel geometry_msgs/Twist ...`

## Next Steps

- Customize movement speeds in `burf_driver.py`
- Adjust video quality/FPS in config
- Add additional sensors
- See full README.md for advanced configuration
