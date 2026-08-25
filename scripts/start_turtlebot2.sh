#!/bin/bash
# Wait for Docker to be ready
echo "Waiting for Docker..."
while ! docker info > /dev/null 2>&1; do sleep 2; done
# Wait for USB and network to settle after boot
sleep 15
# Power cycle the USB hub to reset Xtion depth sensor (briefly drops WiFi - same hub)
echo "Power cycling USB hub for Xtion depth sensor..."
echo 0 > /sys/bus/usb/devices/1-2/authorized
sleep 5
echo 1 > /sys/bus/usb/devices/1-2/authorized
sleep 15
# Unbind audio driver from Xtion
echo "Unbinding audio driver..."
/usr/local/bin/xtion-unbind-audio.sh
sleep 3
# Remove old container if exists
docker stop turtlebot2_bringup 2>/dev/null || true
docker rm turtlebot2_bringup 2>/dev/null || true
echo "Starting TurtleBot2 container..."
docker run -d --name turtlebot2_bringup \
    --restart unless-stopped \
    --privileged \
    --network=host \
    -v /dev:/dev \
    -v /home/burf2000/turtlebot2/src/burf_platform_driver:/root/turtlebot2_ws/src/local/burf_platform_driver \
    turtlebot2_humble:ubuntu22 \
    bash -c "sleep infinity"
sleep 5
# Bringup. launch_lidar:=false because the ldlidar_stl_ros2 SDK is unreliable on
# this box (PL2303/latency) - the real /scan comes from the custom LD06 node below.
if [ -f /home/burf2000/turtlebot2/config/nav2_params.yaml ]; then
    echo "Installing host Nav2 params..."
    docker cp /home/burf2000/turtlebot2/config/nav2_params.yaml turtlebot2_bringup:/root/turtlebot2_ws/src/local/turtlebot2_bringup/config/nav2_params.yaml
fi

echo "Starting TurtleBot2 launch..."
docker exec -d turtlebot2_bringup bash -c \
    "source /opt/ros/humble/setup.bash && \
     source /root/turtlebot2_ws/install/setup.bash && \
     ros2 launch turtlebot2_bringup turtlebot2.launch.py launch_robot_state_publisher:=true launch_lidar:=false"
sleep 8
# Custom LD06 -> /scan node (raw FTDI read; see ld06_scan_node.py header).
echo "Starting custom LD06 LiDAR node..."
docker cp /home/burf2000/ld06_scan_node.py turtlebot2_bringup:/tmp/ld06_scan_node.py
docker exec -d turtlebot2_bringup bash -c \
    "source /opt/ros/humble/setup.bash && python3 /tmp/ld06_scan_node.py > /tmp/ld06.log 2>&1"
# Static mount TF base_footprint -> lidar_link. The launch's own lidar_static_tf is
# gated on launch_lidar (=false here), so publish it here or slam_toolbox drops
# every scan ("message filter queue full") and no map builds.
# z=0.35 is a PLACEHOLDER - measure the real LD06 mount height.
echo "Publishing LD06 static TF (base_footprint -> lidar_link)..."
docker exec -d turtlebot2_bringup bash -c \
    "source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.35 --yaw 0 --pitch 0 --roll 0 --frame-id base_footprint --child-frame-id lidar_link > /tmp/lidartf.log 2>&1"
# Kobuki auto-docking. The package was built in the image all along but never
# launched, so /auto_docking_action did not exist and the robot could not put
# itself on charge. It is a plain action server: idle until the driver sends it
# a goal, so running it always costs nothing.
# THE REMAP IS LOAD-BEARING. kobuki_auto_docking publishes its motor commands
# to /commands/velocity (the ROS1 name), but kobuki_node subscribes to /cmd_vel.
# Without -r the docking state machine runs perfectly and reports ALIGNED_NEAR
# while the robot never moves, because its velocity commands go to a topic with
# no subscriber. Symptom is "docking never completes and never errors".
echo "Starting Kobuki auto-docking action server..."
docker exec -d turtlebot2_bringup bash -c \
    "source /opt/ros/humble/setup.bash && source /root/turtlebot2_ws/install/setup.bash && exec ros2 run kobuki_auto_docking kobuki_auto_docking_node --ros-args -r /commands/velocity:=/cmd_vel > /tmp/autodock.log 2>&1"

echo "TurtleBot2 started successfully"
