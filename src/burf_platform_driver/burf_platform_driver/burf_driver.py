#!/usr/bin/env python3
"""
BurfPlatform ROS2 Driver

This node bridges ROS2 topics to the BurfPlatform WebSocket/WebRTC server.
It subscribes to camera, odometry, and IMU topics from a ROS2 robot and
streams the data to the server. It also receives movement commands from
the server and publishes them to ROS2 cmd_vel topic.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CompressedImage, LaserScan, BatteryState
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import tf2_ros
import base64
import io
import time as _time
import zlib
import asyncio
import websockets
import json
from datetime import datetime
import psutil
import socket
import threading
import numpy as np

from .goto_controller import GoToController
from .path_planner import GridPathPlanner, GridInfo

# Pillow is used to PNG-encode the occupancy grid before relaying it over
# WebSocket. Optional so the driver still launches on hosts where map relay
# isn't needed and Pillow isn't installed.
try:
    from PIL import Image as PILImage
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False
    print("Warning: Pillow not available, map relay disabled")

# Optional dependencies for camera/WebRTC (may not be available on all systems)
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("Warning: cv2 not available, camera features disabled")

try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
    from aiortc.sdp import candidate_from_sdp
    from av import VideoFrame
    WEBRTC_AVAILABLE = True
except ImportError:
    WEBRTC_AVAILABLE = False
    RTCPeerConnection = None
    VideoStreamTrack = object  # Dummy base class
    print("Warning: aiortc/av not available, WebRTC disabled")


# Only define WebRTC video track if dependencies are available
if WEBRTC_AVAILABLE:
    class ROS2VideoTrack(VideoStreamTrack):
        """
        A video track that streams frames from ROS2 camera topic.
        """
        def __init__(self, logger):
            super().__init__()
            self.logger = logger
            self.latest_frame = None
            self.frame_lock = threading.Lock()
            self.frame_count = 0

        def update_frame(self, cv_image):
            """Update the latest frame from ROS2 camera callback"""
            with self.frame_lock:
                self.latest_frame = cv_image
                self.frame_count += 1

        async def recv(self):
            """Generate video frames for WebRTC"""
            pts, time_base = await self.next_timestamp()

            with self.frame_lock:
                if self.latest_frame is None:
                    # Create a black frame if no image available
                    frame_array = np.zeros((480, 640, 3), dtype=np.uint8)
                else:
                    frame_array = self.latest_frame.copy()

            # Convert BGR to RGB (OpenCV uses BGR, WebRTC expects RGB)
            frame_rgb = cv2.cvtColor(frame_array, cv2.COLOR_BGR2RGB)

            # Create VideoFrame from numpy array
            frame = VideoFrame.from_ndarray(frame_rgb, format='rgb24')
            frame.pts = pts
            frame.time_base = time_base

            return frame
else:
    ROS2VideoTrack = None


class BurfPlatformDriver(Node):
    def __init__(self):
        super().__init__('burf_platform_driver')

        # Declare parameters
        self.declare_parameter('server_url', 'wss://platform.burf.co/ws/robot')
        self.declare_parameter('local', False)  # Use local development server
        self.declare_parameter('robot_id', 'ros2-rover-01')
        self.declare_parameter('api_key', '')  # API key for robot authentication
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('use_compressed', False)  # Set to True for /camera/image_raw/compressed
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('map_topic', '/map')
        # Explicit gate — caller (launch file) sets this true only when SLAM
        # is actually running. Auto-detecting /map availability at startup is
        # racy because slam_toolbox can take several seconds to publish the
        # first latched message after boot.
        self.declare_parameter('enable_map_relay', False)
        # Waypoint navigation. Same explicit-gate philosophy: only enable when
        # the robot is actually capable of being driven autonomously (real
        # /cmd_vel + a usable map->base_link TF). Browser hides the goto UI
        # when this capability isn't advertised in robot_hello.
        self.declare_parameter('enable_goto', False)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('telemetry_rate', 5.0)  # Hz
        self.declare_parameter('video_fps', 10)  # Target FPS for video streaming
        self.declare_parameter('depth_fps', 5)  # Target FPS for depth streaming

        # Get parameters
        use_local = self.get_parameter('local').value
        self.server_url = 'ws://192.168.0.50:8081/ws/robot' if use_local else self.get_parameter('server_url').value
        self.robot_id = self.get_parameter('robot_id').value
        self.api_key = self.get_parameter('api_key').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.battery_topic = self.get_parameter('battery_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.enable_map_relay = self.get_parameter('enable_map_relay').value
        self.enable_goto = self.get_parameter('enable_goto').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.telemetry_rate = self.get_parameter('telemetry_rate').value
        self.video_fps = self.get_parameter('video_fps').value
        self.depth_fps = self.get_parameter('depth_fps').value

        # WebSocket connection
        self.websocket = None
        self.ws_connected = False

        # WebRTC peer connection and video track (if available)
        self.pc = None
        self.depth_channel = None
        self.video_track = ROS2VideoTrack(self.get_logger()) if WEBRTC_AVAILABLE else None

        # Latest sensor data
        self.latest_odom = None
        self.latest_imu = None
        self.latest_scan = None
        self.latest_battery = None

        # Frame rate limiting
        self.last_frame_time = 0
        self.frame_interval = 1.0 / self.video_fps
        self.last_depth_time = 0
        self.depth_interval = 1.0 / self.depth_fps

        # ROS2 Subscribers
        if self.use_compressed:
            self.camera_sub = self.create_subscription(
                CompressedImage,
                self.camera_topic,
                self.camera_callback,
                10
            )
        else:
            self.camera_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.camera_callback,
                10
            )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        # Depth camera subscription (16UC1 depth images)
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        # Battery state subscription (optional — not all robots publish this)
        self.battery_sub = self.create_subscription(
            BatteryState,
            self.battery_topic,
            self.battery_callback,
            10
        )

        # OccupancyGrid map subscription (only when caller has set enable_map_relay).
        # slam_toolbox publishes /map with TRANSIENT_LOCAL durability — match it
        # or no message is ever delivered to this subscriber.
        self.map_sub = None
        if self.enable_map_relay:
            if not PIL_AVAILABLE:
                self.get_logger().error(
                    'enable_map_relay=true but Pillow is not installed; map relay disabled'
                )
            else:
                map_qos = QoSProfile(
                    depth=1,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                )
                self.map_sub = self.create_subscription(
                    OccupancyGrid, self.map_topic, self.map_callback, map_qos
                )
                self.get_logger().info(
                    f'Map relay enabled — subscribing to {self.map_topic}'
                )

        # TF buffer — needed for pose_in_map lookups (telemetry) and goto
        # navigation. Always created; lookup_transform fails fast when the
        # tree isn't there, which is fine.
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # GoToController — pure-Python single-waypoint state machine. Only
        # active (ticking) when enable_goto is set; otherwise sits idle and
        # ignores any incoming cmd_goal messages.
        self.goto = GoToController()
        self._goto_active_goal_id = None    # remembered to avoid spamming goal_status
        self._goto_last_state = None        # for state-transition detection
        # Grid path planner for obstacle-aware nav (homegrown A* on the SLAM
        # OccupancyGrid). Cached map grid + info filled in by map_callback.
        self.planner = GridPathPlanner(robot_radius_m=0.18)
        self.latest_map_grid = None         # numpy int8 grid, NOT flipped
        self.latest_map_info = None         # GridInfo
        self._path_waypoints = []           # remaining (x, y) corners
        self._path_index = 0                # index of the *current* segment goal
        self._final_goal_xy = None          # for status reporting
        if self.enable_goto:
            # 10 Hz tick — same cadence as the manual cmd_publish_callback.
            self.goto_timer = self.create_timer(0.1, self._goto_tick)
            self.get_logger().info(
                f'GoTo enabled — using TF {self.map_frame} -> {self.base_frame} for pose'
            )

        # ROS2 Publisher for cmd_vel (using Twist for broader compatibility)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Current movement command (for continuous publishing)
        self.current_cmd = Twist()
        self.cmd_lock = threading.Lock()

        # Telemetry timer
        self.telemetry_timer = self.create_timer(
            1.0 / self.telemetry_rate,
            self.telemetry_callback
        )

        # Movement command timer (publish at 10Hz for smooth control)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_callback)

        self.get_logger().info(f'BurfPlatform Driver initialized')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info(f'Server URL: {self.server_url}')

        # Start WebSocket connection in background thread
        self.ws_thread = threading.Thread(target=self.run_websocket_loop, daemon=True)
        self.ws_thread.start()

    def camera_callback(self, msg):
        """Handle incoming camera images"""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Rate limiting
        if current_time - self.last_frame_time < self.frame_interval:
            return

        self.last_frame_time = current_time

        if not CV2_AVAILABLE:
            self.get_logger().warning('cv2 not available, skipping raw image', throttle_duration_sec=10.0)
            return

        if self.video_track is None:
            return

        try:
            if self.use_compressed:
                # Decode compressed JPEG to OpenCV format for WebRTC
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                cv_image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            else:
                # Convert ROS Image to OpenCV format
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))

                if msg.encoding == 'rgb8':
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                elif msg.encoding == 'rgba8':
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
                elif msg.encoding == 'mono8':
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

            self.video_track.update_frame(cv_image)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def odom_callback(self, msg):
        """Handle incoming odometry data"""
        self.latest_odom = msg

    def imu_callback(self, msg):
        """Handle incoming IMU data"""
        self.latest_imu = msg

    def scan_callback(self, msg):
        """Handle incoming LaserScan data"""
        self.latest_scan = msg
        self.get_logger().info(f'Received scan with {len(msg.ranges)} ranges', throttle_duration_sec=5.0)

    def depth_callback(self, msg):
        """Handle incoming depth images — send via WebRTC DataChannel"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_depth_time < self.depth_interval:
            return
        self.last_depth_time = current_time

        if self.depth_channel is None or self.depth_channel.readyState != 'open':
            return

        try:
            # Depth image is 16UC1 (16-bit unsigned, 1 channel) — values in mm
            # Convert to raw bytes and zlib compress (same format as Sanbot)
            depth_bytes = bytes(msg.data)
            compressed = zlib.compress(depth_bytes, 1)  # Fast compression

            self.depth_channel.send(compressed)
            self.get_logger().debug(f'Sent depth frame: {len(compressed)} bytes ({msg.width}x{msg.height})', throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f'Failed to send depth frame: {e}', throttle_duration_sec=5.0)

    def battery_callback(self, msg):
        """Handle incoming battery state"""
        self.latest_battery = msg

    def _get_pose_in_map(self):
        """Look up the latest map -> base_link transform and return (x, y, yaw).
        Returns None when the TF tree isn't available yet (no SLAM, no fix)."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except Exception:
            return None
        tx = float(t.transform.translation.x)
        ty = float(t.transform.translation.y)
        qz = float(t.transform.rotation.z)
        qw = float(t.transform.rotation.w)
        yaw = 2.0 * math.atan2(qz, qw)
        return (tx, ty, yaw)

    def _emit_goal_status(self, status: dict, force: bool = False) -> None:
        """Send goal_status to the server. Only emits on state transitions
        unless force=True (so we don't spam the WebSocket every 100ms)."""
        new_state = status.get('state')
        if not force and new_state == self._goto_last_state:
            # No state change since the last emit.
            if new_state in (GoToController.ROTATING, GoToController.DRIVING):
                # Steady-state motion: throttle to once per second so the
                # browser still sees distance_remaining progress.
                now = _time.monotonic()
                last = getattr(self, '_goto_last_emit', 0.0)
                if now - last < 1.0:
                    return
                self._goto_last_emit = now
            else:
                # Terminal/idle states (arrived/failed/cancelled/idle): emit
                # exactly once on the transition into the state, never
                # re-broadcast. Re-broadcasting 'arrived' at 10 Hz while parked
                # at a waypoint was making the browser skip route waypoints
                # (each repeat advanced the route; stale repeats from a prior
                # run dropped the first waypoint of the next run).
                return
        self._goto_last_state = new_state
        msg = {
            'type': 'goal_status',
            'timestamp': self.get_timestamp(),
            'payload': {
                'robotId': self.robot_id,
                'state': new_state,
                'distance_remaining': status.get('distance_remaining'),
                'goal_id': status.get('goal_id'),
            }
        }
        loop = getattr(self, 'ws_loop', None)
        if loop is not None:
            asyncio.run_coroutine_threadsafe(self.send_message(msg), loop)

    def _goto_tick(self):
        """10 Hz tick — feed the latest pose into GoToController and publish
        any resulting Twist on /cmd_vel. Emits goal_status on transitions.

        When a multi-segment plan is in flight, intermediate ARRIVED events
        silently advance to the next waypoint instead of emitting; the
        browser only sees a final ARRIVED when the full plan is done."""
        if not self.goto.is_active() and self.goto.state == GoToController.IDLE:
            return
        pose = self._get_pose_in_map()
        if pose is None:
            return
        twist_tuple, status = self.goto.tick(pose, _time.monotonic())
        if twist_tuple is not None:
            lin, ang = twist_tuple
            t = Twist()
            t.linear.x = float(lin)
            t.angular.z = float(ang)
            self.cmd_vel_pub.publish(t)

        state = status.get('state')

        # If this segment arrived but there are more waypoints, advance
        # silently to the next one.
        if state == GoToController.ARRIVED and self._path_waypoints:
            self._path_index += 1
            if self._path_index < len(self._path_waypoints):
                nx, ny = self._path_waypoints[self._path_index]
                self.goto.set_goal(nx, ny, None)
                self.get_logger().info(
                    f'Segment {self._path_index}/{len(self._path_waypoints)} -> '
                    f'({nx:.2f}, {ny:.2f})'
                )
                return  # don't emit ARRIVED; next tick will emit ROTATING
            # Final segment completed
            self._path_waypoints = []
            self._path_index = 0
            self._final_goal_xy = None

        if state != self._goto_last_state:
            self._emit_goal_status(status, force=True)
        else:
            self._emit_goal_status(status)

    def map_callback(self, msg):
        """Relay an OccupancyGrid as a PNG-encoded map_update message.

        Layout sent to clients:
            unknown (-1)  -> grey  (128)
            free    (0)   -> white (255)
            occupied(>=50)-> black (0)

        We flip the image vertically because ROS publishes the grid with
        +y up but image canvases use +y down; doing it server-side once
        keeps the browser code simple. The browser positions/sizes the
        layer using resolution, originX, originY (world-frame metres).
        """
        try:
            w = msg.info.width
            h = msg.info.height
            if w == 0 or h == 0:
                return
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))
            # Cache the raw grid (NOT flipped) for the path planner.
            self.latest_map_grid = data
            self.latest_map_info = GridInfo(
                width=w, height=h,
                resolution=float(msg.info.resolution),
                origin_x=float(msg.info.origin.position.x),
                origin_y=float(msg.info.origin.position.y),
            )
            img = np.full((h, w), 128, dtype=np.uint8)
            img[data == 0] = 255
            img[data >= 50] = 0
            img = np.flipud(img)

            buf = io.BytesIO()
            PILImage.fromarray(img, mode='L').save(buf, format='PNG', optimize=True)
            png_b64 = base64.b64encode(buf.getvalue()).decode('ascii')

            # Yaw of the map frame's origin (almost always 0 from slam_toolbox).
            ox = float(msg.info.origin.position.x)
            oy = float(msg.info.origin.position.y)
            qz = msg.info.origin.orientation.z
            qw = msg.info.origin.orientation.w
            origin_yaw = float(2.0 * np.arctan2(qz, qw))

            payload = {
                'robotId': self.robot_id,
                'width': w,
                'height': h,
                'resolution': float(msg.info.resolution),
                'originX': ox,
                'originY': oy,
                'originYaw': origin_yaw,
                'png_b64': png_b64,
                'frame_id': msg.header.frame_id or 'map',
            }
            message = {
                'type': 'map_update',
                'timestamp': self.get_timestamp(),
                'payload': payload,
            }
            self.get_logger().info(
                f'Relaying map_update: {w}x{h} @ {msg.info.resolution:.3f} m/px, '
                f'{len(png_b64)} bytes b64',
                throttle_duration_sec=10.0,
            )
            loop = getattr(self, 'ws_loop', None)
            if loop is not None:
                asyncio.run_coroutine_threadsafe(self.send_message(message), loop)
        except Exception as e:
            self.get_logger().error(f'Failed to relay map: {e}')

    def cmd_publish_callback(self):
        """Continuously publish movement commands at 10Hz.

        Skip when the GoTo controller is active — that timer owns /cmd_vel
        for the duration of a goal. Without this guard, the goto Twist gets
        overwritten by a zero current_cmd every other tick and the robot
        never actually rotates / drives (silent fail mode that looks like
        a controller stall).
        """
        if self.goto.is_active():
            return
        with self.cmd_lock:
            self.cmd_vel_pub.publish(self.current_cmd)

    def telemetry_callback(self):
        """Send periodic telemetry data to server"""
        if not self.ws_connected or self.websocket is None:
            return

        telemetry = {
            'type': 'telemetry',
            'timestamp': self.get_timestamp(),
            'payload': {
                'robotId': self.robot_id,
                'sensors': self.collect_sensor_data(),
                'system': self.collect_system_data()
            }
        }

        # Send via WebSocket (non-blocking)
        asyncio.run_coroutine_threadsafe(
            self.send_message(telemetry),
            self.ws_loop
        )

    def collect_sensor_data(self):
        """Collect sensor data (IMU, odometry)"""
        sensors = {}

        # IMU data (gyroscope and accelerometer)
        if self.latest_imu is not None:
            sensors['imu'] = {
                'orientation': {
                    'x': float(self.latest_imu.orientation.x),
                    'y': float(self.latest_imu.orientation.y),
                    'z': float(self.latest_imu.orientation.z),
                    'w': float(self.latest_imu.orientation.w)
                },
                'angular_velocity': {
                    'x': float(self.latest_imu.angular_velocity.x),
                    'y': float(self.latest_imu.angular_velocity.y),
                    'z': float(self.latest_imu.angular_velocity.z)
                },
                'linear_acceleration': {
                    'x': float(self.latest_imu.linear_acceleration.x),
                    'y': float(self.latest_imu.linear_acceleration.y),
                    'z': float(self.latest_imu.linear_acceleration.z)
                }
            }

        # Odometry data (position and velocity)
        if self.latest_odom is not None:
            sensors['odometry'] = {
                'position': {
                    'x': float(self.latest_odom.pose.pose.position.x),
                    'y': float(self.latest_odom.pose.pose.position.y),
                    'z': float(self.latest_odom.pose.pose.position.z)
                },
                'orientation': {
                    'x': float(self.latest_odom.pose.pose.orientation.x),
                    'y': float(self.latest_odom.pose.pose.orientation.y),
                    'z': float(self.latest_odom.pose.pose.orientation.z),
                    'w': float(self.latest_odom.pose.pose.orientation.w)
                },
                'linear_velocity': {
                    'x': float(self.latest_odom.twist.twist.linear.x),
                    'y': float(self.latest_odom.twist.twist.linear.y),
                    'z': float(self.latest_odom.twist.twist.linear.z)
                },
                'angular_velocity': {
                    'x': float(self.latest_odom.twist.twist.angular.x),
                    'y': float(self.latest_odom.twist.twist.angular.y),
                    'z': float(self.latest_odom.twist.twist.angular.z)
                }
            }

        # LaserScan data (LiDAR)
        if self.latest_scan is not None:
            # Downsample the ranges for efficiency (send every Nth point)
            downsample_factor = 5
            ranges = list(self.latest_scan.ranges[::downsample_factor])
            # Replace inf/nan values with -1 for JSON serialization
            import math
            ranges = [-1.0 if (math.isinf(r) or math.isnan(r)) else float(r) for r in ranges]

            sensors['scan'] = {
                'angle_min': float(self.latest_scan.angle_min),
                'angle_max': float(self.latest_scan.angle_max),
                'angle_increment': float(self.latest_scan.angle_increment * downsample_factor),
                'range_min': float(self.latest_scan.range_min),
                'range_max': float(self.latest_scan.range_max),
                'ranges': ranges
            }
            self.get_logger().info(f'Adding scan to telemetry: {len(ranges)} points', throttle_duration_sec=5.0)
        else:
            self.get_logger().warning('No scan data available', throttle_duration_sec=5.0)

        # Battery state
        if self.latest_battery is not None:
            sensors['battery'] = {
                'voltage': float(self.latest_battery.voltage),
                'percentage': float(self.latest_battery.percentage),
                'charging': self.latest_battery.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
            }

        # Robot pose in the map frame (only when SLAM has produced TF).
        # Browser uses this to draw the robot's location on the map and to
        # convert click coordinates back into world-frame goals.
        pose = self._get_pose_in_map()
        if pose is not None:
            sensors['pose_in_map'] = {
                'x': pose[0],
                'y': pose[1],
                'theta': pose[2],
                'frame_id': self.map_frame,
            }

        return sensors

    def collect_system_data(self):
        """Collect system information (CPU, memory, network)"""
        try:
            return {
                'cpuUsage': psutil.cpu_percent(interval=None),
                'memoryUsage': psutil.virtual_memory().percent,
                'diskUsage': psutil.disk_usage('/').percent,
                'hostname': socket.gethostname()
            }
        except Exception as e:
            self.get_logger().warning(f'Failed to collect system data: {e}')
            return {}

    def get_timestamp(self):
        """Get current timestamp in ISO format"""
        from datetime import timezone
        return datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'

    async def send_message(self, message):
        """Send message via WebSocket"""
        if self.websocket is not None:
            try:
                await self.websocket.send(json.dumps(message))
            except Exception as e:
                self.get_logger().error(f'Failed to send message: {e}')

    async def send_hello(self):
        """Send initial hello message to server"""
        # Build capabilities based on available dependencies
        capabilities = ['movement', 'odometry', 'imu', 'depth']
        if CV2_AVAILABLE and WEBRTC_AVAILABLE:
            capabilities.insert(0, 'camera')
        # 'map' capability is the contract for the browser to show the map
        # panel — only advertise it when SLAM relay is actually wired up.
        if self.enable_map_relay and PIL_AVAILABLE:
            capabilities.append('map')
        # 'goto' capability — browser shows the click-to-waypoint controls
        # only when the driver actually has the controller wired up. Without
        # enable_goto, incoming cmd_goal messages are ignored.
        if self.enable_goto:
            capabilities.append('goto')

        payload = {
            'robotId': self.robot_id,
            'robotType': 'ros2',  # Explicitly identify as ROS2 robot
            'sdkVersion': '1.0.0',
            'capabilities': capabilities
        }
        # Include API key if configured
        if self.api_key:
            payload['apiKey'] = self.api_key
        hello = {
            'type': 'robot_hello',
            'timestamp': self.get_timestamp(),
            'payload': payload
        }
        await self.send_message(hello)
        self.get_logger().info(f'Sent robot_hello for {self.robot_id}')

    async def handle_message(self, message_str):
        """Handle incoming messages from server"""
        try:
            message = json.loads(message_str)
            msg_type = message.get('type')

            self.get_logger().debug(f'Received message type: {msg_type}')

            if msg_type == 'hello_ack':
                self.get_logger().info('Received hello_ack from server')

            elif msg_type == 'cmd_movement':
                self.handle_movement_command(message.get('payload', {}))

            elif msg_type == 'cmd_goal':
                self.handle_goal_command(message.get('payload', {}))

            elif msg_type == 'cmd_cancel_goal':
                self.handle_cancel_goal_command()

            elif msg_type == 'cmd_snapshot':
                await self.handle_snapshot_command()

            elif msg_type == 'webrtc_offer':
                await self.handle_webrtc_offer(message.get('payload', {}))

            elif msg_type == 'webrtc_ice_candidate':
                await self.handle_ice_candidate(message.get('payload', {}))

            else:
                self.get_logger().info(f'Unhandled message type: {msg_type}')

        except Exception as e:
            self.get_logger().error(f'Error handling message: {e}')

    async def handle_webrtc_offer(self, payload):
        """Handle WebRTC offer from browser client"""
        if not WEBRTC_AVAILABLE:
            self.get_logger().warning('WebRTC not available, ignoring offer')
            return

        try:
            self.get_logger().info('Received WebRTC offer, creating peer connection')

            # Close existing peer connection if any
            if self.pc is not None:
                await self.pc.close()

            # Create new RTCPeerConnection
            self.pc = RTCPeerConnection()

            # Add video track to peer connection
            self.pc.addTrack(self.video_track)

            # Set up ICE candidate handler
            @self.pc.on('icecandidate')
            async def on_icecandidate(candidate):
                if candidate:
                    self.get_logger().info('Sending ICE candidate to browser')
                    ice_msg = {
                        'type': 'webrtc_ice_candidate',
                        'timestamp': self.get_timestamp(),
                        'payload': {
                            'targetClientId': payload.get('sourceClientId'),
                            'candidate': candidate.candidate,
                            'sdpMid': candidate.sdpMid,
                            'sdpMLineIndex': candidate.sdpMLineIndex
                        }
                    }
                    await self.send_message(ice_msg)

            # Handle incoming DataChannels (depth channel created by browser)
            @self.pc.on('datachannel')
            def on_datachannel(channel):
                self.get_logger().info(f'DataChannel received: {channel.label}')
                if channel.label == 'depth':
                    self.depth_channel = channel
                    self.get_logger().info('Depth DataChannel registered')

                    @channel.on('close')
                    def on_close():
                        self.depth_channel = None
                        self.get_logger().info('Depth DataChannel closed')

            # Set remote description (the offer)
            offer_sdp = payload.get('sdp')
            await self.pc.setRemoteDescription(RTCSessionDescription(sdp=offer_sdp, type='offer'))

            # Create answer
            answer = await self.pc.createAnswer()
            await self.pc.setLocalDescription(answer)

            # Send answer back to browser
            self.get_logger().info('Sending WebRTC answer to browser')
            answer_msg = {
                'type': 'webrtc_answer',
                'timestamp': self.get_timestamp(),
                'payload': {
                    'targetClientId': payload.get('sourceClientId'),
                    'sdp': self.pc.localDescription.sdp
                }
            }
            await self.send_message(answer_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to handle WebRTC offer: {e}')

    async def handle_ice_candidate(self, payload):
        """Handle ICE candidate from browser client"""
        if not WEBRTC_AVAILABLE:
            return

        try:
            if self.pc is None:
                self.get_logger().warning('Received ICE candidate but no peer connection exists')
                return

            candidate_str = payload.get('candidate')
            sdp_mid = payload.get('sdpMid')
            sdp_mline_index = payload.get('sdpMLineIndex')

            if candidate_str:
                # Strip "candidate:" prefix if present
                sdp_str = candidate_str
                if sdp_str.startswith('candidate:'):
                    sdp_str = sdp_str[len('candidate:'):]
                candidate = candidate_from_sdp(sdp_str)
                candidate.sdpMid = sdp_mid
                candidate.sdpMLineIndex = sdp_mline_index
                await self.pc.addIceCandidate(candidate)
            self.get_logger().info('Added ICE candidate from browser')

        except Exception as e:
            self.get_logger().error(f'Failed to handle ICE candidate: {e}')

    async def handle_snapshot_command(self):
        """Capture latest camera frame and send as JPEG snapshot"""
        if not CV2_AVAILABLE or self.video_track is None:
            self.get_logger().warning('Snapshot requested but no camera available')
            return

        with self.video_track.frame_lock:
            frame = self.video_track.latest_frame

        if frame is None:
            self.get_logger().warning('Snapshot requested but no frame available')
            return

        try:
            import base64
            _, jpeg_data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            base64_jpeg = base64.b64encode(jpeg_data.tobytes()).decode('utf-8')

            response = {
                'type': 'snapshot_response',
                'timestamp': self.get_timestamp(),
                'payload': {
                    'robotId': self.robot_id,
                    'width': frame.shape[1],
                    'height': frame.shape[0],
                    'data': base64_jpeg
                }
            }
            await self.send_message(response)
            self.get_logger().info(f'Sent snapshot ({len(base64_jpeg)} chars)')
        except Exception as e:
            self.get_logger().error(f'Failed to capture snapshot: {e}')

    def handle_goal_command(self, payload):
        """Receive a single-waypoint goal from the server.

        Payload: {x: float, y: float, theta?: float} (metres / radians in map frame).
        Ignored when enable_goto is false; logs and ack-rejects so the client knows.
        """
        if not self.enable_goto:
            self.get_logger().warning(
                'Received cmd_goal but enable_goto=false; ignoring'
            )
            self._emit_goal_status({
                'state': GoToController.FAILED,
                'distance_remaining': None,
                'goal_id': self.goto.goal_id,
            }, force=True)
            return
        try:
            x = float(payload['x'])
            y = float(payload['y'])
            theta = payload.get('theta')
            theta = float(theta) if theta is not None else None
        except (KeyError, TypeError, ValueError) as e:
            self.get_logger().error(f'Bad cmd_goal payload: {e}; got {payload!r}')
            return

        self._final_goal_xy = (x, y)

        # Plan a path through the SLAM map if we have one. Fall back to the
        # blind single-waypoint behaviour when no map is cached yet (e.g.
        # robot just started, SLAM hasn't published).
        pose = self._get_pose_in_map()
        path: list[tuple[float, float]] = []
        if (pose is not None and
                self.latest_map_grid is not None and
                self.latest_map_info is not None):
            try:
                path = self.planner.plan(
                    start_world=(pose[0], pose[1]),
                    goal_world=(x, y),
                    grid=self.latest_map_grid,
                    info=self.latest_map_info,
                ) or []
            except Exception as e:
                self.get_logger().error(f'Path planning failed: {e}')
                path = []

        if path and len(path) >= 2:
            # Skip the start cell (= robot's current cell); plan() returns
            # corners from start to goal inclusive.
            self._path_waypoints = path[1:]
            self._path_index = 0
            wx, wy = self._path_waypoints[0]
            self.goto.set_goal(wx, wy, None)
            self.get_logger().info(
                f'Planned path: {len(self._path_waypoints)} segment(s) -> ({x:.2f}, {y:.2f}); '
                f'first segment to ({wx:.2f}, {wy:.2f})'
            )
        else:
            # No map / no plan / direct line-of-sight goal
            self._path_waypoints = []
            self._path_index = 0
            # Append the final theta only on the last (only) segment
            self.goto.set_goal(x, y, theta)
            self.get_logger().info(
                f'No path (no map or blind fallback). Goal direct ({x:.2f}, {y:.2f})'
            )

        self._goto_last_state = None  # force next tick to emit
        self._emit_goal_status({
            'state': 'accepted',
            'distance_remaining': None,
            'goal_id': self.goto.goal_id,
        }, force=True)

    def handle_cancel_goal_command(self):
        """Stop any in-progress goal and emit a cancelled status."""
        if not self.goto.is_active():
            return
        self.goto.cancel()
        # Clear any remaining path segments — don't keep driving after cancel
        self._path_waypoints = []
        self._path_index = 0
        self._final_goal_xy = None
        # Publish a zero Twist immediately so the robot stops within one tick.
        zero = Twist()
        self.cmd_vel_pub.publish(zero)
        self._emit_goal_status({
            'state': GoToController.CANCELLED,
            'distance_remaining': None,
            'goal_id': self.goto.goal_id,
        }, force=True)
        self.get_logger().info('Goal cancelled by client')

    def handle_movement_command(self, payload):
        """Handle movement commands from server and update current command"""
        # Manual joystick overrides any active goal — cancel it first so we
        # don't have two things fighting over /cmd_vel.
        if self.goto.is_active():
            self.get_logger().info('Manual cmd_movement received — cancelling active goal')
            self.handle_cancel_goal_command()
        action = payload.get('action')

        twist = Twist()

        # Map server commands to ROS Twist messages
        # Match joystick configuration values (config/joystick.yaml)
        linear_speed = 0.2  # m/s (matches joystick standard speed)
        angular_speed = 0.4  # rad/s (matches joystick standard rotation)

        if action == 'forward':
            twist.linear.x = linear_speed
        elif action == 'backward':
            twist.linear.x = -linear_speed
        elif action == 'left':
            twist.angular.z = angular_speed
        elif action == 'right':
            twist.angular.z = -angular_speed
        elif action == 'stop':
            # Already zero
            pass
        else:
            self.get_logger().warning(f'Unknown movement action: {action}')
            return

        # Update current command (will be published continuously by timer)
        with self.cmd_lock:
            self.current_cmd = twist

        self.get_logger().info(f'Set cmd_vel: {action}')

    async def websocket_handler(self):
        """Main WebSocket connection handler"""
        reconnect_delay = 1
        max_reconnect_delay = 30

        while True:
            try:
                self.get_logger().info(f'Connecting to {self.server_url}...')

                async with websockets.connect(self.server_url) as websocket:
                    self.websocket = websocket
                    self.ws_connected = True
                    reconnect_delay = 1  # Reset delay on successful connection

                    self.get_logger().info('WebSocket connected!')

                    # Send hello message
                    await self.send_hello()

                    # Listen for messages
                    async for message in websocket:
                        await self.handle_message(message)

            except websockets.exceptions.WebSocketException as e:
                self.get_logger().error(f'WebSocket error: {e}')
            except Exception as e:
                self.get_logger().error(f'Connection error: {e}')

            finally:
                self.ws_connected = False
                self.websocket = None

            # Exponential backoff for reconnection
            self.get_logger().info(f'Reconnecting in {reconnect_delay} seconds...')
            await asyncio.sleep(reconnect_delay)
            reconnect_delay = min(reconnect_delay * 2, max_reconnect_delay)

    def run_websocket_loop(self):
        """Run the WebSocket event loop in a separate thread"""
        self.ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.ws_loop)

        try:
            self.ws_loop.run_until_complete(self.websocket_handler())
        except Exception as e:
            self.get_logger().error(f'WebSocket loop error: {e}')
        finally:
            self.ws_loop.close()


def main(args=None):
    rclpy.init(args=args)

    driver = BurfPlatformDriver()

    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
