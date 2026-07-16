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
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import tf2_ros
import base64
import io
import os
import re
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
        # Manual joystick (cmd_movement) speeds. Defaults suit a light robot
        # (TurtleBot/rover); a heavier base like the wheelchair overrides these
        # upward via its params file to clear the motor deadband.
        self.declare_parameter('movement_linear_speed', 0.2)   # m/s
        self.declare_parameter('movement_angular_speed', 0.4)  # rad/s
        # Directory where slam_toolbox posegraph/.data map files are serialized
        # to / loaded from on the robot.
        self.declare_parameter('map_storage_dir', '/home/burf2000/maps')
        # Path-planner obstacle clearance. Defaults preserve prior behaviour
        # (rover/TurtleBot unchanged); a robot whose firmware brakes near
        # surfaces (e.g. the Sanbot's obstacle avoidance) sets a bigger
        # inflation_padding so planned routes/headings keep clear of walls and
        # never trip the firmware. inflation_padding = clearance beyond the
        # robot body edge.
        self.declare_parameter('robot_radius', 0.18)
        self.declare_parameter('inflation_padding', 0.02)
        # Shell command run on a 'cmd_clear_map' request to reset the robot's
        # SLAM map (e.g. restart the slam node so it starts a fresh map). Left
        # empty by default so robots without a reset mechanism just no-op —
        # keeps the driver robot-agnostic; each robot sets its own in its launch.
        self.declare_parameter('clear_map_cmd', '')
        # Shell command run on a 'cmd_map_load' request to bring the robot up in
        # LOCALIZATION mode against a saved map (e.g. relaunch the slam node with
        # slam_mode:=localization + the loaded posegraph). The saved map's
        # basename is appended as a single argv arg. Left empty by default so
        # robots without a mode-switch mechanism fall back to the in-place
        # deserialize_map path below — keeps the driver robot-agnostic; each
        # robot that needs a real mode switch sets its own command.
        self.declare_parameter('load_map_cmd', '')
        # Waypoint accept radius. Default matches GoToController's 0.15m. A
        # coarse-positioning base (the Sanbot's slowest real speed is ~0.25 m/s
        # — a friction floor — so each approach pulse moves 10-25cm) can't
        # reliably land in a 15cm circle; it overshoots, hunts around the goal
        # and stall-fails while physically ON the waypoint. Such robots set a
        # tolerance matching their positioning granularity (Sanbot: 0.30).
        self.declare_parameter('goal_xy_tol', 0.15)
        self.declare_parameter('telemetry_rate', 5.0)  # Hz
        self.declare_parameter('video_fps', 10)  # Target FPS for video streaming
        self.declare_parameter('depth_fps', 5)  # Target FPS for depth streaming

        # Get parameters
        use_local = self.get_parameter('local').value
        self.server_url = 'ws://192.168.0.50:8081/ws/robot' if use_local else self.get_parameter('server_url').value
        self.robot_id = self.get_parameter('robot_id').value
        self.api_key = self.get_parameter('api_key').value
        # Env fallback so the key can stay out of committed config: if the params
        # file leaves api_key empty, take it from PLATFORM_API_KEY / BURF_API_KEY
        # (set only in the deploy env-file / systemd unit).
        if not self.api_key:
            self.api_key = (os.environ.get('PLATFORM_API_KEY')
                            or os.environ.get('BURF_API_KEY') or '')
            if self.api_key:
                self.get_logger().info('api_key sourced from environment')
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
        self.map_storage_dir = self.get_parameter('map_storage_dir').value
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
        self.goto = GoToController(
            goal_xy_tol=self.get_parameter('goal_xy_tol').value)
        self._goto_active_goal_id = None    # remembered to avoid spamming goal_status
        self._goto_last_state = None        # for state-transition detection
        # Grid path planner for obstacle-aware nav (homegrown A* on the SLAM
        # OccupancyGrid). Cached map grid + info filled in by map_callback.
        self.planner = GridPathPlanner(
            robot_radius_m=self.get_parameter('robot_radius').value,
            inflation_padding_m=self.get_parameter('inflation_padding').value,
        )
        self.latest_map_grid = None         # numpy int8 grid, NOT flipped
        self.latest_map_info = None         # GridInfo
        # slam_toolbox serialize/deserialize service clients + message types.
        # Created at startup (below) when map relay is enabled.
        self._serialize_cli = None
        self._deserialize_cli = None
        self._SerializePoseGraph = None
        self._DeserializePoseGraph = None
        self._loaded_map_path = None
        # /initialpose publisher — used by cmd_set_pose to seed slam_toolbox /
        # AMCL localization at a chosen point in the map frame.
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        # Create the slam_toolbox serialize/deserialize service clients (for
        # cmd_map_save / cmd_map_load) now that the None-defaults above are set.
        if self.enable_map_relay:
            self._create_slam_clients()
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
                elif msg.encoding == 'yuv422_yuy2':
                    # v4l2_camera's native passthrough (output_encoding=yuv422_yuy2);
                    # cheap reshape was (h,w,2) above — convert packed YUYV -> BGR.
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2BGR_YUY2)

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

    def _emit_planned_path(self, points, goal_id):
        """Send the computed route (world (x, y) corners, map frame) to the
        server so the browser can draw it on the map as an overlay — the actual
        path the robot will drive, distinct from the user's clicked waypoints.

        `points` is a list of (x, y) tuples ordered start -> goal. An empty list
        clears the overlay (goal reached / cancelled / no plan). Additive: robots
        that never plan simply never emit this, and the browser hides the layer."""
        loop = getattr(self, 'ws_loop', None)
        if loop is None:
            return
        msg = {
            'type': 'planned_path',
            'timestamp': self.get_timestamp(),
            'payload': {
                'robotId': self.robot_id,
                'frame': self.map_frame,
                'goal_id': goal_id,
                'points': [[float(px), float(py)] for (px, py) in points],
            },
        }
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

        # Clear the route overlay once, on the transition into a terminal state
        # (final arrival or failure). Cancel is cleared in its own handler.
        if (state in (GoToController.ARRIVED, GoToController.FAILED)
                and state != self._goto_last_state):
            self._emit_planned_path([], self.goto.goal_id)

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

            # Raw occupancy grid (int8, ROS order: row-major, row 0 = bottom,
            # +y up — NOT the flipped PNG). Values: -1 unknown, 0 free, 1..100
            # occupancy. Shipped so the server can do frontier detection /
            # autonomous exploration itself (the robot keeps NO exploration
            # logic — it just provides its map data and drives cmd_goal). Same
            # width/height/resolution/origin as the fields above.
            grid_b64 = base64.b64encode(
                np.ascontiguousarray(data, dtype=np.int8).tobytes()
            ).decode('ascii')

            payload = {
                'robotId': self.robot_id,
                'width': w,
                'height': h,
                'resolution': float(msg.info.resolution),
                'originX': ox,
                'originY': oy,
                'originYaw': origin_yaw,
                'png_b64': png_b64,
                'grid_b64': grid_b64,
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
        # Map save/load only when the slam_toolbox serialize/deserialize
        # service clients were successfully created; set_pose only needs the
        # /initialpose publisher (always present when relay is on).
        if self.enable_map_relay and self._serialize_cli is not None:
            capabilities.append('map_save')
        if self.enable_map_relay and self._deserialize_cli is not None:
            capabilities.append('map_load')
        if self.enable_map_relay:
            capabilities.append('set_pose')
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

            elif msg_type == 'cmd_clear_map':
                self.handle_clear_map_command()

            elif msg_type == 'cmd_map_save':
                self.handle_map_save_command(message.get('payload', {}))

            elif msg_type == 'cmd_map_load':
                self.handle_map_load_command(message.get('payload', {}))

            elif msg_type == 'cmd_set_pose':
                self.handle_set_pose_command(message.get('payload', {}))

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

        # allow_unknown: exploration sets it true (route toward frontiers through
        # unmapped space); a manual goto omits it, so the planner blocks unknown
        # and never drives the robot blind into unseen walls.
        allow_unknown = bool(payload.get('allow_unknown', False))

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
                    allow_unknown=allow_unknown,
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
            # Overlay the full computed route (start -> goal) on the browser map.
            self._emit_planned_path(path, self.goto.goal_id)
        elif (pose is not None and
              self.latest_map_grid is not None and
              self.latest_map_info is not None):
            # We HAVE a map + pose but A* found no reachable route at all (the
            # robot's own cell is boxed in). Do NOT blind-drive a straight line
            # through known walls — that was the bug that plowed into obstacles.
            # Fail the goal so explore moves on / the operator sees it.
            self.get_logger().warning(
                f'No reachable path to ({x:.2f}, {y:.2f}) on the map; failing '
                f'goal instead of driving blind through obstacles.'
            )
            self._path_waypoints = []
            self._path_index = 0
            self._emit_planned_path([], self.goto.goal_id)  # clear overlay
            self._emit_goal_status({
                'state': GoToController.FAILED,
                'distance_remaining': None,
                'goal_id': self.goto.goal_id,
            }, force=True)
            return
        else:
            # No map/pose yet (startup, SLAM not up) — blind single-waypoint.
            # Safe-ish: there's no known map to violate. Once SLAM publishes,
            # subsequent goals plan properly.
            self._path_waypoints = []
            self._path_index = 0
            # Append the final theta only on the last (only) segment
            self.goto.set_goal(x, y, theta)
            self.get_logger().info(
                f'No map yet; blind direct goal ({x:.2f}, {y:.2f})'
            )
            # Straight-line intent overlay (robot -> goal) when we couldn't plan.
            direct = [(pose[0], pose[1]), (x, y)] if pose is not None else [(x, y)]
            self._emit_planned_path(direct, self.goto.goal_id)

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
        self._emit_planned_path([], self.goto.goal_id)  # clear the map overlay
        # Publish a zero Twist immediately so the robot stops within one tick.
        zero = Twist()
        self.cmd_vel_pub.publish(zero)
        self._emit_goal_status({
            'state': GoToController.CANCELLED,
            'distance_remaining': None,
            'goal_id': self.goto.goal_id,
        }, force=True)
        self.get_logger().info('Goal cancelled by client')

    def handle_clear_map_command(self):
        """Reset the robot's SLAM map on operator request (portal 'Clear map').

        Runs the configurable `clear_map_cmd` (e.g. restart the slam node so it
        starts a fresh map). Fire-and-forget: the slam node restarting will
        publish a fresh empty /map, which the map relay forwards to the portal.
        Robots with no clear_map_cmd set just log and no-op.
        """
        import shlex
        import subprocess
        cmd = self.get_parameter('clear_map_cmd').value
        if not cmd:
            self.get_logger().warning(
                'cmd_clear_map received but clear_map_cmd is not set — no-op')
            return
        try:
            subprocess.Popen(shlex.split(cmd),
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'cmd_clear_map: launched map reset -> {cmd!r}')
            # A clear/new-map relaunch brings slam back up in mapping mode; tell
            # the UI so the mode indicator flips back from any prior localization.
            self._emit_mode_changed('mapping', True)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'cmd_clear_map failed to run {cmd!r}: {e}')

    def _emit_map_save_result(self, name, ok, path=None, error=None,
                              upload_blob=False):
        """Send a map_save_result back over the WebSocket. Includes the current
        map metadata so the server can persist a SavedMap row without a second
        round-trip.

        When upload_blob is True and path is set, the serialized <path>.posegraph
        and <path>.data files are read and base64-encoded into the payload so the
        SERVER holds the map bytes (one central library), not just the robot. The
        map can then be pushed back to any robot via cmd_map_load."""
        # NOTE: omit null fields. The server decodes the message payload via an
        # AnyCodable that CANNOT decode JSON null — a single `"error": null`
        # (the success case) makes it reject the ENTIRE map_save_result before
        # it reaches the persist logic, so the map silently never stores. Only
        # include path/error when they actually have a value.
        payload = {
            'robotId': self.robot_id,
            'name': name,
            'ok': bool(ok),
        }
        if path is not None:
            payload['path'] = path
        if error is not None:
            payload['error'] = error
        info = self.latest_map_info
        if info is not None:
            payload['resolution'] = float(info.resolution)
            payload['originX'] = float(info.origin_x)
            payload['originY'] = float(info.origin_y)
            payload['width'] = int(info.width)
            payload['height'] = int(info.height)
        if ok and upload_blob and path:
            blob = self._read_map_blob(path)
            if blob is not None:
                payload['posegraph_b64'] = blob['posegraph_b64']
                payload['data_b64'] = blob['data_b64']
            else:
                self.get_logger().warning(
                    'map_save_result: serialize succeeded but could not read '
                    'posegraph/.data blob to upload'
                )
        message = {
            'type': 'map_save_result',
            'timestamp': self.get_timestamp(),
            'payload': payload,
        }
        loop = getattr(self, 'ws_loop', None)
        if loop is not None:
            asyncio.run_coroutine_threadsafe(self.send_message(message), loop)

    def handle_map_save_command(self, payload):
        """Serialize the current slam_toolbox map to disk on the robot.

        Payload: {name: str}. Writes <map_storage_dir>/<name>.posegraph + .data
        via the slam_toolbox /serialize_map service. On completion emits a
        map_save_result message (ok + metadata, or ok:False + error) so the
        server persists the SavedMap row only after the bytes actually land.

        The saved posegraph is what `mode: localization` reloads next session
        to relocalize against this map.
        """
        # Sanitise name → safe filename (alnum / dash / underscore only).
        raw_name = str(payload.get('name', '') or '').strip()
        name = re.sub(r'[^A-Za-z0-9_-]', '', raw_name)
        if not name:
            self.get_logger().error(f'cmd_map_save: bad/empty name {raw_name!r}')
            self._emit_map_save_result(raw_name or 'unnamed', False,
                                       error='invalid name')
            return

        if not self.enable_map_relay or self._serialize_cli is None:
            self.get_logger().warning(
                'cmd_map_save received but map relay/serialize client not '
                'available (enable_map_relay=false or slam_toolbox missing)'
            )
            self._emit_map_save_result(name, False, error='map save not enabled')
            return

        if self.latest_map_info is None:
            self.get_logger().warning('cmd_map_save: no map received yet')
            self._emit_map_save_result(name, False, error='no map')
            return

        # slam_toolbox appends .posegraph / .data to the filename we give it.
        path = os.path.join(self.map_storage_dir, name)
        try:
            os.makedirs(self.map_storage_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f'cmd_map_save: cannot create dir: {e}')
            self._emit_map_save_result(name, False, path=path,
                                       error=f'mkdir failed: {e}')
            return

        if not self._serialize_cli.service_is_ready():
            # Don't block the WS handler thread waiting for the service — fail
            # fast and let the user retry once slam_toolbox is up.
            if not self._serialize_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(
                    'cmd_map_save: /slam_toolbox/serialize_map not available'
                )
                self._emit_map_save_result(
                    name, False, path=path,
                    error='serialize_map service unavailable')
                return

        request = self._SerializePoseGraph.Request()
        request.filename = path
        self.get_logger().info(f'cmd_map_save: serializing map to {path}')
        future = self._serialize_cli.call_async(request)

        def _on_done(fut, _name=name, _path=path):
            try:
                fut.result()  # raises if the call failed
                self.get_logger().info(f'Map saved: {_path}.posegraph/.data')
                # Upload the serialized bytes so the server holds the map
                # centrally (not just this robot's disk).
                self._emit_map_save_result(_name, True, path=_path,
                                           upload_blob=True)
            except Exception as e:
                self.get_logger().error(f'Map serialize failed: {e}')
                self._emit_map_save_result(_name, False, path=_path,
                                           error=str(e))

        future.add_done_callback(_on_done)

    # ─── Map load (server-managed) ──────────────────────────────────────────

    def _read_map_blob(self, path):
        """Read <path>.posegraph and <path>.data and return them base64-encoded,
        or None if either is missing/unreadable. `path` is the basename without
        extension (what slam_toolbox serialize_map was given)."""
        try:
            with open(path + '.posegraph', 'rb') as f:
                pg = base64.b64encode(f.read()).decode('ascii')
            with open(path + '.data', 'rb') as f:
                dt = base64.b64encode(f.read()).decode('ascii')
            return {'posegraph_b64': pg, 'data_b64': dt}
        except Exception as e:
            self.get_logger().error(f'Could not read map blob at {path}: {e}')
            return None

    def _emit_map_load_result(self, name, ok, path=None, error=None):
        """Reply to cmd_map_load so the server/browser can toast success/fail.

        Omit null fields — the server's AnyCodable decoder historically choked
        on JSON null, and a single null would reject the whole message (same
        invariant as _emit_map_save_result)."""
        payload = {
            'robotId': self.robot_id,
            'name': name,
            'ok': bool(ok),
        }
        if path is not None:
            payload['path'] = path
        if error is not None:
            payload['error'] = error
        message = {
            'type': 'map_load_result',
            'timestamp': self.get_timestamp(),
            'payload': payload,
        }
        loop = getattr(self, 'ws_loop', None)
        if loop is not None:
            asyncio.run_coroutine_threadsafe(self.send_message(message), loop)

    def _emit_mode_changed(self, mode, ok, error=None):
        """Tell the server/browser the slam mode switched (mapping<->localization).

        The browser's handleModeChanged reads payload.mode + payload.ok (and
        payload.error on failure) to update the mode indicator. Omit null fields
        — the server's AnyCodable decoder historically choked on JSON null."""
        payload = {
            'robotId': self.robot_id,
            'mode': mode,
            'ok': bool(ok),
        }
        if error is not None:
            payload['error'] = error
        message = {
            'type': 'mode_changed_result',
            'timestamp': self.get_timestamp(),
            'payload': payload,
        }
        loop = getattr(self, 'ws_loop', None)
        if loop is not None:
            asyncio.run_coroutine_threadsafe(self.send_message(message), loop)

    def _create_slam_clients(self):
        """Create the slam_toolbox serialize/deserialize service clients at
        startup."""
        if not self.enable_map_relay:
            return
        for attr in ('_serialize_cli', '_deserialize_cli'):
            cli = getattr(self, attr, None)
            if cli is not None:
                try:
                    self.destroy_client(cli)
                except Exception:
                    pass
                setattr(self, attr, None)
        try:
            from slam_toolbox.srv import SerializePoseGraph
            self._SerializePoseGraph = SerializePoseGraph
            self._serialize_cli = self.create_client(
                SerializePoseGraph, '/slam_toolbox/serialize_map')
            self.get_logger().info(
                f'serialize_map client ready (maps dir: {self.map_storage_dir})')
        except Exception as e:
            self.get_logger().warning(
                f'slam_toolbox SerializePoseGraph unavailable; map save off: {e}')
        try:
            from slam_toolbox.srv import DeserializePoseGraph
            self._DeserializePoseGraph = DeserializePoseGraph
            self._deserialize_cli = self.create_client(
                DeserializePoseGraph, '/slam_toolbox/deserialize_map')
            self.get_logger().info('deserialize_map client ready')
        except Exception as e:
            self.get_logger().warning(
                f'slam_toolbox DeserializePoseGraph unavailable; map load off: {e}')

    def handle_map_load_command(self, payload):
        """Load a saved map into the RUNNING slam_toolbox node.

        Payload: {name, path?, posegraph_b64, data_b64}. The server sends the
        map bytes it holds centrally; the driver writes them to
        <map_storage_dir>/<name>.posegraph/.data, then calls
        /slam_toolbox/deserialize_map with that basename and
        match_type=LOCALIZE_AT_POSE so the live node relocalizes against the
        loaded graph.

        Two paths, chosen per-robot:
        - If `load_map_cmd` is set, relaunch slam in LOCALIZATION mode against
          the saved posegraph. This swaps the published /map to the loaded map
          (e.g. a different room) and honours /initialpose so place-robot works.
          Preferred where a mode switch is available (e.g. the TurtleBot).
        - Otherwise fall back to deserialize_map into the RUNNING node with
          match_type=LOCALIZE_AT_POSE — a thin bridge that never relaunches slam,
          but keeps the current map/mode (place-robot won't take). Robot-agnostic
          default for robots without a mode-switch command."""
        raw_name = str(payload.get('name', '') or '').strip()
        name = re.sub(r'[^A-Za-z0-9_-]', '', raw_name)
        if not name:
            self.get_logger().error(f'cmd_map_load: bad/empty name {raw_name!r}')
            self._emit_map_load_result(raw_name or 'unnamed', False,
                                       error='invalid name')
            return

        if not self.enable_map_relay:
            self._emit_map_load_result(
                name, False,
                error='map load not enabled (enable_map_relay=false)')
            return

        pg_b64 = payload.get('posegraph_b64')
        dt_b64 = payload.get('data_b64')
        if not pg_b64 or not dt_b64:
            self.get_logger().error('cmd_map_load: missing posegraph/data blob')
            self._emit_map_load_result(name, False,
                                       error='missing map blob in command')
            return

        path = os.path.join(self.map_storage_dir, name)
        try:
            os.makedirs(self.map_storage_dir, exist_ok=True)
            with open(path + '.posegraph', 'wb') as f:
                f.write(base64.b64decode(pg_b64))
            with open(path + '.data', 'wb') as f:
                f.write(base64.b64decode(dt_b64))
        except Exception as e:
            self.get_logger().error(f'cmd_map_load: write failed: {e}')
            self._emit_map_load_result(name, False, path=path,
                                       error=f'write failed: {e}')
            return

        self.get_logger().info(f'cmd_map_load: wrote {path}.posegraph/.data')

        # Preferred path: if this robot has a load_map_cmd, relaunch slam in
        # LOCALIZATION mode against the saved posegraph. This actually swaps the
        # published /map to the loaded map (e.g. a different room) and honours
        # /initialpose so place-robot works — unlike deserialize-into-a-running-
        # mapping-node, which keeps the current map and ignores /initialpose.
        # The map basename is passed as a single argv arg (name is already
        # sanitised to [A-Za-z0-9_-], so no shell-injection surface).
        load_cmd = str(self.get_parameter('load_map_cmd').value or '').strip()
        if load_cmd:
            import shlex
            import subprocess
            try:
                subprocess.Popen(shlex.split(load_cmd) + [name],
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                self.get_logger().info(
                    f'cmd_map_load: launched localization relaunch -> '
                    f'{load_cmd!r} {name!r}')
                self._loaded_map_path = path
                # Optimistic: the relaunch takes ~10-20s to publish /map; the
                # browser shows "switching…" and refreshes the map list after.
                self._emit_mode_changed('localization', True)
                self._emit_map_load_result(name, True, path=path)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(
                    f'cmd_map_load: load_map_cmd failed to run {load_cmd!r}: {e}')
                self._emit_map_load_result(name, False, path=path,
                                           error=f'load_map_cmd failed: {e}')
            return

        # Fallback (robot-agnostic): deserialize into the running node in place.
        if self._deserialize_cli is None:
            self._emit_map_load_result(
                name, False, path=path,
                error='slam_toolbox deserialize_map unavailable')
            return

        if not self._deserialize_cli.service_is_ready():
            if not self._deserialize_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(
                    'cmd_map_load: /slam_toolbox/deserialize_map not available'
                )
                self._emit_map_load_result(
                    name, False, path=path,
                    error='deserialize_map service unavailable')
                return

        # slam_toolbox DeserializePoseGraph: filename (no extension) + match_type.
        # LOCALIZE_AT_POSE (3) loads the graph and switches the running node to
        # localization against it, starting from the given initial pose (we use
        # the origin; a follow-up cmd_set_pose can seed a better one).
        request = self._DeserializePoseGraph.Request()
        request.filename = path
        request.match_type = 3  # LOCALIZE_AT_POSE
        try:
            request.initial_pose.x = 0.0
            request.initial_pose.y = 0.0
            request.initial_pose.theta = 0.0
        except Exception:
            # Some message variants omit initial_pose; ignore if absent.
            pass

        self.get_logger().info(
            f'cmd_map_load: deserializing {path} into running slam_toolbox'
        )
        future = self._deserialize_cli.call_async(request)

        def _on_done(fut, _name=name, _path=path):
            try:
                fut.result()  # raises if the call failed
                self._loaded_map_path = _path
                self.get_logger().info(
                    f'cmd_map_load: deserialize_map succeeded for {_path}'
                )
                self._emit_map_load_result(_name, True, path=_path)
            except Exception as e:
                self.get_logger().error(f'cmd_map_load: deserialize failed: {e}')
                self._emit_map_load_result(_name, False, path=_path,
                                           error=str(e))

        future.add_done_callback(_on_done)

    def handle_set_pose_command(self, payload):
        """Place the robot at (x, y, theta) in the map frame by publishing a
        PoseWithCovarianceStamped on /initialpose. slam_toolbox localization
        (and AMCL) seed their estimate from this, so the robot relocates there
        and the next scan-match converges around it.

        Payload: {x, y, theta} — metres / radians in the map frame."""
        try:
            x = float(payload['x'])
            y = float(payload['y'])
            theta = float(payload.get('theta', 0.0))
        except (KeyError, TypeError, ValueError) as e:
            self.get_logger().error(f'Bad cmd_set_pose payload: {e}; got {payload!r}')
            self._emit_set_pose_result(False, error='invalid payload')
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        # yaw -> quaternion (z, w only — flat ground)
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        # Modest covariance — tells the filter "fairly sure, but refine".
        # [x, y, ... yaw] diagonal; values from the RViz 2D-Pose-Estimate default.
        cov = [0.0] * 36
        cov[0] = 0.25      # x
        cov[7] = 0.25      # y
        cov[35] = 0.0685   # yaw (~15°)
        msg.pose.covariance = cov

        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            f'cmd_set_pose: published /initialpose ({x:.2f}, {y:.2f}, {theta:.2f} rad)'
        )
        self._emit_set_pose_result(True, x=x, y=y, theta=theta)

    def _emit_set_pose_result(self, ok, x=None, y=None, theta=None, error=None):
        """Reply to cmd_set_pose so the browser can confirm placement.

        Omit null fields — the server's AnyCodable decoder historically choked
        on JSON null (same invariant as _emit_map_save_result)."""
        payload = {
            'robotId': self.robot_id,
            'ok': bool(ok),
        }
        if x is not None:
            payload['x'] = x
        if y is not None:
            payload['y'] = y
        if theta is not None:
            payload['theta'] = theta
        if error is not None:
            payload['error'] = error
        message = {
            'type': 'set_pose_result',
            'timestamp': self.get_timestamp(),
            'payload': payload,
        }
        loop = getattr(self, 'ws_loop', None)
        if loop is not None:
            asyncio.run_coroutine_threadsafe(self.send_message(message), loop)

    def handle_movement_command(self, payload):
        """Handle movement commands from server and update current command"""
        # Manual joystick overrides any active goal — cancel it first so we
        # don't have two things fighting over /cmd_vel.
        if self.goto.is_active():
            self.get_logger().info('Manual cmd_movement received — cancelling active goal')
            self.handle_cancel_goal_command()
        action = payload.get('action')

        twist = Twist()

        # Map server commands to ROS Twist messages. Speeds are params
        # (movement_linear_speed / movement_angular_speed) so a heavier base can
        # override them; defaults match the joystick standard for light robots.
        linear_speed = self.get_parameter('movement_linear_speed').value
        angular_speed = self.get_parameter('movement_angular_speed').value

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

                # max_size=None disables the inbound 1 MB cap: cmd_map_load
                # pushes a full serialized map (posegraph + .data blob, base64)
                # from the server's central library back to the robot, which
                # easily exceeds 1 MB. Without this the client rejects the load
                # frame. The server is trusted, so no inbound limit is needed.
                async with websockets.connect(self.server_url, max_size=None) as websocket:
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
