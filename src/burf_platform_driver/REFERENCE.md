# Burf Platform Driver — Reference

_What the `burf_platform_driver` is, what it contains, and every topic / param / message it speaks. Code-grounded from `BurfPlatform/Drivers/ros2_driver/` (2026-07-11). This is the robot-agnostic driver that puts any ROS2 robot on the Burf Platform — the rover, the TurtleBot2, and now the wheelchair all run this same code._

---

## 1. What it is

A single ROS2 node (`burf_driver`) that bridges a robot's ROS2 graph to the Burf Platform over a WebSocket. It is **robot-agnostic**: every robot-specific detail (topic names, frames, speeds, map-reset command) is a ROS parameter, so the same code runs on every robot with only a per-robot params file.

```
   robot's ROS2 graph                    Platform                browser (operator)
  ┌──────────────────┐   WebSocket    ┌────────────┐   WS      ┌─────────────────┐
  │ /scan /odom /map │◄──wss://──────►│ platform.  │◄────────► │ fleet view,     │
  │ /image_raw /imu  │  robot_hello   │ burf.co    │           │ map, GoTo clicks,│
  │ cmd_vel_nav ...  │  telemetry     │ /ws/robot  │           │ WebRTC video     │
  └──────────────────┘  map_update    └────────────┘           └─────────────────┘
        ▲  burf_driver node  │  cmd_movement / cmd_goal / cmd_map_* / webrtc_*
        └────────────────────┘
```

It does five jobs:
1. **Telemetry relay** — pose, sensors, battery, system stats → Platform (5 Hz default).
2. **Map relay** — a SLAM `/map` → PNG + raw grid for the browser map panel.
3. **Drive** — `cmd_movement` (joystick) and `cmd_goal` (click-to-waypoint) → `Twist` on the robot's cmd_vel topic. GoTo is planned on-robot (homegrown A\*), not blind.
4. **Video** — WebRTC camera stream + JPEG snapshots.
5. **Map lifecycle** — save / load / clear SLAM maps, place the robot (set pose).

---

## 2. Configuration (ROS parameters)

Every behaviour is a parameter. Defaults suit a light robot (rover/TurtleBot); a heavier or differently-wired robot overrides via its own params file.

| Param | Default | Purpose |
|---|---|---|
| `server_url` | `wss://platform.burf.co/ws/robot` | Platform WebSocket endpoint |
| `local` | `false` | Override server_url to a LAN dev server |
| `robot_id` | `ros2-rover-01` | Identifier sent in `robot_hello` |
| `api_key` | `''` | Platform API key. **If empty, read from env `PLATFORM_API_KEY` / `BURF_API_KEY`** — keeps the secret out of committed config. Ownership by `admin@burf.co` is required for map save/persist. |
| `camera_topic` | `/camera/image_raw` | Camera source (`Image`, or `CompressedImage` if `use_compressed`) |
| `use_compressed` | `false` | Subscribe to `CompressedImage` |
| `odom_topic` | `/odom` | Odometry (velocity telemetry) |
| `imu_topic` | `/imu` | IMU (shipped config uses `/imu/data`) |
| `cmd_vel_topic` | `/cmd_vel` | **Twist output.** Point at an arbitration input (e.g. `/cmd_vel_nav`) to keep control behind a safety gate |
| `scan_topic` | `/scan` | LaserScan |
| `depth_topic` | `/camera/depth/image_raw` | Depth image (16UC1) |
| `battery_topic` | `/battery_state` | Battery state |
| `map_topic` | `/map` | OccupancyGrid for the map relay |
| `enable_map_relay` | `false` | Gate: subscribe to map + advertise `map`/`map_save`/`map_load`/`set_pose`. Only on for SLAM robots |
| `enable_goto` | `false` | Gate: wire the on-robot waypoint controller + advertise `goto` |
| `map_frame` | `map` | Map TF frame |
| `base_frame` | `base_footprint` | Robot base TF frame (pose = `map → base_frame`) |
| `movement_linear_speed` | `0.2` m/s | Manual joystick forward/back speed |
| `movement_angular_speed` | `0.4` rad/s | Manual joystick turn speed |
| `robot_radius` | `0.18` m | Path-planner robot radius |
| `inflation_padding` | `0.02` m | Path-planner clearance beyond radius |
| `map_storage_dir` | `/home/burf2000/maps` | Where serialized slam_toolbox maps go |
| `clear_map_cmd` | `''` | Shell command run on `cmd_clear_map` (e.g. restart slam). Empty = no-op |
| `load_map_cmd` | `''` | Shell command to relaunch SLAM in localization mode on `cmd_map_load`. Empty = in-place `deserialize_map` |
| `goal_xy_tol` | `0.15` m | Waypoint accept radius |
| `telemetry_rate` | `5.0` Hz | Telemetry send rate |
| `video_fps` / `depth_fps` | `10` / `5` | Streaming target FPS |

_(`robot_name` appears in the shipped YAML but is not read by the node — dead key.)_

---

## 3. ROS2 topics

### Subscriptions
| Topic (param) | Type | Notes |
|---|---|---|
| camera_topic | `sensor_msgs/Image` \| `CompressedImage` | video + snapshots |
| odom_topic | `nav_msgs/Odometry` | velocity telemetry |
| imu_topic | `sensor_msgs/Imu` | IMU telemetry |
| scan_topic | `sensor_msgs/LaserScan` | scan telemetry + planner |
| depth_topic | `sensor_msgs/Image` (16UC1) | depth |
| battery_topic | `sensor_msgs/BatteryState` | battery |
| map_topic | `nav_msgs/OccupancyGrid` | **QoS: depth 1, TRANSIENT_LOCAL, RELIABLE** (matches slam_toolbox's latched map). Only when `enable_map_relay` **and** Pillow present |

Plus a `tf2_ros.TransformListener` for `map_frame → base_frame` pose lookups (telemetry + GoTo). Goal commands arrive over the WebSocket, not a ROS topic.

### Publishers
| Topic | Type | Purpose |
|---|---|---|
| cmd_vel_topic | `geometry_msgs/Twist` | drive output — manual (10 Hz) or GoTo ticks (10 Hz while active) |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | seed localization on `cmd_set_pose` |

---

## 4. WebSocket protocol

### Inbound (Platform → robot)
| Type | Effect |
|---|---|
| `hello_ack` | connection acknowledged |
| `cmd_movement` | joystick `{action: forward/backward/left/right/stop}` → Twist (cancels any active goal) |
| `cmd_goal` | `{x,y,theta?,allow_unknown?}` — plan via A\* if map+pose, else blind direct; ignored if `enable_goto=false` |
| `cmd_cancel_goal` | stop + clear route overlay |
| `cmd_snapshot` | capture camera → `snapshot_response` (JPEG) |
| `webrtc_offer` / `webrtc_ice_candidate` | WebRTC video negotiation |
| `cmd_clear_map` | run `clear_map_cmd` |
| `cmd_map_save` | serialize slam map → upload blob |
| `cmd_map_load` | write posegraph to disk, relaunch (localization) or deserialize in place |
| `cmd_set_pose` | publish `/initialpose` |

### Outbound (robot → Platform)
`robot_hello` (on connect) · `telemetry` (5 Hz) · `goal_status` (on transitions / throttled) · `planned_path` (route overlay) · `map_update` (per new /map) · `snapshot_response` · `webrtc_answer` · `webrtc_ice_candidate` · `map_save_result` · `map_load_result` · `mode_changed_result` (mapping↔localization) · `set_pose_result`.

_Null-valued optional fields are omitted from payloads — the server's `AnyCodable` decoder used to reject a whole message on a JSON `null`._

### Capabilities (advertised in `robot_hello`)
Always: **movement, odometry, imu, depth**. Conditional: **camera** (cv2+WebRTC present) · **map / map_save / map_load / set_pose** (`enable_map_relay` + Pillow / slam services) · **goto** (`enable_goto`). The browser shows a panel only when its capability is advertised.

---

## 5. On-robot navigation (why A\* lives in the driver)

The driver plans and drives GoTo itself — it does **not** require Nav2. Two small pure-Python modules:

- **`path_planner.py` — homegrown A\*.** On the SLAM OccupancyGrid: denoise small occupied blobs (≤`MIN_OCCUPIED_COMPONENT`=3 cells) → inflate obstacles by `robot_radius + inflation_padding` → optionally block "deep unknown" (>`UNKNOWN_REACH_M`=0.30 m from any mapped-free cell) → 8-connected A\* with octile heuristic + corner-cut prevention → line-of-sight simplification to corner waypoints. The `allow_unknown` flag: **exploration** plans through unmapped space toward frontiers; **manual GoTo** blocks deep unknown so it never drives blind into unseen walls. The robot's own start cell is always force-cleared so sensor noise beside it can't make it unplannable.
- **`goto_controller.py` — drive controller.** State machine `IDLE → ROTATING → DRIVING → ARRIVED` (+ `FAILED` on stall). Deliberately slow (`lin 0.15` m/s, `ang 0.25` rad/s) for cleaner SLAM scans. **Stall watchdog** counts progress as ≥5 cm closer *or* ≥~5° heading improvement, so a slow in-place turn doesn't false-trip — only a genuinely stuck robot fails (15 s). Cosine-weighted speed + gentle yaw P-gain avoid the circle-driving overshoot.

Server-side exploration sends frontier goals as ordinary `cmd_goal` with `allow_unknown: true`; the robot's own planner routes them.

---

## 6. Python dependencies

`websockets` (Platform link) · `numpy` · `psutil` (system telemetry) · **optional, feature-gating:** `Pillow` (map PNG → gates map relay) · `cv2` (camera decode → gates camera) · `aiortc` + `av` (WebRTC video) · `scipy.ndimage` (planner inflation/denoise — has pure-numpy fallbacks).

---

## 7. Per-robot deployment pattern

1. **Install the driver** — vendor the published tarball (`platform.burf.co/install/ros2_driver.tar.gz`) into the robot's ROS2 workspace and `colcon build`. Single source of truth; no forked copy.
2. **Write a params file** — topics, frames, speeds, cmd_vel target, enable flags. Pass it via `burf_driver.launch.py`'s `params_file` arg (the file provides the bulk; launch args like `robot_id`/`enable_*` override on top).
3. **Supply the API key via env** — `PLATFORM_API_KEY` in the environment (systemd unit / `docker run -e` / env-file). Never commit it.

### Worked example — the wheelchair
A no-encoder 90 kg base on a Docker-ROS2 Pi. Its config: `cmd_vel_topic: /cmd_vel_nav` (Platform drive flows through a `twist_mux` behind the ARM switch — enforced in `serial_bridge` via `/autonomy_armed`; the RC stick always outranks it), `base_frame: base_footprint`, `movement_linear_speed: 0.5` / `0.8` (clears the motor deadband), `clear_map_cmd: pkill async_slam_toolbox_node` (slam runs in the same container with `respawn=True` → comes back with an empty map). Odometry is rf2o laser-odometry + a robot_localization EKF (no wheel encoders); slam_toolbox builds the map; all three run in one `wc-autonomy` container so the driver can restart slam for clear-map.

---

## 8. Deploy gotchas (learned the hard way)

- **Map QoS** — slam_toolbox publishes `/map` latched (TRANSIENT_LOCAL); the driver's map sub matches it. A VOLATILE sub misses the latched map while the robot is stationary → no map until it moves.
- **Frames** — the EKF must publish `odom → base_frame` for the *same* `base_frame` slam and the driver use. Publishing `odom → base_link` while the URDF has `base_footprint → base_link` double-parents `base_link` and silently kills both SLAM and GoTo (sim can hide it — sim publishes the correct edge).
- **API key in containers** — the env-fallback only helps if the container actually has `PLATFORM_API_KEY` in its environment. A robot with no key connects *unowned* and map save/persist silently fails.
- **Pillow / aiortc** — missing Pillow disables the whole map relay; missing aiortc/av disables WebRTC video. Both must be in the image, not just the running container.
