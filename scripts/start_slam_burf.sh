#!/bin/bash
# start_slam_burf.sh — SLAM (+ optionally Nav2) + the Burf Platform driver.
#
# Run at boot by turtlebot2-slam-burf.service, AFTER turtlebot2-burf.service has
# recreated the container and started the bringup launch.
#
# ============================================================================
# THE NAV2 TOGGLE — ONE FLAG, TWO CONSEQUENCES
# ============================================================================
# /etc/turtlebot2/nav2.enabled is the SINGLE SOURCE OF TRUTH for Nav2. It is
# read ONCE, here, into $NAV2, and that one variable decides BOTH:
#
#   1. whether nav2.launch.py runs at all, and
#   2. whether burf.launch.py is given enable_nav2:=true
#
# They must never be set independently, because they are two halves of one
# decision about who owns /cmd_vel:
#
#   * burf_platform_driver publishes a 10 Hz /cmd_vel heartbeat. It is a
#     DEADMAN — it repeats the last command so the Kobuki halts if the link to
#     the Platform drops.
#   * Nav2's velocity_smoother publishes /cmd_vel at ~20 Hz and owns the base
#     while a goal is running.
#
#   Nav2 ON  + heartbeat ON     -> two publishers fight, the robot stutters, and
#                                  it reads as "Nav2 is broken" when it is
#                                  really a topic fight.
#   Nav2 OFF + enable_nav2 true -> THE DANGEROUS ONE: nothing driving the base
#                                  AND no deadman. Never allow this.
#
# So do not add a second switch or an env-var override. Change the file and
# re-run this script — that is what scripts/nav2_toggle.sh does.
#
# Usage:
#   start_slam_burf.sh          boot mode: wait for bringup, settle 2 min,
#                               restart SLAM + Nav2 + driver
#   start_slam_burf.sh --now    immediate: skip the boot settle, and leave an
#                               already-running slam_toolbox alone so a map in
#                               progress survives the toggle
# ============================================================================

set -u

NAV2_FLAG=/etc/turtlebot2/nav2.enabled
CTR=turtlebot2_bringup
NOW=false
[ "${1:-}" = "--now" ] && NOW=true

ros() { echo "source /opt/ros/humble/setup.bash && source /root/turtlebot2_ws/install/setup.bash && $1"; }

# NOTE ON THE [b]rackets BELOW — they are not a typo.
# `pkill -f` / `pgrep -f` match against WHOLE command lines, and the command
# line of the `docker exec ... bash -lc "pkill -f burf_driver"` wrapper itself
# contains the string "burf_driver". Without the bracket trick, pkill kills its
# own shell part-way through the list and the remaining kills silently never
# run. `[b]urf_driver` is a regex that matches the literal "burf_driver" but
# NOT the pattern text on the wrapper's own command line. (This is also why the
# old version needed three passes to work at all.)
pat() { printf '[%s]%s' "${1:0:1}" "${1:1}"; }

NAV2_PATS=(controller_server smoother_server planner_server behavior_server
           bt_navigator waypoint_follower velocity_smoother
           lifecycle_manager "ros2 launch turtlebot2_bringup nav2.launch"
           "ros2 launch nav2_bringup")
DRIVER_PATS=(burf_driver rgb_compressed "ros2 launch turtlebot2_bringup burf.launch")
SLAM_PATS=(async_slam_toolbox_node odom_tf_bridge
           "ros2 launch turtlebot2_bringup slam.launch")

# --- read the single source of truth -----------------------------------------
NAV2=false
if [ -f "$NAV2_FLAG" ] && grep -qiE '^[[:space:]]*(1|true|on|yes)[[:space:]]*$' "$NAV2_FLAG"; then
    NAV2=true
fi
echo "nav2 flag ($NAV2_FLAG) => NAV2=$NAV2"

# --- wait for bringup --------------------------------------------------------
for i in $(seq 1 60); do
  docker exec "$CTR" bash -lc \
    'source /opt/ros/humble/setup.bash; ros2 node list 2>/dev/null | grep -q kobuki_node' \
    >/dev/null 2>&1 && break
  sleep 5
done

if [ "$NOW" = false ]; then
  sleep 120   # ~2 min after ros2 started: let USB / TF / scan settle before SLAM
fi

# --- tear down anything already running --------------------------------------
# Launches are detached (docker exec -d), so a systemd restart does NOT reap the
# old ones — clear them here or a restart stacks duplicates. Nav2 is in this
# list so that flipping the flag to OFF really removes Nav2, instead of leaving
# it running while the driver quietly re-arms its heartbeat.
KILL_PATS=("${NAV2_PATS[@]}" "${DRIVER_PATS[@]}")
if [ "$NOW" = false ]; then
  KILL_PATS+=("${SLAM_PATS[@]}")   # boot path: nothing worth preserving
fi
KILL_CMD=""
for p in "${KILL_PATS[@]}"; do
  KILL_CMD+="pkill -9 -f -- \"$(pat "$p")\"; "
done
docker exec "$CTR" bash -lc "$KILL_CMD true" >/dev/null 2>&1
sleep 3
docker exec "$CTR" bash -lc "$KILL_CMD true" >/dev/null 2>&1
sleep 2

# --- SLAM --------------------------------------------------------------------
# On --now, keep a live slam_toolbox: killing it throws away the map being
# built, and the toggle is about Nav2, not about the map.
if docker exec "$CTR" bash -lc "pgrep -f -- \"$(pat async_slam_toolbox_node)\" >/dev/null" >/dev/null 2>&1; then
  echo "slam_toolbox already running — left alone"
else
  echo "launching slam.launch.py"
  docker exec -d "$CTR" bash -c "$(ros 'ros2 launch turtlebot2_bringup slam.launch.py > /tmp/slam.log 2>&1')"
  sleep 20
fi

# --- Nav2 (same $NAV2 that feeds enable_nav2 below) --------------------------
if [ "$NAV2" = true ]; then
  echo "launching nav2.launch.py"
  docker exec -d "$CTR" bash -c "$(ros 'ros2 launch turtlebot2_bringup nav2.launch.py > /tmp/nav2.log 2>&1')"
  sleep 10
else
  echo "nav2 disabled — not launching"
fi

# --- Burf Platform driver ----------------------------------------------------
echo "launching burf.launch.py enable_nav2:=$NAV2"
docker exec -d "$CTR" bash -c "$(ros "ros2 launch turtlebot2_bringup burf.launch.py enable_nav2:=$NAV2 > /tmp/driver.log 2>&1")"

echo "slam + burf launched (nav2=$NAV2) at $(date)"
