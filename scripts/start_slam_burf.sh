#!/bin/bash
# start_slam_burf.sh — SLAM (+ optionally Nav2) + the Burf Platform driver.
#
# Run at boot by turtlebot2-slam-burf.service, AFTER turtlebot2-burf.service has
# recreated the container and started the bringup launch.
#
# ============================================================================
# Nav2 has been REMOVED from this robot. It cost 78% of a CPU core sitting
# idle (measured) while slam_toolbox costs 4.5%, and the built-in A* +
# GoToController planner does the same job at no measurable cost. The
# NAV2_PATS kill list below is kept only to reap strays from older boots.
set -u

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
# SLAM node name substring, NOT the full executable name. Localization mode
# runs localization_slam_toolbox_node while mapping runs async_slam_toolbox_node;
# matching only the async one meant a live localization SLAM was invisible to
# both the teardown below and the "already running" check further down, so a
# --now run launched a SECOND (mapping) slam_toolbox on top of it. Two SLAMs
# both publishing /map and map->odom is the "map going crazy" failure.
SLAM_PATS=(slam_toolbox_node odom_tf_bridge
           "ros2 launch turtlebot2_bringup slam.launch")

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
# NAV2_PATS is retained deliberately even though Nav2 is gone: it reaps any
# stray Nav2 process left over from an older boot, which would otherwise fight
# the planner for /cmd_vel.
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
if docker exec "$CTR" bash -lc "pgrep -f -- \"$(pat slam_toolbox_node)\" >/dev/null" >/dev/null 2>&1; then
  echo "slam_toolbox already running — left alone"
else
  echo "launching slam.launch.py"
  docker exec -d "$CTR" bash -c "$(ros 'ros2 launch turtlebot2_bringup slam.launch.py > /tmp/slam.log 2>&1')"
  sleep 20
fi


# --- Burf Platform driver ----------------------------------------------------
echo "launching burf.launch.py"
docker exec -d "$CTR" bash -c "$(ros "ros2 launch turtlebot2_bringup burf.launch.py > /tmp/driver.log 2>&1")"

echo "slam + burf launched at $(date)"
