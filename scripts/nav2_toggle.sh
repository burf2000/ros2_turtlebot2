#!/bin/bash
# nav2_toggle.sh — turn Nav2 on or off on the TurtleBot2, and apply it now.
#
#   sudo /home/burf2000/nav2_toggle.sh on
#   sudo /home/burf2000/nav2_toggle.sh off
#        /home/burf2000/nav2_toggle.sh status     (no sudo needed)
#
# Writes the SINGLE SOURCE OF TRUTH, /etc/turtlebot2/nav2.enabled, then re-runs
# start_slam_burf.sh --now so the change takes effect without a reboot. The flag
# is on the HOST, not in the container, because start_turtlebot2.sh destroys and
# recreates the container from the image on every boot — anything written inside
# the container is gone at the next power cycle.
#
# One flag, two consequences (see start_slam_burf.sh for the full reasoning):
#   on  -> Nav2 launches AND the driver gets enable_nav2:=true, which stands its
#          10 Hz /cmd_vel deadman heartbeat down so Nav2 can own the base.
#   off -> Nav2 is killed AND the driver gets enable_nav2:=false, heartbeat back.
# Never set one without the other. Nav2 off with the heartbeat also off leaves
# the robot with nothing driving it and no deadman.
#
# Takes ~45 s (SLAM is left running; Nav2 + driver are restarted).
# The setting survives reboot: turtlebot2-slam-burf.service re-reads the flag.

set -eu

FLAG=/etc/turtlebot2/nav2.enabled
START=/home/burf2000/start_slam_burf.sh
CTR=turtlebot2_bringup

usage() { echo "usage: $0 {on|off|status}" >&2; exit 2; }
[ $# -eq 1 ] || usage

read_flag() {
  if [ -f "$FLAG" ] && grep -qiE '^[[:space:]]*(1|true|on|yes)[[:space:]]*$' "$FLAG"; then
    echo true
  else
    echo false
  fi
}

case "$1" in
  status)
    echo "flag file : $FLAG -> $(read_flag)"
    echo -n "nav2 nodes: "
    # -f with the [b]racket trick so pgrep does not match its own wrapper.
    docker exec "$CTR" bash -lc 'pgrep -cf -- "[b]t_navigator"' 2>/dev/null \
      | tr -d '\r' | sed 's/^0$/0 (not running)/'
    echo -n "driver    : "
    docker exec "$CTR" bash -lc \
      'source /opt/ros/humble/setup.bash; timeout 20 ros2 param get /burf_driver enable_nav2' \
      2>/dev/null || echo "(driver not answering)"
    ;;
  on|off)
    [ "$(id -u)" -eq 0 ] || { echo "need root to write $FLAG — use sudo" >&2; exit 1; }
    mkdir -p "$(dirname "$FLAG")"
    if [ "$1" = on ]; then echo true > "$FLAG"; else echo false > "$FLAG"; fi
    echo "wrote $FLAG = $(cat "$FLAG")"
    echo "applying (restarting nav2 + driver; slam left alone)..."
    "$START" --now
    ;;
  *) usage ;;
esac
