#!/usr/bin/env bash
set -e

# Pick a free X display even after an unclean `docker restart`, which may leave
# a stale lock/socket in the container's writable layer.
display_number=1
while [ -e "/tmp/.X${display_number}-lock" ] \
    || [ -e "/tmp/.X11-unix/X${display_number}" ]; do
  display_number=$((display_number + 1))
done
export DISPLAY=":${display_number}"

Xvfb "$DISPLAY" -screen 0 1600x900x24 +extension GLX +render -noreset &
xvfb_pid=$!

for attempt in $(seq 1 30); do
  if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    break
  fi
  sleep 0.2
done

if ! kill -0 "$xvfb_pid" >/dev/null 2>&1; then
  echo "Xvfb failed to start on ${DISPLAY}" >&2
  exit 1
fi

fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display "$DISPLAY" -forever -shared -nopw -rfbport 5900 \
  >/tmp/x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc 6080 localhost:5900 \
  >/tmp/novnc.log 2>&1 &

source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

cleanup() {
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  kill "$xvfb_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

exec ros2 launch aovp_ekf_slam simulation.launch.py \
  rviz:="${AOVP_RVIZ:-true}" gz_gui:="${AOVP_GZ_GUI:-true}"
