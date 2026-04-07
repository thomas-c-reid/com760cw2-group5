#!/bin/bash
set -e

# ── Kill any stale VNC sessions ───────────────────────────────────────────────
vncserver -kill :1 2>/dev/null || true
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true

# ── Start VNC server ─────────────────────────────────────────────────────────
# Run WITHOUT -fg so vncserver daemonises itself and creates the X socket
vncserver :1 \
    -geometry 1920x1080 \
    -depth 24 \
    -SecurityTypes VncAuth \
    -localhost no

# Wait for the X socket to actually appear
echo "[setup] Waiting for VNC server to start..."
for i in $(seq 1 15); do
    if [ -e /tmp/.X11-unix/X1 ]; then
        echo "[setup] VNC server is up (display :1)"
        break
    fi
    sleep 1
done

if [ ! -e /tmp/.X11-unix/X1 ]; then
    echo "[ERROR] VNC server failed to start. Check logs:"
    cat /root/.vnc/*.log 2>/dev/null || true
    exit 1
fi

# ── Start noVNC web proxy (browser access) ────────────────────────────────────
websockify \
    --web=/usr/share/novnc/ \
    --wrap-mode=ignore \
    0.0.0.0:8080 \
    localhost:5901 &

echo "[setup] noVNC proxy started on port 8080"

# ── Source ROS and initialise workspace ──────────────────────────────────────
source /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/ros_ws/src:$GAZEBO_MODEL_PATH

cd /ros_ws

# Initialise catkin workspace on first run
if [ ! -f /ros_ws/src/CMakeLists.txt ]; then
    echo "[setup] Initialising catkin workspace..."
    catkin_init_workspace src
fi

# Build if any packages are present
if ls /ros_ws/src/*/package.xml 1>/dev/null 2>&1; then
    echo "[setup] Building catkin workspace..."
    catkin_make
fi

[ -f /ros_ws/devel/setup.bash ] && source /ros_ws/devel/setup.bash

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║   COM760 CW2 — ROS Noetic + Gazebo Ready            ║"
echo "║                                                      ║"
echo "║   Open in browser:                                   ║"
echo "║   http://localhost:8080/vnc.html                     ║"
echo "║                                                      ║"
echo "║   Or connect VNC client to: localhost:5901           ║"
echo "║   VNC password: ros_docker                          ║"
echo "║                                                      ║"
echo "║   Workspace: /ros_ws/src  (mounted from ./src)      ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# Keep container alive
tail -f /dev/null
