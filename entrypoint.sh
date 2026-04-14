#!/bin/bash
set -e

# Source ROS setup for all commands launched via entrypoint.
source /opt/ros/humble/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Start virtual display (Xvfb) so RViz2/Gazebo don't need XQuartz
Xvfb :1 -screen 0 1920x1080x24 -ac +extension GLX +render -noreset &
XVFB_PID=$!

# Wait for Xvfb to be ready
for i in $(seq 1 10); do
    xdpyinfo -display :1 > /dev/null 2>&1 && break
    sleep 0.5
done

# Start VNC server on port 5900 (no password for local dev)
x11vnc -display :1 -nopw -listen 0.0.0.0 -xkb -forever -shared -bg -o /tmp/x11vnc.log

# Start noVNC web client on port 8080
websockify --web /usr/share/novnc 8080 localhost:5900 &

# noVNC/browser sessions can leave Alt in a stuck state, which makes Openbox
# treat clicks as window-move actions. Remap Alt-mouse bindings to Super.
OPENBOX_CFG_DIR="$HOME/.config/openbox"
OPENBOX_CFG_FILE="$OPENBOX_CFG_DIR/rc.xml"
if [ -f /etc/xdg/openbox/rc.xml ]; then
    mkdir -p "$OPENBOX_CFG_DIR"
    cp /etc/xdg/openbox/rc.xml "$OPENBOX_CFG_FILE"
    sed -i 's/button="A-/button="W-/g' "$OPENBOX_CFG_FILE"
fi

# Start window manager so windows can be moved/resized
openbox &

export ROS_DOMAIN_ID=0

# Echo helpful info
echo "=== ROS 2 Humble Drone Simulation ==="
echo "Workspace: /ros2_ws"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""
echo "Available commands:"
echo "  ros2 launch drone_simulation simulation.launch.py"
echo "  ros2 run drone_simulation position_controller"
echo "  rviz2"
echo ""

# Execute whatever command was passed
exec "$@"