#!/bin/bash
set -e

# NoVNC使う用の設定

# Configuration
VNC_DISPLAY=:1
VNC_PORT=5901
NO_VNC_PORT=6080

echo "Starting VNC server on $VNC_DISPLAY..."
# Remove old locks
rm -rf /tmp/.X1-lock /tmp/.X11-unix/X*

# Start TigerVNC on a dedicated display (:1)
# Use -fg to keep it in foreground or ensure it doesn't exit
tigervncserver $VNC_DISPLAY -geometry 1280x680 -depth 24 -localhost no -PasswordFile /root/.vnc/passwd || {
    echo "VNC server failed to start, attempting to kill existing and retry..."
    tigervncserver -kill $VNC_DISPLAY || true
    tigervncserver $VNC_DISPLAY -geometry 1280x680 -depth 24 -localhost no -PasswordFile /root/.vnc/passwd
}

# Configure Fluxbox to center windows by default
mkdir -p /root/.fluxbox
cat <<EOF > /root/.fluxbox/init
session.screen0.windowPlacement: CenterPlacement
session.screen0.fullMaximization: true
session.screen0.focusModel: ClickToFocus
EOF

# Start Window Manager for the VNC display
DISPLAY=$VNC_DISPLAY fluxbox &
sleep 2 # Give fluxbox time to start

# Start noVNC
/usr/share/novnc/utils/novnc_proxy --vnc localhost:$VNC_PORT --listen $NO_VNC_PORT > /tmp/novnc.log 2>&1 &

echo "noVNC is running at http://localhost:$NO_VNC_PORT/vnc.html (Targets VNC Display $VNC_DISPLAY)"
echo "Host GUI is available if DISPLAY is set to host's display (e.g., :0)"

# Execute CMD
exec "$@"
