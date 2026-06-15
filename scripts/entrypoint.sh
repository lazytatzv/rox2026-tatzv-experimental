#!/bin/bash
set -e

# NoVNC使う用の設定

# Configuration
VNC_DISPLAY=:1
VNC_PORT=5901
NO_VNC_PORT=6080

echo "Preparing X11 environment..."
# Remove old locks
rm -rf /tmp/.X1-lock /tmp/.X11-unix/X*
mkdir -p /tmp/.X11-unix
chmod 1777 /tmp/.X11-unix

echo "Starting VNC server on $VNC_DISPLAY..."
# Using -localhost no to allow proxy access
# Using -SecurityTypes None for simpler testing (Password still checked by noVNC if desired)
tigervncserver $VNC_DISPLAY -geometry 1280x680 -depth 24 -localhost no -PasswordFile /root/.vnc/passwd > /tmp/vnc_startup.log 2>&1

# Configure Fluxbox to center windows by default
mkdir -p /root/.fluxbox
cat <<EOF > /root/.fluxbox/init
session.screen0.windowPlacement: CenterPlacement
session.screen0.fullMaximization: true
session.screen0.focusModel: ClickToFocus
EOF

# Start Window Manager for the VNC display
DISPLAY=$VNC_DISPLAY fluxbox > /tmp/fluxbox.log 2>&1 &
sleep 2 # Give fluxbox time to start

# Start noVNC
/usr/share/novnc/utils/novnc_proxy --vnc localhost:$VNC_PORT --listen $NO_VNC_PORT > /tmp/novnc.log 2>&1 &

echo "noVNC is running at http://localhost:$NO_VNC_PORT/vnc.html"
echo "---------------------------------------------------------"
cat /tmp/vnc_startup.log
echo "---------------------------------------------------------"

# Execute CMD
exec "$@"
