#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /root/lazytatzv_ws/main_ws/install/setup.bash 2>/dev/null || true

export DISPLAY=:2
export QT_QPA_PLATFORM=xcb

echo "Starting Gazebo simulation with rendering enabled..."
# Launch the simulation and perception nodes
ros2 launch robot_bringup robot_bringup.launch.py gazebo:=true headless:=false use_sim_time:=true &
SIM_PID=$!

echo "Waiting for camera images on /camera_synced/image_raw (max 90s)..."
# We will wait up to 90 seconds
MAX_WAIT=90
WAIT_COUNT=0
IMAGE_DETECTED=false

while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
  if timeout 2 ros2 topic echo /camera_synced/image_raw --once >/dev/null 2>&1; then
    echo "Camera images detected!"
    IMAGE_DETECTED=true
    break
  else
    echo -n "."
    sleep 1
    WAIT_COUNT=$((WAIT_COUNT + 1))
  fi
done

if [ "$IMAGE_DETECTED" = "false" ]; then
  echo "Error: Failed to detect camera images."
  echo "=== ROS 2 Topic List ==="
  ros2 topic list
else
  echo "=== Checking /tf output from Gazebo + AprilTag node (timeout 15s) ==="
  # Capturing /tf output
  timeout 15 ros2 topic echo /tf
fi

echo "Cleaning up Gazebo..."
kill -INT $SIM_PID
pkill -f gazebo
pkill -f gz
pkill -f apriltag_node
pkill -f image_syncer
wait $SIM_PID 2>/dev/null
echo "Cleanup done."
