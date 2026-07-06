#!/bin/bash
source /opt/ros/jazzy/setup.bash
source main_ws/install/setup.bash

echo "Starting perception node..."
ros2 launch main_ws/src/bringup/robot_bringup/launch/include/perception.launch.py > /dev/null 2>&1 &
PERC_PID=$!

echo "Starting tag publisher..."
python3 publish_tag.py &
PUB_PID=$!

sleep 5
echo "--- DETECTIONS OUTPUT ---"
ros2 topic echo /detections --once

kill -9 $PERC_PID
kill -9 $PUB_PID
