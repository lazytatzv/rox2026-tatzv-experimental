#!/bin/bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "=== /cmd_vel ==="
ros2 topic info /cmd_vel -v
echo "=== /cmd_vel_ext ==="
ros2 topic info /cmd_vel_ext -v
echo "=== /cmd_vel_teleop ==="
ros2 topic info /cmd_vel_teleop -v
echo "=== /mecanum_drive_controller/reference ==="
ros2 topic info /mecanum_drive_controller/reference -v
