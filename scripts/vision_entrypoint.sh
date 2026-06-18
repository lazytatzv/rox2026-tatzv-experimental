#!/bin/bash
set -e

# Setup ROS 2 Humble
source /opt/ros/humble/setup.bash

# Setup TROS if available
if [ -f /opt/tros/humble/setup.bash ]; then
    source /opt/tros/humble/setup.bash
fi

# RMW Routing (Must match the main container to communicate)
if [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ]; then
    echo "▶ Vision Middleware: Zenoh"
    # export ZENOH_ROUTER_CONFIG_URI=...
else
    echo "▶ Vision Middleware: FastDDS"
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    # FastDDS needs a common profile or default settings
fi

echo "========================================================="
echo " ROX2026 Vision Container Ready (Humble/TROS)"
echo " Run 'just vision-run' to start stereo estimation."
echo "========================================================="

exec "$@"
