#!/bin/bash
cd main_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || true
python3 src/control_analysis/control_analysis/analysis_cli.py step_test_bag
