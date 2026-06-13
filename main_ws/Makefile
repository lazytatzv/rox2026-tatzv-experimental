# ROS 2 Workspace Makefile - Build & Execution Logic
# This file is intended to be run INSIDE the container.

SHELL := /bin/bash

# Base ROS environment
ROS_SETUP := /opt/ros/jazzy/setup.bash
# Workspace setup (only if exists)
WS_SETUP := source install/setup.bash 2>/dev/null || true

.PHONY: help build test clean launch virtual sim sim-gui analysis injector-chirp injector-step rviz

help: ## Show this help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

build: ## Build all packages
	source $(ROS_SETUP) && \
	colcon build \
		--symlink-install \
		--parallel-workers $$(nproc || echo 2) \
		--cmake-args -DCMAKE_BUILD_TYPE=Release

test: ## Run all tests
	source $(ROS_SETUP) && \
	$(WS_SETUP) && \
	colcon test --event-handlers console_direct+

launch: ## Launch the robot in physical mode
	source $(ROS_SETUP) && \
	$(WS_SETUP) && \
	ros2 launch robot_bringup robot_bringup.launch.py

virtual: ## Launch the robot in Virtual Mode
	source $(ROS_SETUP) && \
	$(WS_SETUP) && \
	ros2 launch robot_bringup robot_bringup.launch.py actuator_type:=virtual

analysis: ## Launch Lab Mode: High-fidelity virtual physics for control analysis
	source $(ROS_SETUP) && \
	$(WS_SETUP) && \
	ros2 launch robot_bringup robot_bringup.launch.py gazebo:=false actuator_type:=virtual

rviz: ## Launch RViz2 for visualization
	source $(ROS_SETUP) && \
	$(WS_SETUP) && \
	ros2 run rviz2 rviz2

injector-chirp: ## Inject Chirp signal (0.1Hz to 10Hz) for frequency response
	source $(ROS_SETUP) && \
	$(WS_SETUP) && \
	ros2 run control_analysis signal_injector --ros-args -p mode:=chirp -p frequency_start:=0.1 -p frequency_end:=10.0 -p duration:=10.0

injector-step: ## Inject Step signal (Amplitude 1.0) for time response
	source $(ROS_SETUP) && \
	$(WS_SETUP) && \
	ros2 run control_analysis signal_injector --ros-args -p mode:=step -p amplitude:=1.0 -p duration:=5.0

report: ## [ULTIMATE] Run full automated analysis and generate Bode/Nyquist/Step report
	rm -rf auto_test_bag/
	source $(ROS_SETUP) && $(WS_SETUP) && \
	ros2 launch robot_bringup robot_bringup.launch.py gazebo:=true headless:=true & \
	LAUNCH_PID=$$! ; \
	sleep 15 ; \
	source $(ROS_SETUP) && $(WS_SETUP) && \
	ros2 bag record -o auto_test_bag --storage sqlite3 --qos-profile-overrides-path qos_overrides.yaml -a & \
	BAG_PID=$$! ; \
	sleep 5 ; \
	source $(ROS_SETUP) && $(WS_SETUP) && \
	ros2 run control_analysis auto_analyzer --ros-args -p use_sim_time:=true ; \
	echo "Test sequence finished. Cleaning up..." ; \
	pkill -INT -f "ros2 bag record" || true ; \
	pkill -INT -f "gz sim" || true ; \
	pkill -INT -P $$LAUNCH_PID || true ; \
	sleep 5 ; \
	python3 src/control_analysis/control_analysis/analysis_cli.py auto_test_bag ; \
	echo ">>> FULL REPORT GENERATED: control_report.png <<<"

sim: ## Launch Gazebo physical simulation (Headless)
	source $(ROS_SETUP) && \
	$(WS_SETUP) && \
	ros2 launch robot_bringup robot_bringup.launch.py gazebo:=true headless:=true

sim-gui: ## Launch Gazebo physical simulation with GUI
	-pkill -9 -f "gz sim"
	-pkill -9 -f "ruby"
	source $(ROS_SETUP) && \
	$(WS_SETUP) && \
	ros2 launch robot_bringup robot_bringup.launch.py gazebo:=true headless:=false

clean: ## Purge artifacts
	rm -rf build/ install/ log/

.DEFAULT_GOAL := help
