# ROS 2 Workspace Makefile - Build & Execution Logic
# This file is intended to be run INSIDE the container.

SHELL := /bin/bash

# Base ROS environment
ROS_SETUP := /opt/ros/jazzy/setup.bash
# Workspace setup (only if exists)
WS_SETUP := source install/setup.bash 2>/dev/null || true

.PHONY: help build test clean launch virtual sim sim-gui

help: ## Show this help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-15s\033[0m %s\n", $$1, $$2}'

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

sim: ## Launch Gazebo physical simulation (Headless)
	source $(ROS_SETUP) && \
	$(WS_SETUP) && \
	ros2 launch robot_bringup robot_bringup.launch.py gazebo:=true headless:=true

sim-gui: ## Launch Gazebo physical simulation with GUI
	source $(ROS_SETUP) && \
	$(WS_SETUP) && \
	ros2 launch robot_bringup robot_bringup.launch.py gazebo:=true headless:=false

clean: ## Purge artifacts
	rm -rf build/ install/ log/

.DEFAULT_GOAL := help
