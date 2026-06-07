# ROS 2 Workspace Helper Makefile
# Use this INSIDE the container for fast development cycles.

.PHONY: build test result clean help

build: ## [OPTIMIZED] Build all packages with symlink-install
	colcon build \
		--symlink-install \
		--parallel-workers $$(nproc) \
		--cmake-args -DCMAKE_BUILD_TYPE=Release

test: ## Run tests for all packages (including the Official Kinematics Test)
	colcon test --event-handlers console_direct+

test-kinematics: ## Specifically run the Official Kinematics Validation
	colcon test --packages-select mecanum_kinematics --event-handlers console_direct+
	colcon test-result --verbose --packages-select mecanum_kinematics

result: ## Show detailed test results
	colcon test-result --verbose

clean: ## Purge all build artifacts in this workspace
	rm -rf build/ install/ log/

help: ## Show this help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-15s\033[0m %s\n", $$1, $$2}'

launch:
	ros2 launch robot_bringup robot_bringup.launch.py

.DEFAULT_GOAL := help
