# ROX2026 Tatzv Experimental - Master Makefile
# "The Strongest Way to Control Your Robot"

.PHONY: build up down shell colcon launch virtual format clean help

# --- [ Docker Management ] ---

build: ## Build the Docker image with host network for robust DNS resolution
	DOCKER_BUILDKIT=1 docker compose build --build-arg ROS_DISTRO=jazzy

up: ## Start the container in background
	xhost +local:docker > /dev/null 2>&1 || true
	docker compose up -d

down: ## Stop and remove the container
	docker compose down

shell: ## Enter the running container
	docker compose exec ros2_rox2026 bash

# --- [ ROS 2 Operations - OPTIMIZED ] ---

colcon: ## [OPTIMIZED] Build the workspace with high-performance flags
	docker compose exec ros2_rox2026 bash -c "\
		cd main_ws && \
		colcon build \
			--symlink-install \
			--parallel-workers $(shell nproc || echo 2) \
			--cmake-args \
				-DCMAKE_BUILD_TYPE=Release \
				-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
			--event-handlers desktop_notification- status- \
		"

launch: ## Launch the robot in physical mode
	docker compose exec ros2_rox2026 bash -c "source main_ws/install/setup.bash && ros2 launch robot_bringup robot_bringup.launch.py"

virtual: ## Launch the robot in Virtual Mode for testing
	docker compose exec ros2_rox2026 bash -c "source main_ws/install/setup.bash && ros2 launch robot_bringup robot_bringup.launch.py actuator_type:=virtual"

# --- [ Utility & Maintenance ] ---

format: ## Run the self-healing auto-formatter
	docker compose exec ros2_rox2026 bash -c "./fix_style.sh"

clean: ## Purge build artifacts (Caution: physical delete)
	rm -rf main_ws/build/ main_ws/install/ main_ws/log/ .ccache/

help: ## Show this help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-15s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help
