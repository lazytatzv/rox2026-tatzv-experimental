# ROX2026 Tatzv Experimental - Master Makefile
# "The Strongest Way to Control Your Robot"

.PHONY: build up down shell launch virtual format clean help

# --- [ Docker Management ] ---

build: ## Build the Docker image with BuildKit and JP mirrors
	DOCKER_BUILDKIT=1 docker compose build

up: ## Start the container in background
	xhost +local:docker
	docker compose up -d

down: ## Stop and remove the container
	docker compose down

shell: ## Enter the running container
	docker compose exec ros2_rox2026 bash

# --- [ ROS 2 Operations ] ---

colcon: ## Build the workspace inside Docker
	docker compose exec ros2_rox2026 bash -c "colcon build --symlink-install --parallel-workers 2"

launch: ## Launch the robot in physical mode (Defaults to physical.yaml setting)
	docker compose exec ros2_rox2026 bash -c "source install/setup.bash && ros2 launch robot_bringup robot_bringup.launch.py"

virtual: ## Launch the robot in Virtual Mode for testing
	docker compose exec ros2_rox2026 bash -c "source install/setup.bash && ros2 launch robot_bringup robot_bringup.launch.py actuator_type:=virtual"

# --- [ Utility & Maintenance ] ---

format: ## Run the self-healing auto-formatter
	docker compose exec ros2_rox2026 bash -c "./fix_style.sh"

clean: ## Purge build artifacts (Caution: physical delete)
	rm -rf build/ install/ log/ .ccache/

help: ## Show this help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-15s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help
