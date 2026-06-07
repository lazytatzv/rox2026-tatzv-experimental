# ROX2026 Tatzv Experimental - Master Makefile
# "The Strongest Way to Control Your Robot"

SHELL := /bin/bash

.PHONY: build up down shell colcon launch virtual format clean help

# Determine if we are running inside the Docker container
IN_CONTAINER := $(shell [ -f /.dockerenv ] && echo "true" || echo "false")

ifeq ($(IN_CONTAINER),true)
    # Commands when inside container
    EXEC_PREFIX := 
    BASH_PREFIX := bash -c
else
    # Commands when on host
    EXEC_PREFIX := docker compose exec ros2_rox2026
    BASH_PREFIX := bash -c
endif

# --- [ Docker Management ] ---

build: ## Build the Docker image
	DOCKER_BUILDKIT=1 docker compose build

up: ## Start the container in background
	xhost +local:docker > /dev/null 2>&1 || true
	docker compose up -d

down: ## Stop and remove the container
	docker compose down

shell: ## Enter the running container
	docker compose exec ros2_rox2026 /bin/zsh || docker compose exec ros2_rox2026 /bin/bash

# --- [ ROS 2 Operations - CONTEXT AWARE ] ---

colcon: ## [OPTIMIZED] Build the workspace
	$(EXEC_PREFIX) bash -c "make -C main_ws build"

launch: ## Launch the robot in physical mode
	$(EXEC_PREFIX) bash -c "source main_ws/install/setup.bash && ros2 launch robot_bringup robot_bringup.launch.py"

virtual: ## Launch the robot in Virtual Mode for testing
	$(EXEC_PREFIX) bash -c "source main_ws/install/setup.bash && ros2 launch robot_bringup robot_bringup.launch.py actuator_type:=virtual"

sim: ## [EXPERIMENTAL] Launch Gazebo physical simulation
	$(EXEC_PREFIX) bash -c "source main_ws/install/setup.bash && ros2 launch robot_bringup robot_bringup.launch.py gazebo:=true"

# --- [ Utility & Maintenance ] ---

format: ## Run the self-healing auto-formatter
	$(EXEC_PREFIX) bash -c "./fix_style.sh"

clean: ## Purge build artifacts
	rm -rf main_ws/build/ main_ws/install/ main_ws/log/ .ccache/

help: ## Show this help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-15s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help
