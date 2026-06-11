# ROX2026 Tatzv Experimental - Master Makefile
# "The Strongest Way to Control Your Robot"

SHELL := /bin/bash

.PHONY: build image up down shell test launch virtual sim sim-gui format clean help nix

# Determine if we are running inside the Docker container
IN_CONTAINER := $(shell [ -f /.dockerenv ] && echo "true" || echo "false")

ifeq ($(IN_CONTAINER),true)
    EXEC_PREFIX := 
else
    EXEC_PREFIX := docker compose exec ros2_rox2026
endif

nix:
	nix develop --extra-experimental-features "nix-command flakes"

# --- [ Docker Management ] ---

image: ## Build the Docker image
ifeq ($(IN_CONTAINER),true)
	@echo "🚨 Error: Cannot build image from inside the container."
	@exit 1
else
	DOCKER_BUILDKIT=1 docker compose build
endif

up: ## Start the container in background
ifeq ($(IN_CONTAINER),true)
	@echo "🚨 Error: Container is already running."
else
	xhost +local:docker > /dev/null 2>&1 || true
	docker compose up -d
endif

down: ## Stop and remove the container
ifeq ($(IN_CONTAINER),true)
	@echo "🚨 Error: Cannot stop container from within itself."
else
	docker compose down
endif

shell: ## Enter the running container
ifeq ($(IN_CONTAINER),true)
	@echo "🚀 You are already in the strongest shell."
else
	docker compose exec ros2_rox2026 /bin/zsh || docker compose exec ros2_rox2026 /bin/bash
endif

# --- [ ROS 2 Operations - DELEGATED TO main_ws/Makefile ] ---

build: ## Build the ROS 2 workspace (via main_ws/Makefile)
	$(EXEC_PREFIX) make -C main_ws build

test: ## Run tests (via main_ws/Makefile)
	$(EXEC_PREFIX) make -C main_ws test

launch: ## Launch physical mode (via main_ws/Makefile)
	$(EXEC_PREFIX) make -C main_ws launch

virtual: ## Launch virtual mode (via main_ws/Makefile)
	$(EXEC_PREFIX) make -C main_ws virtual

sim: ## Launch headless simulation (via main_ws/Makefile)
	$(EXEC_PREFIX) make -C main_ws sim

sim-gui: ## Launch simulation with GUI (via main_ws/Makefile)
	$(EXEC_PREFIX) make -C main_ws sim-gui

# --- [ Utility & Maintenance ] ---

format: ## Run the self-healing auto-formatter
	$(EXEC_PREFIX) ./fix_style.sh

clean: ## Purge build artifacts
	$(EXEC_PREFIX) make -C main_ws clean

help: ## Show this help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-15s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help
