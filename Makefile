# Master Makefile - Docker & Environment Management

SHELL := /bin/bash

# Configuration
CONTAINER_NAME := ros2_rox2026

.PHONY: help up down shell build test launch virtual sim sim-gui format clean nix image

help: ## Show this help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-15s\033[0m %s\n", $$1, $$2}'

# --- [ Environment & Docker ] ---

nix: ## Enter Nix development shell
	nix develop --extra-experimental-features "nix-command flakes"

image: ## Build the Docker image
	DOCKER_BUILDKIT=1 docker compose build

up: ## Start the container in background
	xhost +local:docker > /dev/null 2>&1 || true
	docker compose up -d

down: ## Stop and remove the container
	docker compose down

shell: ## Enter the running container
	docker compose exec $(CONTAINER_NAME) /bin/zsh || docker compose exec $(CONTAINER_NAME) /bin/bash

# --- [ ROS 2 Commands (Forwarded to main_ws/Makefile) ] ---

build: ## Build the ROS 2 workspace
	docker compose exec $(CONTAINER_NAME) make -C main_ws build

test: ## Run tests
	docker compose exec $(CONTAINER_NAME) make -C main_ws test

launch: ## Launch physical mode
	docker compose exec $(CONTAINER_NAME) make -C main_ws launch

virtual: ## Launch virtual mode
	docker compose exec $(CONTAINER_NAME) make -C main_ws virtual

sim: ## Launch headless simulation
	docker compose exec $(CONTAINER_NAME) make -C main_ws sim

sim-gui: ## Launch simulation with GUI
	docker compose exec $(CONTAINER_NAME) make -C main_ws sim-gui

format: ## Run the auto-formatter
	docker compose exec $(CONTAINER_NAME) ./fix_style.sh

clean: ## Purge build artifacts
	docker compose exec $(CONTAINER_NAME) make -C main_ws clean

.DEFAULT_GOAL := help
