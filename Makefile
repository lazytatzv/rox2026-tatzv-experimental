# Master Makefile - Docker & Environment Management

SHELL := /bin/bash

# Configuration
CONTAINER_NAME := rox2026_container
export DOCKER_BUILD_TARGET ?= dev

# OS Detection & Docker Network Mode
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
	export DOCKER_NETWORK_MODE ?= host
	# Detect NVIDIA GPU
	HAS_NVIDIA := $(shell command -v nvidia-smi > /dev/null 2>&1 && echo yes || echo no)
else
	export DOCKER_NETWORK_MODE ?= bridge
	HAS_NVIDIA := no
endif

# Compose Files
COMPOSE_FILES := -f compose.yaml
ifeq ($(HAS_NVIDIA),yes)
	# Update .env for Docker Compose and Dev Containers
	_DUMMY := $(shell echo "COMPOSE_GPU_FILE=compose.gpu.yaml" > .env)
else
	# Update .env for Docker Compose and Dev Containers
	_DUMMY := $(shell echo "COMPOSE_GPU_FILE=compose.null.yaml" > .env)
endif

.PHONY: setup-env

setup-env: ## [HOST] Setup direnv hooks for Bash/Fish and allow this project
	@echo "Configuring direnv for Bash..."
	@grep -q 'direnv hook bash' ~/.bashrc || echo 'eval "$$(direnv hook bash)"' >> ~/.bashrc
	@if [ -d ~/.config/fish ]; then \
		echo "Configuring direnv for Fish..."; \
		grep -q 'direnv hook fish' ~/.config/fish/config.fish || echo 'direnv hook fish | source' >> ~/.config/fish/config.fish; \
	fi
	@echo "Allowing current directory in direnv..."
	@direnv allow .
	@echo ">>> Environment setup complete. Please restart your shell or run 'exec \$$SHELL' <<<"

help: ## Show this help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-15s\033[0m %s\n", $$1, $$2}'

# --- [ Environment & Docker ] ---

nix: ## Enter Nix development shell
	nix develop --extra-experimental-features "nix-command flakes"

pull: ## Pull the latest images from the registry
	docker compose $(COMPOSE_FILES) pull

update: ## Update the repository and pull the latest images
	git pull --no-edit
	docker compose $(COMPOSE_FILES) pull

image: ## Build the development Docker image
	DOCKER_BUILDKIT=1 DOCKER_BUILD_TARGET=dev docker compose $(COMPOSE_FILES) build

prod-image: ## Build the production Docker image (Lightweight)
	DOCKER_BUILDKIT=1 DOCKER_BUILD_TARGET=prod docker compose $(COMPOSE_FILES) build

up: ## Start the development container
	xhost +local:docker > /dev/null 2>&1 || true
	DOCKER_NETWORK_MODE=$(DOCKER_NETWORK_MODE) DOCKER_BUILD_TARGET=dev docker compose $(COMPOSE_FILES) up -d

prod-up: ## Start the production container
	xhost +local:docker > /dev/null 2>&1 || true
	DOCKER_NETWORK_MODE=$(DOCKER_NETWORK_MODE) DOCKER_BUILD_TARGET=prod docker compose $(COMPOSE_FILES) up -d

down: ## Stop and remove the container
	docker compose $(COMPOSE_FILES) down

shell: ## Enter the running container
	docker compose $(COMPOSE_FILES) exec $(CONTAINER_NAME) /bin/zsh || docker compose $(COMPOSE_FILES) exec $(CONTAINER_NAME) /bin/bash

# --- [ ROS 2 Commands (Forwarded to main_ws/Makefile) ] ---

build: ## Build the ROS 2 workspace
	docker compose $(COMPOSE_FILES) exec $(CONTAINER_NAME) make -C main_ws build

test: ## Run tests
	docker compose $(COMPOSE_FILES) exec $(CONTAINER_NAME) make -C main_ws test

launch: ## Launch physical mode
	docker compose $(COMPOSE_FILES) exec $(CONTAINER_NAME) make -C main_ws launch

virtual: ## Launch virtual mode
	docker compose $(COMPOSE_FILES) exec $(CONTAINER_NAME) make -C main_ws virtual

sim: ## Launch headless simulation
	docker compose $(COMPOSE_FILES) exec $(CONTAINER_NAME) make -C main_ws sim

sim-gui: ## Launch simulation with GUI
	docker compose $(COMPOSE_FILES) exec $(CONTAINER_NAME) make -C main_ws sim-gui

format: ## Run the auto-formatter
	docker compose $(COMPOSE_FILES) exec $(CONTAINER_NAME) ./scripts/fix_style.sh

clean: ## Purge build artifacts
	docker compose $(COMPOSE_FILES) exec $(CONTAINER_NAME) make -C main_ws clean

.DEFAULT_GOAL := help
