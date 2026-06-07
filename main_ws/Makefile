# Copyright 2026 Tatsukiyano
# ROS 2 Workspace Internal Makefile

.PHONY: build test clean help

build: ## [OPTIMIZED] Build all packages
	colcon build \
		--symlink-install \
		--parallel-workers $$(nproc || echo 2) \
		--cmake-args -DCMAKE_BUILD_TYPE=Release

test: ## Run all tests
	colcon test --event-handlers console_direct+

clean: ## Purge artifacts
	rm -rf build/ install/ log/

help: ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-15s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help
