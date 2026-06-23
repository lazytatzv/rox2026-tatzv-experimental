# Master Justfile - Docker & Environment Management

export DOCKER_BUILD_TARGET := "dev"

# OS Detection & Docker Network Mode
os := `uname -s`
has_nvidia := `command -v nvidia-smi > /dev/null 2>&1 && echo yes || echo no`

docker_network_mode := if os == "Linux" { "host" } else { "bridge" }

# Compose Files
compose_files := if has_nvidia == "yes" { "-f docker/compose.yaml -f docker/compose.gpu.yaml" } else { "-f docker/compose.yaml -f docker/compose.null.yaml" }

# Compose Profiles
# main, rdk
profile := "--profile main"


@default:
    just --list

# --- [ Environment Setup ] ---

# Setup direnv hooks for shell integration
setup-env:
    @echo "Configuring direnv for Bash..."
    @grep -q 'direnv hook bash' ~/.bashrc || echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
    @if [ -d ~/.config/fish ]; then \
        echo "Configuring direnv for Fish..."; \
        grep -q 'direnv hook fish' ~/.config/fish/config.fish || echo 'direnv hook fish | source' >> ~/.config/fish/config.fish; \
    fi
    @echo "Allowing current directory in direnv..."
    @direnv allow .
    @echo ">>> Environment setup complete. Please restart your shell <<<"

# Switch ROS 2 middleware to FastDDS
use-fastdds:
    @sed -i '' '/RMW_IMPLEMENTATION/d' .env 2>/dev/null || sed -i '/RMW_IMPLEMENTATION/d' .env
    @echo "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> .env
    @echo "Switched to FastDDS. Restart container to apply."

# Switch ROS 2 middleware to Zenoh
use-zenoh:
    @sed -i '' '/RMW_IMPLEMENTATION/d' .env 2>/dev/null || sed -i '/RMW_IMPLEMENTATION/d' .env
    @echo "RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> .env
    @echo "Switched to Zenoh. Restart container to apply."


# --- [ Docker Control ] ---

# Start development container (auto-enabling local xhost access)
up:
    @xhost +local:docker > /dev/null 2>&1 || true
    DOCKER_NETWORK_MODE={{docker_network_mode}} docker compose {{profile}} {{compose_files}} up -d

# Build Docker images
build:
    DOCKER_BUILDKIT=1 DOCKER_BUILD_TARGET=dev IMAGE_TAG=dev docker compose {{profile}} {{compose_files}} build

# Start Foxglove Bridge for Web/App UI
foxglove:
    docker exec -it rox2026_container bash -c "source /opt/ros/jazzy/setup.bash && ros2 run foxglove_bridge foxglove_bridge"

# Stop container
down:
    docker compose {{profile}} {{compose_files}} down

# Enter running container shell
shell:
    docker compose {{profile}} {{compose_files}} exec ros2_rox2026 /bin/bash


# --- [ ROS 2 Workspace (Executed inside Container) ] ---

# Compile all ROS 2 packages in workspace
colcon:
    docker compose {{profile}} {{compose_files}} exec ros2_rox2026 just -f main_ws/Justfile build

# Run all ROS 2 workspace tests
test:
    docker compose {{profile}} {{compose_files}} exec ros2_rox2026 just -f main_ws/Justfile test

# Clean all build artifacts (build/, install/, log/)
clean:
    docker compose {{profile}} {{compose_files}} exec ros2_rox2026 just -f main_ws/Justfile clean

# Auto-format codebase (Black & Uncrustify)
format:
    docker compose {{profile}} {{compose_files}} exec ros2_rox2026 bash -c "scripts/fix_style.sh"


# --- [ Simulation & Robot Control (Executed inside Container) ] ---

# Launch headless Gazebo simulation
sim:
    docker compose {{profile}} {{compose_files}} exec ros2_rox2026 just -f main_ws/Justfile sim

# Launch Gazebo simulation with GUI (noVNC display :2)
sim-gui:
    docker compose {{profile}} {{compose_files}} exec ros2_rox2026 just -f main_ws/Justfile sim-gui

# Launch physical robot nodes
launch:
    docker compose {{profile}} {{compose_files}} exec ros2_rox2026 just -f main_ws/Justfile launch

# Launch minimal teleop-only (no sensors, no Nav2, just mecanum + joystick)
teleop:
    docker compose {{profile}} {{compose_files}} exec ros2_rox2026 just -f main_ws/Justfile teleop

# Run automated control engineering analysis (mode can be step, sine, chirp, auto)
analyze-control mode="auto" bag="control_analysis_bag" report="full_analysis_report":
    docker compose {{profile}} {{compose_files}} exec ros2_rox2026 just -f main_ws/Justfile analyze-control {{mode}} {{bag}} {{report}}

# Alias for README compatibility
report mode="auto" bag="control_analysis_bag" report_name="full_analysis_report":
    just analyze-control {{mode}} {{bag}} {{report_name}}
