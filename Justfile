# Master Justfile - Docker & Environment Management

export DOCKER_BUILD_TARGET := "dev"

# OS Detection & Docker Network Mode
os := `uname -s`
has_nvidia := `command -v nvidia-smi > /dev/null 2>&1 && echo yes || echo no`

docker_network_mode := if os == "Linux" { "host" } else { "bridge" }

# Compose Files
compose_files := if has_nvidia == "yes" { "-f docker/compose.yaml -f docker/compose.gpu.yaml" } else { "-f docker/compose.yaml -f docker/compose.null.yaml" }

@default:
    just --list

# --- [ Environment ] ---

# Setup direnv hooks
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

# Switch RMW to FastDDS
use-fastdds:
    @sed -i '' '/RMW_IMPLEMENTATION/d' .env 2>/dev/null || sed -i '/RMW_IMPLEMENTATION/d' .env
    @echo "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> .env
    @echo "Switched to FastDDS. Restart container to apply."

# Switch RMW to Zenoh
use-zenoh:
    @sed -i '' '/RMW_IMPLEMENTATION/d' .env 2>/dev/null || sed -i '/RMW_IMPLEMENTATION/d' .env
    @echo "RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> .env
    @echo "Switched to Zenoh. Restart container to apply."

# --- [ Docker ] ---

# Start development container
up:
    @xhost +local:docker > /dev/null 2>&1 || true
    DOCKER_NETWORK_MODE={{docker_network_mode}} docker compose {{compose_files}} up -d

# Stop container
down:
    docker compose {{compose_files}} down

# Enter running container
shell:
    docker compose {{compose_files}} exec ros2_rox2026 /bin/bash

# Enter vision container
vision-shell:
    docker compose {{compose_files}} exec ros2_vision /bin/bash

# Start stereo vision algorithm in vision container
vision-run:
    docker compose {{compose_files}} exec ros2_vision bash -c 'source /opt/tros/humble/setup.bash && ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4_int16.launch.py use_mipi_cam:=True mipi_rotation:=0.0'

# --- [ Helpers ] ---
_ensure-up:
    @if ! docker compose {{compose_files}} ps | grep -q "ros2_rox2026"; then \
        echo "Container is not running. Starting it now..."; \
        just up; \
    fi

# --- [ ROS 2 (Forwarded to main_ws) ] ---

# Build workspace
build: _ensure-up
    docker compose {{compose_files}} exec ros2_rox2026 just -f main_ws/Justfile build

# Launch headless simulation
sim: _ensure-up
    docker compose {{compose_files}} exec ros2_rox2026 just -f main_ws/Justfile sim

# Launch simulation with GUI
sim-gui: _ensure-up
    docker compose {{compose_files}} exec ros2_rox2026 just -f main_ws/Justfile sim-gui
