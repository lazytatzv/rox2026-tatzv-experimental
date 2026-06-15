# ROX2026 Ultimate Docker Architecture
# Multi-stage build for Dev/Prod separation

ARG ROS_DISTRO=jazzy

# ==============================================================================
# 1. Base Stage: Common ROS 2 dependencies and runtime libraries
# ==============================================================================
FROM ros:${ROS_DISTRO}-ros-base AS base
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root/lazytatzv_ws

# Runtime Dependencies (No compilers here)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-serial-driver \
    ros-${ROS_DISTRO}-io-context \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-localization \
    tigervnc-standalone-server \
    tigervnc-common \
    tigervnc-tools \
    fluxbox \
    novnc \
    websockify \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# VNC Password Setup
RUN mkdir -p /root/.vnc && \
    (echo "password" | vncpasswd -f > /root/.vnc/passwd) && \
    chmod 600 /root/.vnc/passwd

# ==============================================================================
# 2. Builder Stage: Build tools and compilation
# ==============================================================================
FROM base AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    libboost-all-dev \
    ccache \
    && rm -rf /var/lib/apt/lists/*

# Use ccache
RUN ln -sf /usr/bin/ccache /usr/local/bin/gcc && \
    ln -sf /usr/bin/ccache /usr/local/bin/g++

# Install build-time rosdep dependencies
COPY ./main_ws/src ./main_ws/src
RUN rosdep update && \
    rosdep install --from-paths main_ws/src --ignore-src -y -r || true

# Build the workspace
COPY ./main_ws ./main_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd main_ws && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ==============================================================================
# 3. Prod Stage: Final lightweight image for the robot
# ==============================================================================
FROM base AS prod

# Copy only the built artifacts
COPY --from=builder /root/lazytatzv_ws/main_ws/install ./main_ws/install
COPY ./scripts ./scripts

# Setup entrypoint
RUN printf "%s\n" \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "source /root/lazytatzv_ws/main_ws/install/setup.bash" \
    >> /root/.bashrc

ENTRYPOINT ["./scripts/entrypoint.sh"]
CMD ["ros2", "launch", "robot_bringup", "robot_bringup.launch.py"]

# ==============================================================================
# 4. Dev Stage: Full environment for coding and debugging
# ==============================================================================
FROM builder AS dev

# Add extra dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    fish neovim nano less curl gh evtest black ros-${ROS_DISTRO}-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*

# Copy whole repo for dev (though usually mounted via volume)
COPY . .

# Setup dev bashrc
RUN printf "%s\n" \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "if [ -f /root/lazytatzv_ws/main_ws/install/setup.bash ]; then source /root/lazytatzv_ws/main_ws/install/setup.bash; fi" \
    "export PATH=/usr/lib/ccache:\$PATH" \
    >> /root/.bashrc

ENTRYPOINT ["./scripts/entrypoint.sh"]
CMD ["bash"]
