ARG ROS_DISTRO=jazzy

# ==============================================================================
# 1. Base Stage: Common ROS 2 runtime libraries
# ==============================================================================
FROM ros:${ROS_DISTRO}-ros-base AS base
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root/lazytatzv_ws

# Common Runtime Dependencies
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
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-ros2-socketcan \
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
# 2. Deps Stage: Source dependencies (rosdep)
# ==============================================================================
FROM base AS deps

# Install Build Tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    libboost-all-dev \
    ccache \
    && rm -rf /var/lib/apt/lists/*

# Install source dependencies using rosdep
COPY ./main_ws/src ./main_ws/src
RUN rosdep update && \
    rosdep install --from-paths main_ws/src --ignore-src -y -r || true && \
    rm -rf main_ws/src

# ==============================================================================
# 3. Dev Stage: Specialized for coding (Inherits Deps, No local build)
# ==============================================================================
FROM deps AS dev

# Extra dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    fish neovim nano less curl gh evtest black ros-${ROS_DISTRO}-plotjuggler-ros just \
    && rm -rf /var/lib/apt/lists/*

# Use ccache
RUN ln -sf /usr/bin/ccache /usr/local/bin/gcc && \
    ln -sf /usr/bin/ccache /usr/local/bin/g++

# Copy whole repo (though usually shadowed by volume)
COPY . .

# Setup dev bashrc
RUN printf "%s\n" \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "if [ -f /root/lazytatzv_ws/main_ws/install/setup.bash ]; then source /root/lazytatzv_ws/main_ws/install/setup.bash; fi" \
    "export PATH=/usr/lib/ccache:\$PATH" \
    >> /root/.bashrc

ENTRYPOINT ["./scripts/entrypoint.sh"]
CMD ["bash"]

# ==============================================================================
# 4. Builder Stage: Production-only build
# ==============================================================================
FROM deps AS builder

COPY ./main_ws ./main_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd main_ws && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ==============================================================================
# 5. Prod Stage: Final lightweight image
# ==============================================================================
FROM base AS prod

COPY --from=builder /root/lazytatzv_ws/main_ws/install ./main_ws/install
COPY ./scripts ./scripts

# Setup prod bashrc
RUN printf "%s\n" \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "source /root/lazytatzv_ws/main_ws/install/setup.bash" \
    >> /root/.bashrc

ENTRYPOINT ["./scripts/entrypoint.sh"]
CMD ["ros2", "launch", "robot_bringup", "robot_bringup.launch.py"]
