# Copyright 2026 Tatsukiyano
# Flexible ROS 2 Distro selection
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

# Use bash
SHELL ["/bin/bash", "-c"]

# --- 1. Infrastructure Optimization ---
# We use standard mirrors for stability.
# We DO NOT touch /etc/resolv.conf here to avoid "Read-only file system" errors.
RUN apt-get update --allow-releaseinfo-change && \
    apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    git \
    fish \
    less \
    nano \
    vim \
    neovim \
    gh \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    evtest \
    libboost-all-dev \
    ccache \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-ament-uncrustify \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-serial-driver \
    ros-${ROS_DISTRO}-io-context \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-robot-localization \
    black \
    && rm -rf /var/lib/apt/lists/*

# --- 2. Environment Configuration ---
ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=/root/lazytatzv_ws/main_ws
ENV CCACHE_DIR=/root/.ccache
WORKDIR /root/lazytatzv_ws

# --- 3. Dependency Layer (rosdep) ---
COPY ./main_ws/src /tmp/src
RUN apt-get update && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths /tmp/src --ignore-src -y -r && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/src || true

# --- 4. Development Tools Setup ---
RUN ln -sf /usr/bin/ccache /usr/local/bin/gcc && \
    ln -sf /usr/bin/ccache /usr/local/bin/g++ && \
    ln -sf /usr/bin/ccache /usr/local/bin/cc && \
    ln -sf /usr/bin/ccache /usr/local/bin/c++

# --- 5. Source Code Layer ---
COPY . .

# Final env setup
RUN printf "%s\n" \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "if [ -f /root/lazytatzv_ws/main_ws/install/setup.bash ]; then source /root/lazytatzv_ws/main_ws/install/setup.bash; fi" \
    "export PATH=/usr/lib/ccache:\$PATH" \
    >> /root/.bashrc

CMD ["bash"]
