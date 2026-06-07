// Copyright 2026 Tatsukiyano
# Flexible ROS 2 Distro selection
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

# Use bash
SHELL ["/bin/bash", "-c"]

# --- 1. Infrastructure & Mirror Optimization ---
# Fix for Ubuntu 24.04+ (Noble) which uses DEB822 format (.sources files)
# Also adding retry logic for apt-get update to handle flaky networks
RUN (if [ -f /etc/apt/sources.list.d/ubuntu.sources ]; then \
        sed -i 's@http://archive.ubuntu.com@http://jp.archive.ubuntu.com@g' /etc/apt/sources.list.d/ubuntu.sources; \
        sed -i 's@http://security.ubuntu.com@http://jp.archive.ubuntu.com@g' /etc/apt/sources.list.d/ubuntu.sources; \
    else \
        sed -i 's@http://archive.ubuntu.com@http://jp.archive.ubuntu.com@g' /etc/apt/sources.list; \
        sed -i 's@http://security.ubuntu.com@http://jp.archive.ubuntu.com@g' /etc/apt/sources.list; \
    fi) && \
    for i in {1..5}; do apt-get update && break || sleep 5; done && \
    apt-get install -y --no-install-recommends \
    build-essential curl git python3-colcon-common-extensions \
    python3-pip python3-rosdep python3-vcstool \
    evtest libboost-all-dev ccache \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-ament-uncrustify \
    python3-black \
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
    rm -rf /tmp/src

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
