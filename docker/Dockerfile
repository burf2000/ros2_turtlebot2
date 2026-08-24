# TurtleBot 2 ROS2 Humble
# Standard Ubuntu 22.04 base (no GPU acceleration)
# Simplified build using pre-built packages where possible

ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Install system dependencies and ROS2 packages via apt (pre-built binaries)
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    libudev-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libboost-all-dev \
    # ROS2 packages (pre-built)
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-aggregator \
    ros-humble-depthimage-to-laserscan \
    ros-humble-openni2-camera \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Python deps for the Burf Platform driver (WebRTC video + websocket)
RUN pip3 install --no-cache-dir \
    "websockets>=12.0,<14.0" \
    "psutil>=5.9.0,<7.0" \
    "opencv-python-headless>=4.8.0,<5.0" \
    "aiortc>=1.6.0,<2.0" \
    "av>=10.0.0,<14.0" \
    "numpy<2.0"

# Create workspace
ENV ROS_WS=/root/turtlebot2_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Only clone Kobuki packages (not available as pre-built binaries)
# These are the only packages we need to build from source
RUN cd ${ROS_WS}/src && \
    git clone https://github.com/kobuki-base/kobuki_core.git && \
    git clone https://github.com/kobuki-base/kobuki_ros.git && \
    git clone https://github.com/kobuki-base/kobuki_ros_interfaces.git && \
    git clone https://github.com/kobuki-base/velocity_smoother.git && \
    git clone https://github.com/kobuki-base/cmd_vel_mux.git

# Clone ECL dependencies (required by Kobuki, not available pre-built)
RUN cd ${ROS_WS}/src && \
    git clone https://github.com/stonier/ecl_tools.git && \
    git clone https://github.com/stonier/ecl_lite.git && \
    git clone https://github.com/stonier/ecl_core.git

# Install rosdep dependencies for source packages
RUN apt-get update && \
    rosdep update --rosdistro=humble || true && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    rm -rf /var/lib/apt/lists/*

# Build only the Kobuki + ECL packages from source
# Much faster than building everything
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error

# Okdo LD06 LiDAR driver (ldlidar_stl_ros2) — no apt/rosdep binary, vendor from
# source (same approach as the wheelchair build). turtlebot2.launch.py launches
# package 'ldlidar_stl_ros2', exe 'ldlidar_stl_ros2_node'. NB: this SDK's baud
# param is `port_baudrate` (NOT serial_baudrate) — see bringup config/ldlidar.yaml.
# (Deliberately AFTER the big Kobuki/ECL build layer so adding it doesn't bust
# the docker cache — a full rebuild takes ~an hour on the Nano.)
RUN cd ${ROS_WS}/src && \
    git clone --depth 1 https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git && \
    source /opt/ros/humble/setup.bash && \
    source ${ROS_WS}/install/setup.bash && \
    colcon build --symlink-install --packages-select ldlidar_stl_ros2 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------------------------
# Nav2 (autonomous navigation) — BAKED IN, because the container is recreated
# from this image on every boot (scripts/start_turtlebot2.sh does
# docker stop/rm/run), so anything apt-installed into a *running* container is
# lost at the next reboot.
#
# ⚠ ros-humble-diagnostic-updater is listed BOTH above and here ON PURPOSE.
# The version pinned in the older base layer (4.0.6) ships the Python module
# and the C++ headers but NO libdiagnostic_updater.so. Nav2's binaries link
# against that .so, so with 4.0.6 every Nav2 node dies at startup with
#   error while loading shared libraries: libdiagnostic_updater.so
# lifecycle_manager then leaves the whole stack `unconfigured` and no
# /navigate_to_pose action ever appears. 4.0.7 restores the shared library.
# Do NOT drop this line thinking it is a duplicate — it is a forced upgrade.
#
# Deliberately a LATE, SEPARATE layer: the Kobuki/ECL source build above takes
# well over an hour on a Jetson Nano, and adding packages here must not bust
# its cache.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# Copy local packages (will be mounted, but copy for standalone use)
COPY src/turtlebot2_description ${ROS_WS}/src/local/turtlebot2_description
COPY src/turtlebot2_bringup ${ROS_WS}/src/local/turtlebot2_bringup
COPY src/burf_platform_driver ${ROS_WS}/src/local/burf_platform_driver

# Build local packages (incl. the Burf Platform driver)
RUN source /opt/ros/humble/setup.bash && \
    source ${ROS_WS}/install/setup.bash && \
    colcon build --symlink-install --packages-select \
        turtlebot2_description turtlebot2_bringup burf_platform_driver

# Setup entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
