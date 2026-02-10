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
    ros-humble-camera-info-manager \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

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

# Copy local packages (will be mounted, but copy for standalone use)
COPY src/turtlebot2_description ${ROS_WS}/src/local/turtlebot2_description
COPY src/turtlebot2_bringup ${ROS_WS}/src/local/turtlebot2_bringup

# Build local packages
RUN source /opt/ros/humble/setup.bash && \
    source ${ROS_WS}/install/setup.bash && \
    colcon build --symlink-install --packages-select turtlebot2_description turtlebot2_bringup

# Setup entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
