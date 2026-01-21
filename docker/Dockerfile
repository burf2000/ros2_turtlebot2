# TurtleBot 2 ROS2 Humble on Jetson Nano
# Base image from dusty-nv/jetson-containers

ARG BASE_IMAGE=dustynv/ros:humble-ros-base-l4t-r32.7.1
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    libudev-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libboost-all-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages available via apt
RUN apt-get update && apt-get install -y \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-diagnostic-updater \
    || true \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
ENV ROS_WS=/root/turtlebot2_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Clone Kobuki packages
RUN cd ${ROS_WS}/src && \
    git clone https://github.com/kobuki-base/kobuki_core.git && \
    git clone https://github.com/kobuki-base/kobuki_ros.git && \
    git clone https://github.com/kobuki-base/velocity_smoother.git && \
    git clone https://github.com/kobuki-base/cmd_vel_mux.git

# Clone ECL dependencies
RUN cd ${ROS_WS}/src && \
    git clone https://github.com/stonier/ecl_tools.git && \
    git clone https://github.com/stonier/ecl_lite.git && \
    git clone https://github.com/stonier/ecl_core.git

# Clone Sophus (geometric library needed by kobuki)
RUN cd ${ROS_WS}/src && \
    git clone https://github.com/strasdat/Sophus.git

# Clone OrbbecSDK for Astra camera
RUN cd ${ROS_WS}/src && \
    git clone https://github.com/orbbec/OrbbecSDK_ROS2.git && \
    cd OrbbecSDK_ROS2 && \
    git checkout main

# Install rosdep dependencies
RUN apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep update --rosdistro=humble || true && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    rm -rf /var/lib/apt/lists/*

# Build workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip-build-finished

# Copy local packages (will be mounted, but copy for standalone use)
COPY src/turtlebot2_description ${ROS_WS}/src/turtlebot2_description
COPY src/turtlebot2_bringup ${ROS_WS}/src/turtlebot2_bringup

# Build local packages
RUN source /opt/ros/humble/setup.bash && \
    source ${ROS_WS}/install/setup.bash && \
    colcon build --symlink-install --packages-select turtlebot2_description turtlebot2_bringup

# Setup entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
