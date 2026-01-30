# TurtleBot 2 ROS2 Humble on Jetson Nano
# Base image from dusty-nv/jetson-containers

ARG BASE_IMAGE=dustynv/ros:humble-ros-base-l4t-r32.7.1
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Fix expired ROS2 GPG key
RUN rm -f /etc/apt/trusted.gpg.d/ros* || true && \
    apt-get update || true && \
    apt-get install -y curl ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list && \
    rm -rf /var/lib/apt/lists/*

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
    libfmt-dev \
    libopenni2-dev \
    && rm -rf /var/lib/apt/lists/*

# Install xacro via pip (needed for URDF processing)
RUN pip3 install xacro

# Install Sophus system-wide (required by ecl_linear_algebra)
RUN cd /tmp && \
    git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    git checkout 1.22.10 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/Sophus

# Create workspace
ENV ROS_WS=/root/turtlebot2_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Clone Kobuki packages
RUN cd ${ROS_WS}/src && \
    git clone https://github.com/kobuki-base/kobuki_core.git && \
    git clone https://github.com/kobuki-base/kobuki_ros.git && \
    git clone https://github.com/kobuki-base/kobuki_ros_interfaces.git && \
    git clone https://github.com/kobuki-base/velocity_smoother.git && \
    git clone https://github.com/kobuki-base/cmd_vel_mux.git

# Clone ECL dependencies
RUN cd ${ROS_WS}/src && \
    git clone https://github.com/stonier/ecl_tools.git && \
    git clone https://github.com/stonier/ecl_lite.git && \
    git clone https://github.com/stonier/ecl_core.git

# Clone openni2_camera for ASUS Xtion Pro (iron branch supports Humble)
RUN cd ${ROS_WS}/src && \
    git clone -b iron https://github.com/ros-drivers/openni2_camera.git

# Clone depthimage_to_laserscan (virtual laser scan from depth camera)
RUN cd ${ROS_WS}/src && \
    git clone -b ros2 https://github.com/ros-perception/depthimage_to_laserscan.git

# Clone angles package (required by kobuki_node)
RUN cd ${ROS_WS}/src && \
    git clone -b humble-devel https://github.com/ros/angles.git

# Clone diagnostics packages (required by kobuki_node)
RUN cd ${ROS_WS}/src && \
    git clone -b ros2-humble https://github.com/ros/diagnostics.git

# Clone xacro and joint_state_publisher for robot description
RUN cd ${ROS_WS}/src && \
    git clone -b ros2 https://github.com/ros/xacro.git && \
    git clone -b ros2 https://github.com/ros/joint_state_publisher.git

# Install rosdep dependencies
RUN apt-get update && \
    source /opt/ros/humble/install/setup.bash && \
    rosdep update --rosdistro=humble || true && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    rm -rf /var/lib/apt/lists/*

# Build workspace (multiple passes to handle dependency order)
# Note: -Wno-error flags needed for older GCC 7 on Jetson Nano (Ubuntu 18.04)
# -Wno-error=pragmas: kobuki_core uses pragmas not recognized by GCC 7
RUN source /opt/ros/humble/install/setup.bash && \
    export CXXFLAGS="-Wno-error -Wno-error=pragmas" && \
    for i in 1 2 3 4 5; do \
        echo "=== Build pass $i ===" && \
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wno-error -Wno-error=pragmas" --continue-on-error && \
        source install/setup.bash 2>/dev/null || true; \
    done && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wno-error -Wno-error=pragmas" --continue-on-error || true

# Copy local packages (will be mounted, but copy for standalone use)
COPY src/turtlebot2_description ${ROS_WS}/src/turtlebot2_description
COPY src/turtlebot2_bringup ${ROS_WS}/src/turtlebot2_bringup

# Build local packages
RUN source /opt/ros/humble/install/setup.bash && \
    source ${ROS_WS}/install/setup.bash && \
    colcon build --symlink-install --packages-select turtlebot2_description turtlebot2_bringup || true

# Setup entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
