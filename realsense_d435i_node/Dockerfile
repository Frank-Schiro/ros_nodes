FROM nvidia/cuda:12.3.1-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# =============== #
# D435i
# =============== #

# Install basic dependencies
RUN apt-get update && \
    apt-get install -y \
    software-properties-common \
    wget \
    curl \
    lsb-release \
    gpg \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Step 1: Add the key first
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# Step 2: Then configure the repository
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo jammy main" | \
    tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && \
    apt-get install -y librealsense2-utils librealsense2



# Step 3 - Install developer and debug packages
RUN apt-get install -y \
    librealsense2-dev \
    librealsense2-dbg \
    && rm -rf /var/lib/apt/lists/*

# =============== #
# ROS
# =============== #

RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*


# Add ROS2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
RUN apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y ros-humble-diagnostic-updater


# Install RealSense ROS2 wrapper directly from apt repositories
RUN apt-get update && \
    apt-get install -y \
    ros-humble-realsense2-* \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set up display
ENV DISPLAY=:0

RUN mkdir /workspace


# Launches realsense-viewer to test camera
# You may have to start/stop the toggle switches a few times for it to start working
# CMD ["realsense-viewer"]

# Default command - launches RealSense camera node
# CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true"]

# Launch full realsense suite
CMD ["bash", "-c", "\
    source /opt/ros/humble/setup.bash && \
    ros2 launch realsense2_camera rs_launch.py \
    pointcloud.enable:=true \
    align_depth.enable:=true \
    enable_sync:=true \
    enable_color:=true \
    enable_depth:=true \
    # disabled to minimize resource consumption
    # enable_infra1:=true \
    # enable_infra2:=true \
    enable_gyro:=true \
    enable_accel:=true \
    enable_rgbd:=true \
    unite_imu_method:=1 \
    spatial_filter.enable:=true \
    temporal_filter.enable:=true \
    hole_filling_filter.enable:=true \
    depth_module.depth_profile:=848x480x30 \
    rgb_camera.color_profile:=848x480x30 \
    "]