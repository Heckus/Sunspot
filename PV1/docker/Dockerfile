# Description: ROS2 Humble with CORRECT Pi 5 Camera Support.
# This version follows the official camera_ros instructions to build both
# libcamera and camera_ros from source within the same ROS workspace.
FROM ros:humble-ros-base

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- 1. Install ALL System-Level Dependencies ---
# This includes build tools and libraries needed for BOTH libcamera and ROS.
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    ninja-build \
    pkg-config \
    # libcamera dependencies (from their official documentation)
    libboost-dev \
    libgnutls28-dev \
    libtiff5-dev \
    libevent-dev \
    libgtest-dev \
    libyaml-dev \
    python3-yaml \
    python3-ply \
    python3-jinja2 \
    # Additional libcamera dependencies
    libssl-dev \
    libudev-dev \
    # ROS tools & other dependencies
    curl \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-colcon-meson \
    # OpenCV and image processing dependencies
    libopencv-dev \
    python3-opencv \
    # Additional ROS packages that might be missing
    clang-format \
    python3-flake8 \
    python3-mypy \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 2. Install Python Build Tools ---
RUN pip3 install --upgrade \
    meson \
    jinja2 \
    ply \
    colcon-meson \
    setuptools \
    wheel

# --- 3. Create ROS Workspace and Clone ALL Sources ---
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Clone libcamera (Pi specific version)
RUN git clone https://github.com/raspberrypi/libcamera.git -b v0.2.0 src/libcamera

# Clone camera_ros
RUN git clone https://github.com/christianrauch/camera_ros.git -b main src/camera_ros

# --- 4. Initialize rosdep and update ---
RUN rosdep init || true && rosdep update

# --- 5. Install Available ROS Dependencies ---
# Install what's available, skip problematic packages
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys="libcamera clang-format ros-humble-ament-cmake-clang-format ros-humble-ament-cmake-mypy ros-humble-ament-cmake-pyflakes" || true

# --- 6. Install missing ROS packages manually ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-camera-info-manager \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-image-transport \
    ros-humble-ament-cmake \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 7. Build the Entire ROS Workspace ---
# colcon will use colcon-meson to build libcamera, then build camera_ros against it.
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)

# --- 8. Install Final Runtime Packages ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-camera-calibration \
    ros-humble-image-view \
    ros-humble-rqt-image-view \
    # I2C tools
    i2c-tools \
    libi2c-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 9. Install Python packages ---
RUN python3 -m pip install --no-cache-dir \
    "numpy~=1.23.5" \
    "opencv-python==4.8.0.76" \
    "ultralytics" \
    "torch~=2.0" \
    "torchvision~=0.15" \
    "pyyaml" \
    "picamera2"

# --- 10. Final Setup ---
# Source the new workspace we built inside the image.
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /home

# Default command
CMD ["/bin/bash"]