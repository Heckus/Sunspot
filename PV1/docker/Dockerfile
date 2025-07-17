# Description: ROS2 Humble with Pi Camera Support using proper libcamera integration
# Uses colcon workspace approach with Raspberry Pi libcamera fork
FROM ros:humble-ros-base

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- 1. Install System Dependencies ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    ninja-build \
    pkg-config \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    # libcamera dependencies (as per libcamera.org/getting-started.html)
    libboost-dev \
    libgnutls28-dev \
    libtiff5-dev \
    libevent-dev \
    libgtest-dev \
    libyaml-dev \
    python3-yaml \
    python3-ply \
    python3-jinja2 \
    libssl-dev \
    libudev-dev \
    libdrm-dev \
    libjpeg-dev \
    libdw-dev \
    libunwind-dev \
    # ROS tools & dependencies
    curl \
    python3-rosdep \
    python3-colcon-common-extensions \
    # OpenCV and image processing
    libopencv-dev \
    python3-opencv \
    # Additional tools
    clang-format \
    python3-flake8 \
    python3-mypy \
    # I2C tools
    i2c-tools \
    libi2c-dev \
    # Video4Linux utilities for debugging
    v4l-utils \
    # Add dependency for python-prctl (needed by picamera2)
    libcap-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 2. Install colcon-meson (with compatible meson version) ---
RUN pip3 install --no-cache-dir \
    "meson>=0.56,<1.0" \
    "colcon-meson>=0.2.0" \
    jinja2 \
    ply

# --- 3. Create workspace and setup libcamera + camera_ros ---
RUN mkdir -p /camera_ws/src
WORKDIR /camera_ws

# Clone Raspberry Pi libcamera fork (essential for newer camera modules)
RUN cd src && \
    git clone https://github.com/raspberrypi/libcamera.git

# Clone camera_ros
RUN cd src && \
    git clone https://github.com/christianrauch/camera_ros.git -b main

# --- 4. Initialize rosdep ---
RUN rosdep init || true && rosdep update

# --- 5. Install ROS Dependencies (skip libcamera to avoid conflicts) ---
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y \
    --rosdistro humble \
    --skip-keys="libcamera clang-format" && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 6. Install additional ROS packages ---
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
    ros-humble-builtin-interfaces \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rviz2 \
    ros-humble-image-proc \
    ros-humble-vision-opencv \
    ros-humble-image-view \
    ros-humble-rqt-image-view \
    ros-humble-vision-msgs \
    ros-humble-camera-calibration \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 7. Build libcamera first, then camera_ros ---
# Note: Using parallel-workers 1 and disabling python bindings for reliability in low-memory environments.
# If you have allocated more memory to Docker (e.g., >8GB), you can try increasing workers and removing -Dpython=disabled.
RUN . /opt/ros/humble/setup.bash && \
    # First, build and install only libcamera
    colcon build \
        --packages-select libcamera \
        --event-handlers=console_direct+ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -Dpython=disabled \
        --parallel-workers 1 && \
    \
    # Now, source the new install space and build the rest
    . /camera_ws/install/setup.bash && \
    colcon build \
        --packages-up-to camera_ros \
        --event-handlers=console_direct+ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --parallel-workers 1

# --- 8. Install Python packages ---
RUN python3 -m pip install --no-cache-dir \
    "numpy~=1.23.5" \
    "opencv-python==4.8.0.76" \
    "ultralytics" \
    "torch~=2.0" \
    "torchvision~=0.15" \
    "pyyaml" \
    "picamera2"

# --- 9. Setup environment ---
RUN echo "source /camera_ws/install/setup.bash" >> /root/.bashrc

# --- 10. Create camera info directory ---
RUN mkdir -p /root/.ros/camera_info

# Set working directory
WORKDIR /home

# Default command
CMD ["/bin/bash"]