# Description: ROS2 Humble with Pi 5 Camera Support
# Uses a two-stage approach: build libcamera separately, then build camera_ros
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
    # libcamera dependencies
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
    # ROS tools & dependencies
    curl \
    python3-pip \
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
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 2. Install specific versions of Python build tools ---
RUN pip3 install --upgrade \
    "meson>=0.63.0" \
    "colcon-meson>=0.2.0" \
    jinja2 \
    ply \
    setuptools \
    wheel

# --- 3. Build libcamera separately (system-wide) ---
WORKDIR /tmp
RUN git clone --branch v0.3.0 https://github.com/raspberrypi/libcamera.git && \
    cd libcamera && \
    meson setup build --buildtype=release \
        -Dpipelines=rpi/vc4,rpi/pisp \
        -Dipas=rpi/vc4,rpi/pisp \
        -Dv4l2=enabled \
        -Dqcam=disabled \
        -Dcam=disabled \
        -Dlc-compliance=disabled \
        -Dgstreamer=disabled \
        -Dtest=false && \
    ninja -C build && \
    ninja -C build install && \
    ldconfig && \
    cd .. && rm -rf libcamera

# --- 4. Create ROS Workspace ---
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Clone camera_ros
RUN git clone https://github.com/christianrauch/camera_ros.git -b main src/camera_ros

# --- 5. Initialize rosdep ---
RUN rosdep init || true && rosdep update

# --- 6. Install ROS Dependencies ---
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys="libcamera clang-format ros-humble-ament-cmake-clang-format ros-humble-ament-cmake-mypy ros-humble-ament-cmake-pyflakes" || true

# --- 7. Install missing ROS packages manually ---
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
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 8. Set PKG_CONFIG_PATH to find libcamera ---
ENV PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:/usr/local/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH"
ENV LD_LIBRARY_PATH="/usr/local/lib:/usr/local/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

# --- 9. Build camera_ros ---
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

# --- 10. Install Additional ROS packages ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-camera-info-manager \
    ros-humble-image-proc \
    ros-humble-vision-opencv \
    ros-humble-image-view \
    ros-humble-rqt-image-view \
    ros-humble-vision-msgs \
    ros-humble-camera-calibration \
    python3-colcon-common-extensions \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 11. Install Python packages ---
RUN python3 -m pip install --no-cache-dir \
    "numpy~=1.23.5" \
    "opencv-python==4.8.0.76" \
    "ultralytics" \
    "torch~=2.0" \
    "torchvision~=0.15" \
    "pyyaml" \
    "picamera2"

# --- 12. Final Setup ---
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /home

# Default command
CMD ["/bin/bash"]