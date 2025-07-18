# Description: ROS2 Humble with Pi Camera Support using proper libcamera integration
# Fixed version addressing package availability issues
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
    # Additional DRM/KMS dependencies (available packages only)
    libdrm2 \
    libdrm-common \
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
    # Available libcamera packages
    libcamera-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 2. Add Raspberry Pi OS repository for libcamera packages ---
# This step adds the official Raspberry Pi repository to get libcamera packages
RUN echo "deb http://archive.raspberrypi.org/debian/ bullseye main" > /etc/apt/sources.list.d/raspi.list && \
    curl -fsSL https://archive.raspberrypi.org/debian/raspberrypi.gpg.key | apt-key add - && \
    apt-get update || true

# Try to install Raspberry Pi specific packages (may not work in all architectures)
RUN apt-get install -y --no-install-recommends \
    libcamera-apps || echo "libcamera-apps not available, continuing without it" && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 3. Install colcon-meson (with compatible meson version) ---
RUN pip3 install --no-cache-dir \
    "meson>=0.56,<1.0" \
    "colcon-meson>=0.2.0" \
    jinja2 \
    ply

# --- 4. Build libcamera from source (since system packages aren't available) ---
RUN mkdir -p /tmp/libcamera_build && \
    cd /tmp/libcamera_build && \
    git clone https://git.libcamera.org/libcamera/libcamera.git && \
    cd libcamera && \
    meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=disabled -Dqt=disabled -Dcam=disabled -Dlc-compliance=disabled -Dtest=false -Ddocumentation=disabled && \
    ninja -C build && \
    ninja -C build install && \
    ldconfig && \
    cd / && rm -rf /tmp/libcamera_build

# --- 5. Install Python libcamera bindings ---
RUN pip3 install --no-cache-dir \
    "pybind11[global]" \
    && mkdir -p /tmp/pycamera_build && \
    cd /tmp/pycamera_build && \
    git clone https://github.com/raspberrypi/picamera2.git && \
    cd picamera2 && \
    pip3 install --no-cache-dir -e . && \
    cd / && rm -rf /tmp/pycamera_build

# --- 6. Create workspace and setup camera_ros ---
RUN mkdir -p /camera_ws/src
WORKDIR /camera_ws

# Clone camera_ros
RUN cd src && \
    git clone https://github.com/christianrauch/camera_ros.git -b main

# --- 7. Initialize rosdep ---
RUN rosdep init || true && rosdep update

# --- 8. Install ROS Dependencies ---
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y \
    --rosdistro humble \
    --skip-keys="libcamera clang-format" && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 9. Install additional ROS packages ---
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

# --- 10. Build camera_ros ---
RUN . /opt/ros/humble/setup.bash && \
    colcon build \
        --packages-up-to camera_ros \
        --event-handlers=console_direct+ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --parallel-workers 1

# --- 11. Install additional Python packages ---
RUN python3 -m pip install --no-cache-dir \
    "numpy~=1.23.5" \
    "opencv-python==4.8.0.76" \
    "ultralytics" \
    "torch~=2.0" \
    "torchvision~=0.15" \
    "pyyaml"

# --- 12. Setup environment ---
RUN echo "source /camera_ws/install/setup.bash" >> /root/.bashrc

# --- 13. Create camera info directory ---
RUN mkdir -p /root/.ros/camera_info

# --- 14. Set environment variables for headless operation ---
ENV DISPLAY=""
ENV QT_QPA_PLATFORM=offscreen
ENV PICAMERA2_HEADLESS=1

# Set working directory
WORKDIR /home

# Default command
CMD ["/bin/bash"]