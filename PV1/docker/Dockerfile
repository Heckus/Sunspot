# Description: Corrected ROS2 Humble with Pi Camera Support using libcamera
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- 1. Install System Dependencies ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential cmake git ninja-build pkg-config python3-pip \
    python3-setuptools python3-wheel \
    # libcamera dependencies
    libboost-dev libgnutls28-dev libtiff5-dev libevent-dev libyaml-dev \
    python3-yaml python3-ply python3-jinja2 libssl-dev libudev-dev \
    libdrm-dev libjpeg-dev \
    # ROS tools & image processing
    curl python3-rosdep python3-colcon-common-extensions libopencv-dev \
    python3-opencv v4l-utils i2c-tools libi2c-dev \
    # Dependency for python-prctl (needed by picamera2)
    libcap-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 2. Install colcon-meson ---
# Required for building some ROS2 packages, install before building libcamera
RUN pip3 install --no-cache-dir "meson>=0.56,<1.0" "colcon-meson>=0.2.0"

# --- 3. Build libcamera from source ---
RUN mkdir -p /tmp/libcamera_build && \
    cd /tmp/libcamera_build && \
    git clone https://git.libcamera.org/libcamera/libcamera.git && \
    cd libcamera && \
    meson setup build --buildtype=release -Dpipelines=rpi/vc4 -Dv4l2=true && \
    ninja -C build && \
    ninja -C build install && \
    ldconfig && \
    cd / && rm -rf /tmp/libcamera_build

# --- 4. Install picamera2 and its Python bindings for libcamera (CORRECTED) ---
RUN pip3 install --no-cache-dir "pybind11[global]" && \
    mkdir -p /tmp/picamera2_build && \
    cd /tmp/picamera2_build && \
    git clone https://github.com/raspberrypi/picamera2.git && \
    cd picamera2 && \
    # Use a standard install, not an editable (-e) one
    pip3 install --no-cache-dir . && \
    cd / && rm -rf /tmp/picamera2_build

# --- 5. Create and setup ROS2 workspace ---
RUN mkdir -p /camera_ws/src
WORKDIR /camera_ws
RUN cd src && git clone https://github.com/christianrauch/camera_ros.git -b main

# --- 6. Install ROS Dependencies ---
RUN rosdep init || true && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
    --rosdistro humble --skip-keys="libcamera"
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
# --- 7. Build camera_ros ---
RUN . /opt/ros/humble/setup.bash && \
    colcon build \
        --packages-up-to camera_ros \
        --event-handlers=console_direct+ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --parallel-workers 1

# --- 8. Install additional Python packages ---
RUN python3 -m pip install --no-cache-dir \
    "numpy~=1.23.5" \
    "ultralytics" \
    "torch~=2.0" \
    "torchvision~=0.15"

# --- 9. Setup environment ---
RUN echo "source /camera_ws/install/setup.bash" >> /root/.bashrc
RUN mkdir -p /root/.ros/camera_info

# Environment variables for headless operation
ENV QT_QPA_PLATFORM=offscreen
ENV PICAMERA2_HEADLESS=1

WORKDIR /home
CMD ["/bin/bash"]