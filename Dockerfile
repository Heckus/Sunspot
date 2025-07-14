# Description: ROS2 Humble with Pi 5 Camera Support using libcamera and picamera2
# This version builds libcamera and libcamera-apps from source to ensure compatibility.
FROM ros:humble-ros-base

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash for shell commands
SHELL ["/bin/bash", "-c"]

# --- 1. Install All Build-Time and Run-Time Dependencies ---
RUN apt-get update && apt-get install -y \
    # Build tools for libcamera and others
    build-essential \
    cmake \
    git \
    meson \
    ninja-build \
    pkg-config \
    # libcamera dependencies
    libboost-dev \
    libgnutls28-dev \
    libtiff5-dev \
    libevent-dev \
    # Python tools
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-opencv \
    # Other tools
    i2c-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 2. Build and Install libcamera From Source ---
WORKDIR /usr/src
RUN git clone https://github.com/raspberrypi/libcamera.git && \
    cd libcamera && \
    meson build -Dpipelines=raspberrypi -Dtest=false -Dv4l2=true && \
    ninja -C build install

# --- 3. Build and Install libcamera-apps From Source ---
# Note: This provides the 'libcamera-hello' and other tools.
WORKDIR /usr/src
RUN git clone https://github.com/raspberrypi/libcamera-apps.git && \
    cd libcamera-apps && \
    meson build && \
    ninja -C build install

# --- 4. Update Library Links and Clean Up Build Artifacts ---
RUN ldconfig && \
    rm -rf /usr/src/libcamera /usr/src/libcamera-apps

# --- 5. Install ROS2 Packages ---
RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-image-proc \
    ros-humble-vision-opencv \
    ros-humble-image-view \
    ros-humble-rqt-image-view \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    python3-colcon-common-extensions \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 6. Install Python Dependencies ---
RUN python3 -m pip install --no-cache-dir \
    numpy \
    opencv-python \
    ultralytics \
    torch \
    torchvision \
    pyyaml \
    picamera2

# --- 7. Setup Workspace and Test Script ---
WORKDIR /ros2_ws
RUN mkdir -p src
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
COPY test_camera_stack.sh /test_camera_stack.sh
RUN chmod +x /test_camera_stack.sh

# --- 8. Set Default Directory and Command ---
WORKDIR /ros2_ws
CMD ["/bin/bash"]