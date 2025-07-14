# Description: ROS2 Humble on Debian Bookworm for Raspberry Pi 5
# This version uses a Debian base image and adds all necessary build dependencies.
FROM debian:bookworm-slim

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8

# --- 1. Install ALL Build-Time & System Dependencies ---
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    git \
    ninja-build \
    pkg-config \
    # Dependencies for libcamera & libcamera-apps
    libboost-dev \
    libboost-program-options-dev \
    libgnutls28-dev \
    libtiff5-dev \
    libevent-dev \
    libexif-dev \
    libavcodec-dev \
    libavdevice-dev \
    libpng-dev \
    libdrm-dev \
    libcap-dev \
    # Python & Other Tools
    curl \
    gnupg \
    lsb-release \
    python3-pip \
    python3-dev \
    python3-opencv \
    i2c-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 2. Install Python build tools via Pip ---
# Added PyYAML for libcamera build process
RUN pip3 install --upgrade --break-system-packages meson jinja2 ply pyyaml

# --- 3. Add the ROS2 APT Repository ---
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# --- 4. Build and Install libcamera From Source ---
WORKDIR /usr/src
RUN git clone https://github.com/raspberrypi/libcamera.git && \
    cd libcamera && \
    meson build -Dpipelines=rpi/pisp -Dtest=false -Dv4l2=true && \
    ninja -C build install

# --- 5. Build and Install libcamera-apps From Source ---
WORKDIR /usr/src
RUN git clone https://github.com/raspberrypi/libcamera-apps.git && \
    cd libcamera-apps && \
    meson build && \
    ninja -C build install

# --- 6. Update Library Links & Cleanup ---
RUN ldconfig && \
    rm -rf /usr/src/libcamera /usr/src/libcamera-apps

# --- 7. Install ROS2 Packages & Final Python Runtime ---
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-image-view \
    ros-humble-rqt-image-view \
    && apt-get clean && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir --break-system-packages \
       picamera2 \
       ultralytics \
       torch \
       torchvision

# --- 8. Setup Workspace and Test Script ---
WORKDIR /ros2_ws
RUN mkdir -p src
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
COPY test_camera_stack.sh /test_camera_stack.sh
RUN chmod +x /test_camera_stack.sh

# --- 9. Set Default Directory and Command ---
WORKDIR /ros2_ws
CMD ["/bin/bash"]