# Description: ROS2 Humble on Debian Bookworm for Raspberry Pi 5
# This version uses a Debian base image to match the host OS, ensuring hardware compatibility.
FROM debian:bookworm-slim

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8

# --- 1. Install Prerequisites and Add ROS2 Repository ---
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# --- 2. Install ROS2, Camera Libraries, and Python Tools ---
# This is much simpler as we can get everything from apt
RUN apt-get update && apt-get install -y \
    # ROS2 Humble
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-image-view \
    ros-humble-rqt-image-view \
    # Raspberry Pi Camera Stack (natively)
    libcamera-apps \
    python3-libcamera \
    python3-picamera2 \
    # Python and Other Tools
    python3-pip \
    python3-opencv \
    i2c-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 3. Install Additional Python Dependencies ---
RUN python3 -m pip install --no-cache-dir \
    ultralytics \
    torch \
    torchvision

# --- 4. Setup Workspace and Test Script ---
WORKDIR /ros2_ws
RUN mkdir -p src
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
COPY test_camera_stack.sh /test_camera_stack.sh
RUN chmod +x /test_camera_stack.sh

# --- 5. Set Default Directory and Command ---
WORKDIR /ros2_ws
CMD ["/bin/bash"]