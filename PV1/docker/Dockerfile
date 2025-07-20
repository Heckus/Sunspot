# Description: Clean ROS 2 Humble environment with camera_ros and CycloneDDS.
# Base image provides ROS and basic tools.
FROM ros:humble-ros-base

# Set environment for non-interactive installs and shell
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- 1. Install Core System & ROS 2 Dependencies ---
# Install everything in a single RUN command to reduce image layers.
# This includes Python tools, ROS build tools, and your specific ROS packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Python & Build essentials
    python3-pip \
    python3-colcon-common-extensions \
    git \
    # Your required ROS packages
    # camera_ros will automatically pull in libcamera and cv_bridge as dependencies.
    ros-humble-camera-ros \
    ros-humble-rmw-cyclonedds-cpp \
    # Common tools for computer vision and visualization
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-image-proc \
    ros-humble-rviz2 \
    # Optional but useful tools
    i2c-tools \
    v4l-utils \
    # Clean up apt cache to keep image small
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 2. Install Python Packages ---
# Install Python packages required for your application nodes.
# Note: We do not install opencv-python from pip to avoid conflicts with the
# system version (ros-humble-vision-opencv) that ROS depends on.
RUN python3 -m pip install --no-cache-dir \
    "numpy" \
    "ultralytics" \
    "torch" \
    "torchvision" \
    "pyyaml" \
    "picamera2"

# --- 3. Setup Environment ---
# Source the main ROS 2 setup file in the .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# --- 4. Create camera info directory ---
RUN mkdir -p /root/.ros/camera_info

# Set a working directory for your projects
WORKDIR /root/ros2_ws

# Default command to start an interactive bash shell
CMD ["/bin/bash"]