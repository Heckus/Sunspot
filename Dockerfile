# Description: ROS2 Humble with Pi 5 Camera Support using libcamera and picamera2
# This version uses a COPY instruction for scripts to avoid syntax/line-ending errors.
FROM ros:humble-ros-base

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash for shell commands
SHELL ["/bin/bash", "-c"]

# Install essential build tools, Python, and modern camera libraries
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-opencv \
    libcamera-apps \
    i2c-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install core ROS2 vision and tool packages
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

# Install Python dependencies using pip
RUN python3 -m pip install --no-cache-dir \
    numpy \
    opencv-python \
    ultralytics \
    torch \
    torchvision \
    pyyaml \
    picamera2

# Create ROS2 workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Set up ROS2 environment in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Copy the validation script into the container and make it executable
COPY test_camera_stack.sh /test_camera_stack.sh
RUN chmod +x /test_camera_stack.sh

# Default working directory
WORKDIR /ros2_ws

# Default command to start an interactive session
CMD ["/bin/bash"]