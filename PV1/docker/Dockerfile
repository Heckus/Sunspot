# Description: ROS2 Humble with CORRECT Pi 5 Camera Support.
# This version follows the official camera_ros instructions to build both
# libcamera and camera_ros from source within the same ROS workspace.
FROM ros:humble-ros-base

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- 1. Install ALL System-Level Dependencies ---
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
    # ROS tools & other dependencies
    curl \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 2. Add Official ROS2 Repository (CRITICAL FIX) ---
# This tells apt where to find all the 'ros-humble-*' packages.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# --- 3. Install Python Build Tools (including colcon-meson) ---
RUN pip3 install --upgrade meson jinja2 ply colcon-meson

# --- 4. Create ROS Workspace and Clone ALL Sources ---
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
RUN git clone https://github.com/raspberrypi/libcamera.git -b v0.2.0 src/libcamera && \
    git clone https://github.com/christianrauch/camera_ros.git -b main src/camera_ros

# --- 5. Install All ROS Dependencies for the Workspace ---
# Now that the ROS repo is added, rosdep will successfully find the packages.
RUN apt-get update && \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys=libcamera

# --- 6. Build the Entire ROS Workspace ---
RUN . /opt/ros/humble/setup.bash && colcon build --symlink-install

# --- 7. Install Final Runtime Packages ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-cv-bridge \
    ros-humble-rviz2 \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-image-proc \
    ros-humble-vision-opencv \
    ros-humble-image-view \
    ros-humble-rqt-image-view \
    ros-humble-sensor-msgs \
    ros-humble-vision-msgs \
    ros-humble-geometry-msgs \
    ros-humble-camera-calibration \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    "numpy~=1.23.5" \
    "opencv-python==4.8.0.76" \
    "ultralytics" \
    "torch~=2.0" \
    "torchvision~=0.15" \
    "pyyaml" \
    "picamera2"

# --- 8. Final Setup ---
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
WORKDIR /home
CMD ["/bin/bash"]