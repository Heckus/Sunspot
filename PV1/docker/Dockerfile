# Description: ROS2 Humble with CORRECT Pi 5 Camera Support.
# This version follows the official camera_ros instructions to build both
# libcamera and camera_ros from source within the same ROS workspace.
FROM ros:humble-ros-base

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- 1. Install ALL System-Level Dependencies ---
# This includes build tools and libraries needed for BOTH libcamera and ROS.
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    ninja-build \
    pkg-config \
    # libcamera dependencies (from their official documentation)
    libboost-dev \
    libgnutls28-dev \
    libtiff5-dev \
    libevent-dev \
    # ROS tools
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 2. Install Python Build Tools (including colcon-meson) ---
# As per the instructions, colcon-meson is needed to build libcamera in a workspace.
RUN pip3 install --upgrade meson jinja2 ply colcon-meson

# --- 3. Create ROS Workspace and Clone ALL Sources ---
# We clone both libcamera and the recommended camera_ros fork into the workspace.
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
RUN git clone https://github.com/raspberrypi/libcamera.git -b v0.2.0 src/libcamera && \
    git clone https://github.com/christianrauch/camera_ros.git  src/camera_ros

# --- 4. Install All ROS Dependencies for the Workspace ---
# rosdep will read the package.xml for both packages and install their dependencies.
# The '--skip-keys=libcamera' flag is crucial, as it tells rosdep "don't install the old
# libcamera from apt, I have the source code for it right here".
RUN rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys=libcamera

# --- 5. Build the Entire ROS Workspace ---
# colcon will use colcon-meson to build libcamera, then build camera_ros against it.
RUN . /opt/ros/humble/setup.bash && colcon build --symlink-install

# --- 6. Install Final Runtime Packages ---
# These are packages needed to run the rest of your application.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-cv-bridge \
    ros-humble-rviz2 \
    ros-humble-vision-msgs \
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

# --- 7. Final Setup ---
# Source the new workspace we built inside the image.
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
WORKDIR /home
CMD ["/bin/bash"]