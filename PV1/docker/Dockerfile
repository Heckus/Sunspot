# Description: ROS2 Humble with CORRECT Pi 5 Camera Support using libcamera and picamera2
# This version builds both libcamera AND camera_ros from source to guarantee correct linking.
FROM ros:humble-ros-base

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash for shell commands
SHELL ["/bin/bash", "-c"]

# --- 1. Install Core Build-Time Dependencies (for libcamera AND ROS) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    ninja-build \
    pkg-config \
    openssh-client \
    # libcamera & ROS dependencies
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
    qtbase5-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    # Python and ROS tools
    python3-pip \
    python3-dev \
    python3-rosdep \
    i2c-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

#--- Add known git hosts to avoid manual confirmation ---
RUN mkdir -p -m 0700 ~/.ssh && \
    ssh-keyscan github.com >> ~/.ssh/known_hosts && \
    ssh-keyscan gitlab.com >> ~/.ssh/known_hosts && \
    ssh-keyscan bitbucket.org >> ~/.ssh/known_hosts

#--- 2. Install Python build tools via Pip ---
RUN pip3 install --upgrade meson jinja2 ply

# --- 3. Build and Install libcamera From Source ---
WORKDIR /usr/src
RUN git clone https://github.com/raspberrypi/libcamera.git && \
    cd libcamera && \
    meson build -Dpipelines=rpi/pisp -Dtest=false -Dv4l2=true && \
    ninja -C build install

# --- 4. Build and Install libcamera-apps From Source ---
WORKDIR /usr/src
RUN git clone https://github.com/raspberrypi/libcamera-apps.git && \
    cd libcamera-apps && \
    meson build && \
    ninja -C build install

# --- 5. CRITICAL FIX: Update Library Links ---
# This forces the system to recognize the new libcamera you just built
RUN ldconfig

# --- 6. Create ROS Workspace and Add camera_ros ---
# This step creates a workspace and clones the correct camera_ros fork into it.
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
RUN git clone https://github.com/christianrauch/camera_ros.git src/camera_ros

# --- 7. Install ROS Dependencies for a Source Build ---
# rosdep installs all libraries needed to COMPILE the packages in your workspace.
RUN rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# --- 8. Build the ROS Workspace (Compiles camera_ros) ---
# This is the key: it compiles camera_ros from source, linking it against the new libcamera.
RUN . /opt/ros/humble/setup.bash && colcon build --symlink-install

# --- 9. Install Remaining ROS and Python Packages ---
# Install other pre-built ROS nodes and the final Python packages.
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
    python3-colcon-common-extensions \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    "numpy~=1.23.5" \
    "opencv-python==4.8.0.76" \
    "ultralytics" \
    "torch~=2.0" \
    "torchvision~=0.15" \
    "pyyaml" \
    "picamera2"

# --- 10. Final Setup ---
# Source the new workspace we built inside the image.
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
WORKDIR /home
CMD ["/bin/bash"]