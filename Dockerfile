# [cite_start]Description: ROS2 Humble with Pi 5 Camera Support using libcamera and picamera2 [cite: 1]
# This version includes graphics libraries (Qt5) to enable preview windows.
FROM ros:humble-ros-base 

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash for shell commands
SHELL ["/bin/bash", "-c"]

# --- 1. Install Core Build-Time Dependencies (for libcamera) ---
RUN apt-get update && apt-get install -y \
    # Build tools for libcamera and others
    build-essential \
    cmake \
    git \
    ninja-build \
    pkg-config \
    openssh-client \
    # libcamera & libcamera-apps dependencies
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
    # Graphics libraries for preview window
    qtbase5-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    # Python tools
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-opencv \
    # Other tools
    i2c-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- Add known git hosts to avoid manual confirmation ---
# RUN mkdir -p -m 0700 ~/.ssh && \
#     ssh-keyscan github.com >> ~/.ssh/known_hosts && \
#     ssh-keyscan gitlab.com >> ~/.ssh/known_hosts && \
#     ssh-keyscan bitbucket.org >> ~/.ssh/known_hosts
# --- 2. Install Python build tools via Pip ---
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

# --- 5. Update Library Links and Clean Up Build Artifacts ---
RUN ldconfig && \
    rm -rf /usr/src/libcamera /usr/src/libcamera-apps

# --- FIX: Re-add Ubuntu and ROS GPG Keys ---
# RUN apt-get update && \
#     apt-get install -y --no-install-recommends \
#         gnupg \
#         ubuntu-keyring \
#         curl && \
#     rm -f /etc/apt/sources.list.d/ros2.list && \
#     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# --- 6. Install ROS2 Packages ---
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
# --- 7. Install Final Build Dependency for Python Packages ---
# This is placed here to guarantee it's available for the next step.
RUN apt-get update && apt-get install -y libcap-dev && rm -rf /var/lib/apt/lists/* 

# --- 8. Install Python Run-time Dependencies ---
RUN python3 -m pip install --no-cache-dir \
    numpy \
    opencv-python \
    ultralytics \
    torch \
    torchvision \
    pyyaml \
    picamera2

# --- 9. Setup Workspace and Test Script ---
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# --- 10. Set Default Directory and Command ---
WORKDIR /home
CMD ["/bin/bash"]