# Description: ROS 2 Humble with a specific libcamera version (0.5) and camera_ros built from source.
# This approach ensures camera compatibility on devices like the Raspberry Pi 5.
FROM ros:humble-ros-base

# Set environment for non-interactive installs and shell
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- 1. Install ALL Build Dependencies ---
# Get everything needed for libcamera, camera_ros, and your Python packages in one go.
# This creates a more efficient Docker image layer.
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build essentials & source control
    build-essential cmake git pkg-config \
    # Python & ROS build tools
    python3-pip python3-colcon-common-extensions python3-rosdep \
    # libcamera build dependencies
    meson ninja-build libboost-dev libgnutls28-dev libtiff5-dev libdrm-dev libudev-dev libjpeg-dev \
    # Other ROS packages needed
    ros-humble-vision-opencv ros-humble-rmw-cyclonedds-cpp \
    # Dependency for picamera2
    libcap-dev \
    # Optional but useful tools
    i2c-tools v4l-utils \
    # Clean up apt cache
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 2. Build and Install libcamera v0.5 from Source ---
# This step is critical. We build libcamera and install it into the system's
# standard directories (/usr/lib, /usr/include) so that other programs can find it.
WORKDIR /usr/src
RUN git clone https://git.libcamera.org/libcamera/libcamera.git && \
    cd libcamera && \
    git checkout v0.5 && \
    meson build --prefix=/usr -Dpipelines=raspberrypi -Dtest=false -Ddocumentation=false && \
    ninja -C build install && \
    # Update the linker cache to recognize the new libraries
    ldconfig

# --- 3. Install Python Packages ---
# Install these before building the ROS workspace.
RUN python3 -m pip install --no-cache-dir \
    picamera2 \
    ultralytics \
    torch \
    torchvision

# --- 4. Create, Configure, and Build the ROS Workspace ---
# Now that libcamera is installed system-wide, we can build camera_ros.
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/christianrauch/camera_ros.git
WORKDIR /root/ros2_ws
# Initialize rosdep to find dependencies for camera_ros
RUN rosdep init || true && rosdep update
# Install dependencies, skipping libcamera since we built it ourselves.
RUN rosdep install --from-paths src --ignore-src -r -y --skip-keys=libcamera
# Now, finally, build the workspace. This will succeed because libcamera is findable.
RUN . /opt/ros/humble/setup.bash && colcon build --symlink-install

# --- 5. Setup Final Environment ---
# Source both the main ROS setup file and our workspace's setup file.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Set the final working directory
WORKDIR /root/ros2_ws

# Default command to start an interactive bash shell
CMD ["/bin/bash"]