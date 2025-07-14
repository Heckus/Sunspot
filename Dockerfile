# Stage 1: Build ROS 2 from source on a Debian Bookworm base
FROM debian:bookworm as ros2_builder

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget

# Initialize rosdep
RUN rosdep init && rosdep update

# Create a ROS 2 workspace
WORKDIR /ros2_ws
RUN wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos \
    && vcs import src < ros2.repos

# Install ROS 2 dependencies
RUN rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1"

# Build the ROS 2 workspace
RUN . /usr/share/colcon_cd/function/colcon_cd.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---

# Stage 2: Create the final runtime image
FROM debian:bookworm-slim

# Set environment
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8

# Install all runtime dependencies we discovered previously
RUN apt-get update && apt-get install -y \
    # Build tools for python packages
    build-essential \
    cmake \
    # Dependencies for libcamera & python
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
    python3-pip \
    python3-dev \
    python3-opencv \
    i2c-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install python build tools
RUN pip3 install --upgrade --break-system-packages meson jinja2 ply pyyaml

# Build and install libcamera & apps
WORKDIR /usr/src
RUN git clone https://github.com/raspberrypi/libcamera.git && \
    cd libcamera && \
    meson build -Dpipelines=rpi/pisp -Dtest=false -Dv4l2=true && \
    ninja -C build install
WORKDIR /usr/src
RUN git clone https://github.com/raspberrypi/libcamera-apps.git && \
    cd libcamera-apps && \
    meson build && \
    ninja -C build install
RUN ldconfig && \
    rm -rf /usr/src/libcamera /usr/src/libcamera-apps

# Copy the compiled ROS 2 from the builder stage
COPY --from=ros2_builder /ros2_ws /ros2_ws

# Install final Python runtime packages
RUN pip3 install --no-cache-dir --break-system-packages \
       picamera2 \
       ultralytics \
       torch \
       torchvision

# Setup workspace and test script
WORKDIR /ros2_ws
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
COPY test_camera_stack.sh /test_camera_stack.sh
RUN chmod +x /test_camera_stack.sh

# Set default directory and command
WORKDIR /ros2_ws
CMD ["/bin/bash"]