# ROS2 Humble + libcamera for Raspberry Pi 5 with HQ Camera (Fixed)
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LIBCAMERA_LOG_LEVELS=*:INFO
ENV LD_LIBRARY_PATH=/usr/local/lib:/usr/lib/aarch64-linux-gnu
ENV PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/usr/lib/aarch64-linux-gnu/pkgconfig

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    meson \
    ninja-build \
    pkg-config \
    git \
    wget \
    curl \
    # Python and development
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-opencv \
    # libcamera core dependencies (minimal set)
    libyaml-dev \
    libssl-dev \
    libboost-program-options-dev \
    libdrm-dev \
    libjpeg-dev \
    libtiff-dev \
    libpng-dev \
    libudev-dev \
    # FIX: Add libcap-dev for python-prctl dependency
    libcap-dev \
    # OpenCV dependencies for computer vision
    libopencv-dev \
    libopencv-contrib-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    # Hardware access
    i2c-tools \
    udev \
    # Camera tools
    v4l-utils \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-camera-info-manager \
    ros-humble-image-proc \
    ros-humble-vision-opencv \
    ros-humble-usb-cam \
    ros-humble-v4l2-camera \
    ros-humble-camera-calibration \
    ros-humble-image-view \
    ros-humble-rqt-image-view \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-diagnostic-msgs \
    ros-humble-diagnostic-updater \
    python3-colcon-common-extensions \
    python3-rosdep \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for computer vision and YOLO
RUN python3 -m pip install --no-cache-dir \
    numpy \
    opencv-python \
    opencv-contrib-python \
    Pillow \
    scipy \
    matplotlib \
    # YOLO and AI dependencies
    ultralytics \
    torch \
    torchvision \
    # Picamera2 for Pi camera control
    picamera2 \
    # Hardware monitoring
    smbus2 \
    gpiozero \
    # Additional utilities
    transforms3d \
    python-dateutil \
    pyyaml

# --- IMPROVED libcamera build ---
# More robust configuration with better error handling
RUN git clone https://git.libcamera.org/libcamera/libcamera.git && \
    cd libcamera && \
    # Use a more stable release
    git checkout v0.2.0 && \
    # Configure with more explicit options
    meson setup build \
        --buildtype=release \
        -Dv4l2=true \
        -Dtest=false \
        -Ddocumentation=false \
        -Dpycamera=enabled \
        -Dpipelines=rpi/vc4 \
        -Dipas=rpi/vc4 \
        -Dgstreamer=disabled \
        -Dqcam=disabled \
        -Dlc-compliance=disabled \
        -Dcam=enabled && \
    ninja -C build && \
    ninja -C build install && \
    ldconfig && \
    cd .. && rm -rf libcamera

# --- IMPROVED libcamera-apps build ---
# More conservative build with better compatibility
RUN git clone https://github.com/raspberrypi/libcamera-apps.git && \
    cd libcamera-apps && \
    # Use a more stable version
    git checkout v1.5.0 && \
    # Configure with minimal features for maximum compatibility
    meson setup build \
        --buildtype=release \
        -Denable_libav=false \
        -Denable_drm=false \
        -Denable_egl=false \
        -Denable_qt=false \
        -Denable_opencv=true \
        -Denable_tflite=false \
        -Denable_hailo=false && \
    ninja -C build && \
    ninja -C build install && \
    ldconfig && \
    cd .. && rm -rf libcamera-apps

# Create ROS2 workspace and add camera nodes
WORKDIR /ros2_ws
RUN mkdir -p src

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc 

# Create comprehensive test script
RUN echo '#!/bin/bash\n\
echo "=== ROS2 Pi5 Camera + YOLO Container Test ==="\n\
echo ""\n\
echo "=== System Info ==="\n\
uname -a\n\
echo ""\n\
echo "=== libcamera version ==="\n\
libcamera-hello --version 2>/dev/null || echo "libcamera-hello not found"\n\
echo ""\n\
echo "=== Available cameras ==="\n\
libcamera-hello --list-cameras 2>/dev/null || echo "No cameras detected"\n\
echo ""\n\
echo "=== V4L2 devices ==="\n\
v4l2-ctl --list-devices 2>/dev/null || echo "V4L2 not available"\n\
echo ""\n\
echo "=== I2C devices ==="\n\
i2cdetect -l 2>/dev/null || echo "I2C not available"\n\
echo ""\n\
echo "=== ROS2 version ==="\n\
ros2 --version\n\
echo ""\n\
echo "=== ROS2 camera packages ==="\n\
ros2 pkg list | grep -E "(camera|image|vision)"\n\
echo ""\n\
echo "=== Test HQ camera (5 seconds) ==="\n\
timeout 8 libcamera-hello --timeout 5000 --nopreview 2>/dev/null || echo "Camera test failed"\n\
echo ""\n\
echo "=== Python packages test ==="\n\
python3 -c "import cv2; print(f\"OpenCV version: {cv2.__version__}\")" 2>/dev/null || echo "OpenCV import failed"\n\
python3 -c "import torch; print(f\"PyTorch version: {torch.__version__}\")" 2>/dev/null || echo "PyTorch import failed"\n\
python3 -c "import ultralytics; print(f\"Ultralytics version: {ultralytics.__version__}\")" 2>/dev/null || echo "Ultralytics import failed"\n\
python3 -c "from picamera2 import Picamera2; print(\"Picamera2: OK\")" 2>/dev/null || echo "Picamera2 import failed"\n\
python3 -c "import libcamera; print(\"libcamera Python bindings: OK\")" 2>/dev/null || echo "libcamera Python bindings not available"\n\
echo ""\n\
echo "=== Test YOLO model download ==="\n\
python3 -c "from ultralytics import YOLO; model = YOLO(\"yolov8n.pt\"); print(\"YOLO model loaded successfully\")" 2>/dev/null || echo "YOLO model test failed"\n\
echo ""\n\
echo "=== Test complete ==="\n\
' > /test_setup.sh && chmod +x /test_setup.sh

# Create UPS monitoring script
RUN echo '#!/usr/bin/env python3\n\
import smbus2\n\
import time\n\
\n\
def read_ups_data():\n\
    try:\n\
        bus = smbus2.SMBus(1)\n\
        # Basic UPS detection\n\
        devices = []\n\
        for addr in range(0x08, 0x78):\n\
            try:\n\
                bus.read_byte(addr)\n\
                devices.append(hex(addr))\n\
            except:\n\
                pass\n\
        \n\
        if devices:\n\
            print(f"I2C devices detected: {devices}")\n\
            print("UPS may be at one of these addresses")\n\
        else:\n\
            print("No I2C devices detected")\n\
        \n\
        bus.close()\n\
    except Exception as e:\n\
        print(f"Error: {e}")\n\
        print("Make sure I2C is enabled")\n\
\n\
if __name__ == "__main__":\n\
    read_ups_data()\n\
' > /test_ups.py && chmod +x /test_ups.py

# Default working directory
WORKDIR /ros2_ws

# Default command
CMD ["/bin/bash"]