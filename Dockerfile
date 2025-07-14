# ROS2 Humble + Pi Camera - No pyv4l2 Version
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/lib:/usr/lib/aarch64-linux-gnu

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools (minimal)
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl \
    # Python and development
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-opencv \
    # Camera support
    libv4l-dev \
    v4l-utils \
    # Hardware access
    i2c-tools \
    udev \
    # Additional utilities
    nano \
    htop \
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

# Install Python dependencies (excluding pyv4l2)
RUN python3 -m pip install --no-cache-dir \
    numpy \
    opencv-python \
    opencv-contrib-python \
    Pillow \
    scipy \
    matplotlib \
    ultralytics \
    torch \
    torchvision \
    smbus2 \
    gpiozero \
    transforms3d \
    python-dateutil \
    pyyaml

# Create ROS2 workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc 

# Create camera test script using OpenCV only
RUN echo '#!/usr/bin/env python3\n\
import cv2\n\
import sys\n\
import subprocess\n\
\n\
def list_v4l2_devices():\n\
    """List available V4L2 devices using system command"""\n\
    try:\n\
        result = subprocess.run(["v4l2-ctl", "--list-devices"], \n\
                              capture_output=True, text=True)\n\
        if result.returncode == 0:\n\
            print("V4L2 devices:")\n\
            print(result.stdout)\n\
        else:\n\
            print("No V4L2 devices found or v4l2-ctl failed")\n\
    except FileNotFoundError:\n\
        print("v4l2-ctl not found")\n\
\n\
def test_camera():\n\
    print("Testing camera access...")\n\
    \n\
    # First list V4L2 devices\n\
    list_v4l2_devices()\n\
    \n\
    # Try different camera indices\n\
    for i in range(5):\n\
        print(f"Trying camera {i}...")\n\
        cap = cv2.VideoCapture(i)\n\
        if cap.isOpened():\n\
            ret, frame = cap.read()\n\
            if ret:\n\
                print(f"Camera {i} working! Frame shape: {frame.shape}")\n\
                # Try to get some camera properties\n\
                width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)\n\
                height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)\n\
                fps = cap.get(cv2.CAP_PROP_FPS)\n\
                print(f"Resolution: {width}x{height}, FPS: {fps}")\n\
                cap.release()\n\
                return i\n\
            else:\n\
                print(f"Camera {i} opened but no frame")\n\
        cap.release()\n\
    \n\
    print("No working cameras found")\n\
    return None\n\
\n\
if __name__ == "__main__":\n\
    camera_id = test_camera()\n\
    if camera_id is not None:\n\
        print(f"Use camera {camera_id} for your applications")\n\
        sys.exit(0)\n\
    else:\n\
        print("No cameras available")\n\
        sys.exit(1)\n\
' > /test_camera.py && chmod +x /test_camera.py

# Create ROS2 camera node example
RUN echo '#!/usr/bin/env python3\n\
import rclpy\n\
from rclpy.node import Node\n\
from sensor_msgs.msg import Image\n\
from cv_bridge import CvBridge\n\
import cv2\n\
\n\
class CameraNode(Node):\n\
    def __init__(self):\n\
        super().__init__(\"camera_node\")\n\
        self.publisher = self.create_publisher(Image, \"camera/image\", 10)\n\
        self.bridge = CvBridge()\n\
        self.cap = cv2.VideoCapture(0)\n\
        \n\
        if not self.cap.isOpened():\n\
            self.get_logger().error("Could not open camera")\n\
            return\n\
        \n\
        self.timer = self.create_timer(0.1, self.publish_frame)\n\
        self.get_logger().info("Camera node started")\n\
    \n\
    def publish_frame(self):\n\
        ret, frame = self.cap.read()\n\
        if ret:\n\
            msg = self.bridge.cv2_to_imgmsg(frame, \"bgr8\")\n\
            msg.header.stamp = self.get_clock().now().to_msg()\n\
            self.publisher.publish(msg)\n\
        else:\n\
            self.get_logger().warn("Failed to capture frame")\n\
    \n\
    def destroy_node(self):\n\
        self.cap.release()\n\
        super().destroy_node()\n\
\n\
def main():\n\
    rclpy.init()\n\
    node = CameraNode()\n\
    try:\n\
        rclpy.spin(node)\n\
    except KeyboardInterrupt:\n\
        pass\n\
    finally:\n\
        node.destroy_node()\n\
        rclpy.shutdown()\n\
\n\
if __name__ == "__main__":\n\
    main()\n\
' > /camera_node.py && chmod +x /camera_node.py

# Create comprehensive test script
RUN echo '#!/bin/bash\n\
echo "=== ROS2 Pi Camera Container Test (No pyv4l2 Version) ==="\n\
echo ""\n\
echo "=== System Info ==="\n\
uname -a\n\
echo ""\n\
echo "=== V4L2 devices ==="\n\
v4l2-ctl --list-devices 2>/dev/null || echo "V4L2 not available"\n\
echo ""\n\
echo "=== Video devices ==="\n\
ls -la /dev/video* 2>/dev/null || echo "No video devices found"\n\
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
echo "=== Python packages test ==="\n\
python3 -c "import cv2; print(f\"OpenCV version: {cv2.__version__}\")" 2>/dev/null || echo "OpenCV import failed"\n\
python3 -c "import torch; print(f\"PyTorch version: {torch.__version__}\")" 2>/dev/null || echo "PyTorch import failed"\n\
python3 -c "import ultralytics; print(f\"Ultralytics version: {ultralytics.__version__}\")" 2>/dev/null || echo "Ultralytics import failed"\n\
python3 -c "from cv_bridge import CvBridge; print(\"cv_bridge available\")" 2>/dev/null || echo "cv_bridge import failed"\n\
echo ""\n\
echo "=== Camera test ==="\n\
python3 /test_camera.py\n\
echo ""\n\
echo "=== Test YOLO model download ==="\n\
python3 -c "from ultralytics import YOLO; model = YOLO(\"yolov8n.pt\"); print(\"YOLO model loaded successfully\")" 2>/dev/null || echo "YOLO model test failed"\n\
echo ""\n\
echo "=== ROS2 V4L2 camera test ==="\n\
echo "Testing ROS2 v4l2_camera node..."\n\
timeout 5 ros2 run v4l2_camera v4l2_camera_node 2>/dev/null || echo "v4l2_camera node test failed or no camera"\n\
echo ""\n\
echo "=== Test complete ==="\n\
echo "NOTE: This version uses OpenCV and ROS2 v4l2_camera for camera access."\n\
echo "No pyv4l2 dependency - should work more reliably."\n\
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