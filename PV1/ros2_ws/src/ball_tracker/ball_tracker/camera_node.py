#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import yaml
from picamera2 import Picamera2
import time

class CameraNode(Node):
    """
    A node to capture images from a Picamera2 camera, publish them as a ROS Image message,
    and publish the camera's calibration data.
    """
    def __init__(self):
        super().__init__('camera_node')

        # Declare parameters for configuration
        self.declare_parameter('camera.width', 1920)
        self.declare_parameter('camera.height', 1080)
        self.declare_parameter('camera.framerate', 30.0)
        self.declare_parameter('camera.format', 'BGR888')
        self.declare_parameter('camera_info_url', '') # Path to calibration YAML

        # Get parameters
        self.width = self.get_parameter('camera.width').get_parameter_value().integer_value
        self.height = self.get_parameter('camera.height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('camera.framerate').get_parameter_value().double_value
        self.format = self.get_parameter('camera.format').get_parameter_value().string_value
        self.camera_info_url = self.get_parameter('camera_info_url').get_parameter_value().string_value

        self.get_logger().info(
            f"Camera settings: {self.width}x{self.height} @ {self.framerate} FPS, Format: {self.format}"
        )

        # Create publishers for the raw image and camera info
        self.image_publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.camera_info_publisher_ = self.create_publisher(CameraInfo, 'camera_info', 10)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Load camera calibration data
        self.camera_info_msg = self.load_camera_info(self.camera_info_url)

        # Initialize the camera
        self.picam2 = self.initialize_camera()
        if self.picam2 is None:
            self.get_logger().error("Camera initialization failed. Shutting down.")
            rclpy.shutdown()
            return

        # Set up a timer to capture and publish frames
        timer_period = 1.0 / self.framerate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Camera node has been started and is publishing.')

    def initialize_camera(self):
        """Initializes the Picamera2 instance."""
        try:
            picam2 = Picamera2()
            config = picam2.create_video_configuration(
                main={"size": (self.width, self.height), "format": self.format},
                controls={"FrameRate": self.framerate}
            )
            picam2.configure(config)
            picam2.start()
            # Allow some time for the sensor to stabilize
            time.sleep(1.0)
            self.get_logger().info("Picamera2 started successfully.")
            return picam2
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Picamera2: {e}")
            return None

    def load_camera_info(self, calibration_file):
        """
        Loads camera calibration data from a YAML file compatible with ROS.
        """
        if not calibration_file or not os.path.exists(calibration_file):
            self.get_logger().warning(
                f"Calibration file not found at '{calibration_file}'. Publishing empty CameraInfo."
            )
            return None
        try:
            with open(calibration_file, 'r') as f:
                calib_data = yaml.safe_load(f)

            cam_info = CameraInfo()
            cam_info.width = calib_data.get('image_width', 0)
            cam_info.height = calib_data.get('image_height', 0)
            cam_info.k = calib_data.get('camera_matrix', {}).get('data', [0.0]*9)
            cam_info.d = calib_data.get('distortion_coefficients', {}).get('data', [0.0]*5)
            cam_info.r = calib_data.get('rectification_matrix', {}).get('data', [0.0]*9)
            cam_info.p = calib_data.get('projection_matrix', {}).get('data', [0.0]*12)
            cam_info.distortion_model = calib_data.get('distortion_model', 'plumb_bob')
            self.get_logger().info(f"Successfully loaded camera calibration from {calibration_file}")
            return cam_info

        except Exception as e:
            self.get_logger().error(f"Error loading calibration file '{calibration_file}': {e}")
            return None

    def timer_callback(self):
        """
        Capture a frame, convert it to a ROS message, and publish.
        """
        try:
            # Capture a frame from the camera
            frame = self.picam2.capture_array()

            # The Picamera2 library captures in RGB format.
            # OpenCV (and thus CvBridge) typically expects BGR.
            # However, when we specify 'BGR888' in the config, Picamera2 handles the conversion.
            # If colors are swapped, you may need to manually convert with:
            # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Convert the frame to a ROS Image message
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            current_time = self.get_clock().now().to_msg()
            ros_image_msg.header.stamp = current_time
            ros_image_msg.header.frame_id = 'camera_link' # Or a more descriptive frame name

            # Publish the image message
            self.image_publisher_.publish(ros_image_msg)

            # If camera info is loaded, publish it with the same timestamp
            if self.camera_info_msg:
                self.camera_info_msg.header.stamp = current_time
                self.camera_info_msg.header.frame_id = ros_image_msg.header.frame_id
                self.camera_info_publisher_.publish(self.camera_info_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to capture and publish frame: {e}")

    def destroy_node(self):
        """Gracefully shut down the camera."""
        if self.picam2:
            self.picam2.stop()
            self.get_logger().info("Picamera2 stopped.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()