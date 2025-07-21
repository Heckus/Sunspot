#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class FrameSaverNode(Node):
    """
    Subscribes to a ROS Image topic and saves the frames to a specified
    directory when enabled. Designed for saving raw footage to a USB drive.
    """
    def __init__(self):
        super().__init__('frame_saver_node')

        # Declare parameters
        self.declare_parameter('save_path', '/media/usb_drive/ros_images')
        self.declare_parameter('saving_enabled', False)

        # Get parameters
        self.save_path = self.get_parameter('save_path').get_parameter_value().string_value
        self.saving_enabled = self.get_parameter('saving_enabled').get_parameter_value().bool_value
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create a subscriber to the raw image topic
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10) # QoS profile depth

        # Ensure the save directory exists
        if self.saving_enabled:
            self.setup_save_directory()

        self.get_logger().info('Frame saver node started.')
        if self.saving_enabled:
            self.get_logger().info(f"Saving frames to: {self.save_path}")
        else:
            self.get_logger().info("Frame saving is currently DISABLED. Set 'saving_enabled' to True to start.")


    def setup_save_directory(self):
        """Creates the directory for saving images if it doesn't exist."""
        try:
            if not os.path.exists(self.save_path):
                os.makedirs(self.save_path)
                self.get_logger().info(f"Created save directory: {self.save_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to create save directory '{self.save_path}': {e}")
            self.saving_enabled = False # Disable saving if directory can't be made


    def image_callback(self, msg):
        """Callback function to save the incoming image if enabled."""
        if not self.saving_enabled:
            return

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Generate a unique filename based on the current timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(self.save_path, f"frame_{timestamp}.jpg")
            
            # Save the image
            # Using a separate thread could improve performance for high-frequency topics
            # by offloading the file I/O, but for simplicity, we do it directly.
            success = cv2.imwrite(filename, cv_image)
            
            if not success:
                 self.get_logger().warn(f"Failed to save frame to {filename}")

        except Exception as e:
            self.get_logger().error(f"Error processing and saving image: {e}")

def main(args=None):
    rclpy.init(args=args)
    frame_saver_node = FrameSaverNode()
    try:
        rclpy.spin(frame_saver_node)
    except KeyboardInterrupt:
        pass
    finally:
        frame_saver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()