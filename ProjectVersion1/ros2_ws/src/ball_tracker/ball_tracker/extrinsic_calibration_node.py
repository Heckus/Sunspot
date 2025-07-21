#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import yaml

class ExtrinsicCalibrationNode(Node):
    """
    A node to perform 6-point extrinsic calibration. It subscribes to an image topic,
    allows the user to click on 6 points, calculates the camera's pose (rvec and tvec)
    using solvePnP, and saves the result to a YAML file.
    """
    def __init__(self):
        super().__init__('extrinsic_calibration_node')

        # Declare parameters
        self.declare_parameter('output_file_path', 'src/ball_tracker/config/extrinsics.yaml')
        self.declare_parameter('world_points', [
            0.0, 0.0, 0.0,  # 1. Bottom-Front-Left
            1.0, 0.0, 0.0,  # 2. Bottom-Front-Right
            1.0, 1.0, 0.0,  # 3. Bottom-Back-Right
            0.0, 1.0, 0.0,  # 4. Bottom-Back-Left
            0.0, 1.0, 1.0,  # 5. Top-Back-Left
            1.0, 1.0, 1.0   # 6. Top-Back-Right
        ])

        # Get parameters
        self.output_file = self.get_parameter('output_file_path').get_parameter_value().string_value
        world_points_flat = self.get_parameter('world_points').get_parameter_value().double_array_value
        self.world_points = np.array(world_points_flat, dtype=np.float32).reshape((6, 3))

        # Class variables
        self.bridge = CvBridge()
        self.intrinsics_received = False
        self.camera_matrix = None
        self.dist_coeffs = None
        self.clicked_points = []
        self.window_name = "Extrinsic Calibration - Click 6 Points"
        
        # Subscribers
        self.image_subscriber = self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        self.camera_info_subscriber = self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, 1)

        self.get_logger().info('Extrinsic calibration node started. Waiting for image and camera info...')
        self.print_instructions()

    def print_instructions(self):
        self.get_logger().info("--- Extrinsic Calibration Instructions ---")
        self.get_logger().info(f"Click on the {len(self.world_points)} specified corners of your reference object.")
        self.get_logger().info("Clicking Order:")
        self.get_logger().info(f"1. Point at {self.world_points[0].tolist()} (e.g., Bottom-Front-Left)")
        self.get_logger().info(f"2. Point at {self.world_points[1].tolist()} (e.g., Bottom-Front-Right)")
        self.get_logger().info(f"3. Point at {self.world_points[2].tolist()} (e.g., Bottom-Back-Right)")
        self.get_logger().info(f"4. Point at {self.world_points[3].tolist()} (e.g., Bottom-Back-Left)")
        self.get_logger().info(f"5. Point at {self.world_points[4].tolist()} (e.g., Top-Back-Left)")
        self.get_logger().info(f"6. Point at {self.world_points[5].tolist()} (e.g., Top-Back-Right)")
        self.get_logger().info("-----------------------------------------")
        self.get_logger().info("After clicking all points, press 'c' to calculate.")
        self.get_logger().info("Press 'r' to restart point selection.")
        self.get_logger().info("Press 'q' to quit.")

    def camera_info_callback(self, msg):
        """Stores camera intrinsic parameters and then unsubscribes."""
        if not self.intrinsics_received:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.intrinsics_received = True
            self.get_logger().info('Camera intrinsics received.')
            self.destroy_subscription(self.camera_info_subscriber)

    def image_callback(self, msg):
        """Receives an image, displays it, and handles the calibration logic."""
        if not self.intrinsics_received:
            self.get_logger().warn('No camera intrinsics yet, skipping image.', throttle_skip_first=True, throttle_duration_sec=5)
            return

        # We only need one good image for calibration
        self.get_logger().info('Image received. Displaying for calibration.')
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Unsubscribe from the image topic now that we have our frame
        self.destroy_subscription(self.image_subscriber)

        # Set up the GUI
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback, frame)
        
        self.run_calibration_loop(frame)

    def mouse_callback(self, event, x, y, flags, param_frame):
        """Handles mouse clicks to collect points."""
        if event == cv2.EVENT_LBUTTONDOWN and len(self.clicked_points) < len(self.world_points):
            self.clicked_points.append((x, y))
            # Draw on the image passed as param
            cv2.circle(param_frame, (x, y), 7, (0, 255, 0), -1)
            label = f"P{len(self.clicked_points)}"
            cv2.putText(param_frame, label, (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (50, 200, 50), 2)
            self.get_logger().info(f"Clicked point {len(self.clicked_points)}: ({x}, {y})")

    def run_calibration_loop(self, frame):
        """Manages the user interaction loop for calibration."""
        original_frame = frame.copy()
        
        while rclpy.ok():
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(100) & 0xFF

            if key == ord('q'):
                break
            
            if key == ord('r'):
                self.get_logger().info("Resetting points.")
                self.clicked_points = []
                frame = original_frame.copy()

            if len(self.clicked_points) == len(self.world_points) and key == ord('c'):
                self.calculate_and_save_pose(frame)
                # Keep showing the result until the user quits or accepts
        
        self.get_logger().info("Shutting down extrinsic calibrator.")
        cv2.destroyAllWindows()
        rclpy.shutdown()

    def calculate_and_save_pose(self, frame):
        """Performs the solvePnP calculation and saves the result."""
        self.get_logger().info("Calculating camera pose...")
        image_points_np = np.array(self.clicked_points, dtype=np.float32)

        try:
            retval, rvec, tvec = cv2.solvePnP(
                self.world_points, image_points_np, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if not retval:
                self.get_logger().error("solvePnP failed. Check your points and try again.")
                return

            reprojection_error = self.calculate_reprojection_error(rvec, tvec, image_points_np)
            self.get_logger().info(f"solvePnP successful! Reprojection error: {reprojection_error:.4f} pixels.")

            # Draw the world axes on the image to verify
            self.draw_axes(frame, rvec, tvec)
            cv2.putText(frame, "OK? Press 'y' to save, 'r' to retry.", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
            cv2.imshow(self.window_name, frame)
            
            # Wait for confirmation
            while rclpy.ok():
                confirm_key = cv2.waitKey(100) & 0xFF
                if confirm_key == ord('y'):
                    self.save_extrinsics(rvec, tvec)
                    break
                elif confirm_key == ord('r'):
                    self.get_logger().info("Retrying. Resetting points.")
                    self.clicked_points = [] # Allow re-clicking
                    # Restore the original frame view (without axes)
                    original_frame = frame.copy() # Need to get the clean frame again
                    cv2.imshow(self.window_name, original_frame)
                    return # Exit this function to allow re-clicking on a clean slate.

        except Exception as e:
            self.get_logger().error(f"An error occurred during calibration: {e}")

    def save_extrinsics(self, rvec, tvec):
        """Saves the rotation and translation vectors to a YAML file."""
        self.get_logger().info(f"Saving extrinsics to: {self.output_file}")
        
        # Ensure the directory exists
        output_dir = os.path.dirname(self.output_file)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        rvec_list = rvec.flatten().tolist()
        tvec_list = tvec.flatten().tolist()

        data = {
            'camera_extrinsic': {
                'rvec': rvec_list,
                'tvec': tvec_list
            }
        }

        with open(self.output_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        
        self.get_logger().info("Extrinsics saved successfully. You can now close the node (q or Ctrl+C).")
    
    def calculate_reprojection_error(self, rvec, tvec, image_points):
        """Calculates the reprojection error to evaluate calibration quality."""
        projected_points, _ = cv2.projectPoints(self.world_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        error = cv2.norm(image_points, projected_points.reshape(-1, 2), cv2.NORM_L2) / len(projected_points)
        return error

    def draw_axes(self, frame, rvec, tvec):
        """Draws the X, Y, Z axes of the world origin on the image."""
        axis_points_3d = np.float32([[0,0,0], [0.5,0,0], [0,0.5,0], [0,0,0.5]]).reshape(-1,3) # 50cm axes
        image_points, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        image_points = np.int32(image_points.reshape(-1, 2))
        
        origin, x_end, y_end, z_end = image_points
        cv2.line(frame, tuple(origin), tuple(x_end), (0,0,255), 3) # X=Red
        cv2.line(frame, tuple(origin), tuple(y_end), (0,255,0), 3) # Y=Green
        cv2.line(frame, tuple(origin), tuple(z_end), (255,0,0), 3) # Z=Blue

def main(args=None):
    rclpy.init(args=args)
    extrinsic_calibrator = ExtrinsicCalibrationNode()
    rclpy.spin(extrinsic_calibrator)

if __name__ == '__main__':
    main()