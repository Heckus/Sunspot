#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np

class CalculationNode(Node):
    """
    Subscribes to 2D detections and camera info to calculate the 3D position of the ball.
    Publishes the 3D position as a PoseStamped message and a visualization Marker.
    """
    def __init__(self):
        super().__init__('calculation_node')

        # Declare a parameter for the known physical radius of the ball (in meters)
        self.declare_parameter('ball_radius_m', 0.105) # Standard volleyball radius
        self.ball_radius = self.get_parameter('ball_radius_m').get_parameter_value().double_value

        # Subscribers
        self.detection_subscriber = self.create_subscription(
            Detection2DArray,
            'detections_2d',
            self.detection_callback,
            10)
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.camera_info_callback,
            10)

        # Publishers
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'ball_pose', 10)
        self.marker_publisher_ = self.create_publisher(Marker, 'ball_marker', 10)

        # Instance variables to store camera intrinsics
        self.camera_matrix = None
        self.focal_length_x = None
        self.focal_length_y = None
        self.principal_point_x = None
        self.principal_point_y = None

        self.get_logger().info('Calculation node has been started.')

    def camera_info_callback(self, msg):
        """Stores the camera intrinsic parameters."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.focal_length_x = self.camera_matrix[0, 0]
            self.focal_length_y = self.camera_matrix[1, 1]
            self.principal_point_x = self.camera_matrix[0, 2]
            self.principal_point_y = self.camera_matrix[1, 2]
            self.get_logger().info('Camera intrinsics received and stored.')
            # Unsubscribe after receiving the info once, as it's static
            self.destroy_subscription(self.camera_info_subscriber)


    def detection_callback(self, msg):
        """Processes detections and calculates 3D position."""
        if self.camera_matrix is None:
            self.get_logger().warning('Waiting for camera intrinsics... Cannot calculate position.')
            return

        # For this project, assume the most confident detection is the ball
        best_detection = None
        highest_score = -1.0
        for detection in msg.detections:
            score = detection.results[0].hypothesis.score
            if score > highest_score:
                highest_score = score
                best_detection = detection

        if best_detection:
            # --- Estimate Distance (Z-depth) ---
            # Use the average of bounding box width and height for apparent radius in pixels
            apparent_radius_px = (best_detection.bbox.size_x + best_detection.bbox.size_y) / 4.0
            
            if apparent_radius_px <= 0:
                return

            # Average focal length
            avg_focal_length = (self.focal_length_x + self.focal_length_y) / 2.0
            
            # Pinhole camera model formula to find distance (Z)
            distance_z = (self.ball_radius * avg_focal_length) / apparent_radius_px

            # --- Estimate X and Y in the camera frame ---
            u = best_detection.bbox.center.position.x
            v = best_detection.bbox.center.position.y

            # Inverse projection to find 3D coordinates in the camera's frame
            cam_x = (u - self.principal_point_x) / self.focal_length_x * distance_z
            cam_y = (v - self.principal_point_y) / self.focal_length_y * distance_z
            cam_z = distance_z
            
            # --- Publish the results ---
            self.publish_pose(msg.header, cam_x, cam_y, cam_z)
            self.publish_marker(msg.header, cam_x, cam_y, cam_z)


    def publish_pose(self, header, x, y, z):
        """Publishes the calculated 3D position as a PoseStamped message."""
        pose_msg = PoseStamped()
        pose_msg.header = header # Use the same timestamp and frame_id as the detection
        
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        # Orientation is left as default (0,0,0,1 - no rotation)
        pose_msg.pose.orientation.w = 1.0

        self.pose_publisher_.publish(pose_msg)

    def publish_marker(self, header, x, y, z):
        """Publishes a visualization marker for RViz."""
        marker_msg = Marker()
        marker_msg.header = header
        marker_msg.ns = "ball_tracker"
        marker_msg.id = 0
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        
        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = z
        marker_msg.pose.orientation.w = 1.0

        # Set the scale of the marker (diameter)
        marker_msg.scale.x = self.ball_radius * 2.0
        marker_msg.scale.y = self.ball_radius * 2.0
        marker_msg.scale.z = self.ball_radius * 2.0

        # Set the color (e.g., yellow)
        marker_msg.color.r = 1.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 0.8 # Alpha (transparency)

        marker_msg.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

        self.marker_publisher_.publish(marker_msg)


def main(args=None):
    rclpy.init(args=args)
    calculation_node = CalculationNode()
    try:
        rclpy.spin(calculation_node)
    except KeyboardInterrupt:
        pass
    finally:
        calculation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()