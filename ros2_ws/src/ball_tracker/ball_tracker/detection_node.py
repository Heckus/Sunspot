#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult # Import this
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class DetectionNode(Node):
    """
    A node that subscribes to raw images, runs YOLO detection, and publishes
    a debug image and structured detection data. Now with dynamic parameters.
    """
    def __init__(self):
        super().__init__('detection_node')

        # Declare parameters for the YOLO model
        self.declare_parameter('yolo_model_path', 'path/to/your/model.pt')
        # We declare the parameter first, then get its value
        self.declare_parameter('confidence_threshold', 0.65)

        # Get the initial parameter value
        model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # --- NEW: Add a parameter callback ---
        self.add_on_set_parameters_callback(self.parameters_callback)
        # ------------------------------------

        # Initialize the YOLO model
        try:
            self.yolo_model = YOLO(model_path)
            self.get_logger().info(f"Successfully loaded YOLO model from: {model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            self.yolo_model = None
            rclpy.shutdown()
            return

        # Create a subscriber to the raw image topic
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)

        # Create publishers
        self.image_publisher_ = self.create_publisher(Image, 'image_detections', 10)
        self.detection_publisher_ = self.create_publisher(Detection2DArray, 'detections_2d', 10)

        self.bridge = CvBridge()
        self.get_logger().info('Detection node has been started.')

    # --- NEW: Parameter callback function ---
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'confidence_threshold':
                self.confidence_threshold = param.value
                self.get_logger().info(f"Set new confidence threshold to: {self.confidence_threshold}")
        return SetParametersResult(successful=True)
    # ---------------------------------------

    def image_callback(self, msg):
        """Callback function to process incoming images."""
        if self.yolo_model is None:
            return

        current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.yolo_model(current_frame, verbose=False)
        result = results[0]

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for box in result.boxes:
            # Use the class member `self.confidence_threshold` which is updated by the callback
            if box.conf[0] > self.confidence_threshold:
                box_coords = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = box_coords

                cv2.rectangle(current_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f"Ball: {box.conf[0]:.2f}"
                cv2.putText(current_frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                detection = Detection2D()
                detection.header = msg.header
                object_hypothesis = ObjectHypothesisWithPose()
                object_hypothesis.hypothesis.class_id = 'volleyball'
                object_hypothesis.hypothesis.score = float(box.conf[0])
                detection.results.append(object_hypothesis)

                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)
                detection_array.detections.append(detection)

        if len(detection_array.detections) > 0:
            self.detection_publisher_.publish(detection_array)

        processed_img_msg = self.bridge.cv2_to_imgmsg(current_frame, "bgr8")
        self.image_publisher_.publish(processed_img_msg)


def main(args=None):
    rclpy.init(args=args)
    detection_node = DetectionNode()
    try:
        rclpy.spin(detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()