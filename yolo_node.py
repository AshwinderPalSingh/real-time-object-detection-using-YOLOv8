# Needed ROS 2 libraries
import rclpy
from rclpy.node import Node

# Messages for Image data and standard Vision messages for detections
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
# Uncomment if Pose2D is needed (usually not required for basic bbox)
# from geometry_msgs.msg import Pose2D

# Tool to convert between ROS Image messages and OpenCV images
from cv_bridge import CvBridge

# OpenCV library
import cv2

# YOLO library from Ultralytics
from ultralytics import YOLO
import os # To potentially make model path relative

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.get_logger().info('YOLO Detector Node has started.')

        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()

        # --- Parameters ---
        # Declare parameters for model path and confidence threshold
        self.declare_parameter('model_path', 'yolov8n.pt') # Default model
        self.declare_parameter('confidence_threshold', 0.5) # Default confidence

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # --- Load Model ---
        # Ensure 'yolov8n.pt' (or specified model) is accessible
        # Consider making the path relative or using find_package_share if model is packaged
        if not os.path.isabs(model_path):
             # Basic check if it's just the filename, could be improved
             # In a real package, you might put models in share/yolo_detector/models
             self.get_logger().info(f"Assuming model '{model_path}' is in current directory or PATH.")
        
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLOv8 model loaded successfully from: {model_path}')
        except Exception as e:
            self.get_logger().error(f"Error loading YOLO model from '{model_path}': {e}")
            rclpy.shutdown()
            return

        # --- Subscriber ---
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',            # Topic name for input images
            self.image_callback,     # Function to call when an image arrives
            10)                      # QoS profile depth

        # --- Publisher 1: Debug Image ---
        self.debug_image_publisher = self.create_publisher(
            Image,
            '/detections_image',     # Topic name for viewing in RQT
            10)                      # QoS profile depth

        # --- Publisher 2: Detection Data ---
        self.detection_publisher = self.create_publisher(
            Detection2DArray,        # Message type from vision_msgs
            '/detections',           # Topic name for structured data
            10)                      # QoS profile depth

        self.get_logger().info('Subscribers and Publishers initialized.')
        self.get_logger().info(f'Confidence Threshold set to: {self.confidence_threshold}')


    def image_callback(self, msg: Image):
        self.get_logger().debug('Received image.')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Run YOLO inference
        try:
            # Pass confidence threshold to the model prediction
            results = self.model(cv_image, conf=self.confidence_threshold, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Error during YOLO inference: {e}")
            return

        # Prepare messages
        detection_data_msg = Detection2DArray()
        detection_data_msg.header = msg.header

        # Process results
        if results[0].boxes is not None:
            for box in results[0].boxes:
                coords = box.xyxy[0].cpu().numpy()
                xmin, ymin, xmax, ymax = map(int, coords)
                confidence = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                class_name = self.model.names[cls_id]

                # Draw on debug image
                cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                label = f"{class_name}: {confidence:.2f}"
                cv2.putText(cv_image, label, (xmin, ymin - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Fill structured data message
                detection = Detection2D()
                detection.header = msg.header
                detection.bbox.center.position.x = float((xmin + xmax) / 2.0)
                detection.bbox.center.position.y = float((ymin + ymax) / 2.0)
                detection.bbox.size_x = float(xmax - xmin)
                detection.bbox.size_y = float(ymax - ymin)

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = class_name
                hypothesis.hypothesis.score = confidence
                detection.results.append(hypothesis)

                detection_data_msg.detections.append(detection)

        # Publish structured data
        self.detection_publisher.publish(detection_data_msg)

        # Publish debug image
        try:
            debug_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            debug_image_msg.header = msg.header
            self.debug_image_publisher.publish(debug_image_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    if rclpy.ok(): # Check if init was successful before spinning
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Shutting down node...')
        finally:
            # Ensure destroy_node() is called only if the node was created
            if node and rclpy.ok():
                 node.destroy_node()
                 rclpy.shutdown()

if __name__ == '__main__':
    main()
