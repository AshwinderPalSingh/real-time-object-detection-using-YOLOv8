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

# OpenCV library (though used less directly now for drawing debug image)
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
        if not os.path.isabs(model_path):
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
            # 1. Convert ROS Image message to OpenCV image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # 2. Run YOLO inference on the OpenCV image
        try:
            results = self.model(cv_image, conf=self.confidence_threshold, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Error during YOLO inference: {e}")
            return

        # --- Prepare messages for publishing ---
        detection_data_msg = Detection2DArray()
        detection_data_msg.header = msg.header # Use header from input image

        # 3. Process detection results to extract data and fill Detection2DArray
        if results[0].boxes is not None:
            for box in results[0].boxes:
                # Extract data needed for the /detections topic
                coords = box.xyxy[0].cpu().numpy()
                xmin, ymin, xmax, ymax = map(int, coords)
                confidence = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                class_name = self.model.names[cls_id]

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

                # Optional: Tracking ID if available
                # if box.id is not None:
                #    detection.id = str(int(box.id[0].cpu().numpy()))

                detection_data_msg.detections.append(detection)

        # 4. Publish the structured detection data
        self.detection_publisher.publish(detection_data_msg)

        # 5. Generate and publish the debug image using Ultralytics plot() function
        try:
            # Use the plot() method from the results object
            # This returns a NumPy array (BGR) with boxes and labels drawn
            annotated_frame = results[0].plot()

            # Convert the annotated frame (NumPy array) to a ROS Image message
            debug_image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
            debug_image_msg.header = msg.header # Ensure header matches original
            self.debug_image_publisher.publish(debug_image_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    if rclpy.ok(): # Check if init was successful
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Shutting down node...')
        finally:
            # Cleanup
            if node and rclpy.ok():
                 node.destroy_node()
                 rclpy.shutdown()

if __name__ == '__main__':
    main()
