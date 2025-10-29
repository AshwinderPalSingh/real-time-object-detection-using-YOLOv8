import rclpy
from rclpy.node import Node

# Import the specific message type for detection data
from vision_msgs.msg import Detection2DArray
# Optional: Import Twist if you want to publish robot commands
# from geometry_msgs.msg import Twist

class DecisionMaker(Node):
    def __init__(self):
        super().__init__('decision_maker_node')
        self.get_logger().info('Decision Maker Node has started.')

        # Declare parameter for minimum confidence
        self.declare_parameter('min_confidence', 0.50)
        self.min_confidence = self.get_parameter('min_confidence').get_parameter_value().double_value
        self.get_logger().info(f'Minimum confidence threshold set to: {self.min_confidence}')


        # Subscribe to the '/detections' topic
        self.subscription = self.create_subscription(
            Detection2DArray,         # Message Type
            '/detections',            # Topic Name
            self.detection_callback,  # Callback function
            10)                       # QoS depth
        self.subscription # prevent unused variable warning

        # --- Optional: Publisher for robot commands ---
        # self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def detection_callback(self, msg: Detection2DArray):
        """
        Processes incoming detection data and makes a simple decision.
        """
        car_detected = False
        person_detected = False
        num_cars = 0
        num_people = 0

        # Loop through detected objects
        for detection in msg.detections:
            if not detection.results:
                continue

            top_hypothesis = detection.results[0].hypothesis
            class_name = top_hypothesis.class_id
            score = top_hypothesis.score

            # Check for cars or people above the confidence threshold
            if class_name == 'car' and score >= self.min_confidence:
                car_detected = True
                num_cars += 1
            elif class_name == 'person' and score >= self.min_confidence:
                person_detected = True
                num_people += 1

        # Make and log the decision
        if car_detected or person_detected:
            self.get_logger().warn(f'Objects detected! Cars: {num_cars}, People: {num_people}. STOPPING!')
            # --- Add robot stop command logic here ---
            # stop_msg = Twist()
            # self.cmd_vel_publisher.publish(stop_msg)
        else:
            self.get_logger().info('Path is clear. Moving forward.')
            # --- Add robot move command logic here ---
            # move_msg = Twist()
            # move_msg.linear.x = 0.5
            # self.cmd_vel_publisher.publish(move_msg)

def main(args=None):
    rclpy.init(args=args)
    decision_maker = DecisionMaker()
    try:
        rclpy.spin(decision_maker)
    except KeyboardInterrupt:
        decision_maker.get_logger().info('Shutting down Decision Maker node...')
    finally:
        if decision_maker and rclpy.ok():
            decision_maker.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
