# Import necessary modules
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Node class for ROS 2
from sensor_msgs.msg import Image  # Message type for image data
from std_msgs.msg import String  # Message type for string data
from cv_bridge import CvBridge  # Bridge between ROS and OpenCV
import cv2  # OpenCV for image processing
import mediapipe as mp  # Mediapipe library for hand detection

# Define a node class for hand detection
class HandDetector(Node):
    def __init__(self):
        # Initialize the node with the name 'hands'
        super().__init__('hands')
        
        # Create a subscription to the 'video_frames' topic with the Image message type
        # Calls 'listener_callback' whenever a new message is received
        self.subscription = self.create_subscription(Image, 'video_frames', self.listener_callback, 10)
        
        # Create a publisher for the 'cmd_hand' topic with the String message type
        self.publisher_ = self.create_publisher(String, 'cmd_hand', 10)
        
        # Initialize the CvBridge for converting ROS Image messages to OpenCV format
        self.br = CvBridge()
        
        # Initialize Mediapipe's Hands solution for hand detection
        self.mp_hands = mp.solutions.hands.Hands()
        
        # Initialize Mediapipe's drawing utilities for visualizing hand landmarks
        self.mp_draw = mp.solutions.drawing_utils

    def listener_callback(self, data):
        # Convert the incoming ROS Image message to an OpenCV format image
        frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        
        # Process the image to detect hands using Mediapipe
        results = self.mp_hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        
        # If hand landmarks are detected, publish a message and log the detection
        if results.multi_hand_landmarks:
            # Publish a string message indicating that a hand was detected
            self.publisher_.publish(String(data='Hand detected!'))
            
            # Log the detection in the ROS 2 system
            self.get_logger().info('Hand detected!')

# Main function to run the node
def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)
    
    # Create an instance of the HandDetector node
    node = HandDetector()
    
    # Keep the node spinning to process incoming data and callbacks
    rclpy.spin(node)
    
    # Destroy the node after spinning ends
    node.destroy_node()
    
    # Shut down the ROS 2 client library
    rclpy.shutdown()

# Run the main function if the script is executed directly
if __name__ == '__main__':
    main()
