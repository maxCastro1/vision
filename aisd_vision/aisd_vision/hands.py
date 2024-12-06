# Import necessary modules
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Node class for ROS 2
from sensor_msgs.msg import Image  # ROS message type for image data
from cv_bridge import CvBridge  # Bridge between ROS Image and OpenCV image
import cv2  # OpenCV for image processing
import mediapipe as mp  # Mediapipe for hand detection
from aisd_msgs.msg import Hand  # Custom message type for hand position

# Initialize Mediapipe Hands solution
mp_hands = mp.solutions.hands

# Define the HandDetector node class
class HandDetector(Node):
    def __init__(self):
        # Initialize the node with the name 'hands'
        super().__init__('hands')
        
        # Create a subscription to the 'video_frames' topic (Image type messages)
        # Calls 'listener_callback' whenever a new message is received
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10
        )
        
        # Publisher for the 'cmd_hand' topic (Hand type messages)
        self.hand_publisher = self.create_publisher(Hand, 'cmd_hand', 10)
        
        # Bridge for converting ROS Image messages to OpenCV images
        self.br = CvBridge()

    def listener_callback(self, msg):
       
        # Convert ROS Image message to OpenCV image
        image = self.br.imgmsg_to_cv2(msg)

        # Define constants for Mediapipe hand landmarks
        PINKY_FINGER_TIP = 20
        INDEX_FINGER_TIP = 8

        # Analyze the image for hand landmarks
        with mp_hands.Hands(
            model_complexity=0,  # Simplified model complexity
            min_detection_confidence=0.5,  # Minimum confidence for detection
            min_tracking_confidence=0.5  # Minimum confidence for tracking
        ) as myhands:

            # Optimize performance by marking the image as not writeable
            image.flags.writeable = False
            # Convert BGR image to RGB (required by Mediapipe)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # Process the image to detect hands
            results = myhands.process(image)

            if results.multi_hand_landmarks:
                # If hand landmarks are detected, extract positions
                msg = Hand()
                msg.xpinky = results.multi_hand_landmarks[0].landmark[PINKY_FINGER_TIP].x
                msg.xindex = results.multi_hand_landmarks[0].landmark[INDEX_FINGER_TIP].x
                
                # Check if there are any subscribers to the 'cmd_hand' topic
                if self.hand_publisher.get_subscription_count() > 0:
                    # Publish the hand position as a Hand message
                    self.hand_publisher.publish(msg)
                    self.get_logger().info(f'Published hand positions: xpinky={msg.xpinky}, xindex={msg.xindex}')
                else:
                    self.get_logger().info('Waiting for a subscriber.')

# Main function to initialize and run the node
def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the HandDetector node
    node = HandDetector()
    
    # Spin the node to keep it alive and processing callbacks
    rclpy.spin(node)
    
    # Cleanup: destroy the node and shut down the client library
    node.destroy_node()
    rclpy.shutdown()

# Entry point: run the main function if the script is executed directly
if __name__ == '__main__':
    main()
