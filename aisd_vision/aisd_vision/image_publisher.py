# Import necessary modules from ROS 2 Python library
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


# Define a class that inherits from the Node class in ROS 2
class ImagePublisher(Node):
    def __init__(self):
        # Initialize the node with a name 'image_publisher'
        super().__init__('image_publisher')
        
        # Create a publisher that publishes messages of type 'Image' to the topic 'video_frames'
        self.publisher_ = self.create_publisher(
            Image, 'video_frames', 10
        )
        
        # Set the timer to call the timer_callback function periodically (every 0.1 seconds)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Open a connection to the default camera (usually camera 0)
        self.cap = cv2.VideoCapture(0)  # Use your camera
        
        # Create a CvBridge instance for converting between OpenCV and ROS Image messages
        self.br = CvBridge()

    def timer_callback(self):
        # Read a frame from the camera
        ret, frame = self.cap.read()
        
        # If the frame was successfully read
        if ret:
            # Convert the OpenCV frame to a ROS Image message and publish it
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame, 'bgr8'))
            
            # Log a message indicating a frame has been published
            self.get_logger().info('Publishing video frame')


# Define the main function to initialize and run the node
def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create an instance of the ImagePublisher node
    node = ImagePublisher()
    
    # Print a message to indicate the node is running
    print("Image Publisher Node Running")
    
    # Spin the node to keep it running and processing callbacks
    rclpy.spin(node)
    
    # Destroy the node when done to clean up resources
    node.destroy_node()
    
    # Shut down the rclpy library
    rclpy.shutdown()


# Run the main function if the script is executed directly
if __name__ == '__main__':
    main()
