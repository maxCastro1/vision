import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MoveNode(Node):
    def __init__(self):
        super().__init__('move_node')
        # Subscribe to the cmd_hand topic
        self.subscription = self.create_subscription(String, 'cmd_hand', self.listener_callback, 10)
        # Publisher for the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        twist = Twist()

        # Interpret the hand gesture and set the movement
        if msg.data == 'move_forward':
            twist.linear.x = 1.0  # Move forward
        elif msg.data == 'move_backward':
            twist.linear.x = -1.0  # Move backward
        elif msg.data == 'turn_left':
            twist.angular.z = 1.0  # Turn left
        elif msg.data == 'turn_right':
            twist.angular.z = -1.0  # Turn right
        else:
            self.get_logger().info('Unknown command. Stopping the robot.')

        # Publish the movement command
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
