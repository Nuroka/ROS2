import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveForwardNode(Node):
    def __init__(self):
        super().__init__('move_forward_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Waiting for subscriber to connect ...')
        while self.publisher_.get_subscription_count() == 0:
            time.sleep(0.1)
        
        self.get_logger().info('Subscriber connected! Moving forward for 2 sec')
        self.move_forward_for_two_seconds()

    def move_forward_for_two_seconds(self):
        msg = Twist()
        msg.linear.x = 0.2 
        msg.angular.z = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'publishing: linear.x = {msg.linear.x}')

        time.sleep(2.0)

        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'publishing: linear.x = {msg.linear.x}(stop)')

        time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardNode()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
