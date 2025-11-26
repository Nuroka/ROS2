import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveForwardNode(Node):
    def __init__(self):
        super().__init__('move_forward_node')

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()