import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')    

        self.publisher_ = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.timer = self.create_timer(0.5, self.send_velocity)

        self.velocity = Twist()
        self.velocity.linear.x = 2.0
        self.velocity.angular.z = 1.0

    def send_velocity(self):
        self.publisher_.publish(self.velocity)
        # self.get_logger().info('Publishing: linear.x: {:.2f},angular.z : {:.2f}'.format(self.velocity.linear.x,self.velocity.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()