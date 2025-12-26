import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')    

        self.turtle1_publisher_ = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.turtle1_timer = self.create_timer(0.5, self.send_turtle1_velocity)

        self.turtle1_velocity = Twist()
        self.turtle1_velocity.linear.x = 2.0
        self.turtle1_velocity.angular.z = 1.0

        self.spawn_client = self.create_client(Spawn,'spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /spawn service...')

        self.request = Spawn.Request()
        self.request.x = 5.0
        self.request.y = 5.0
        self.request.theta = 0.0

        self.request.name = 'turtle2'
        self.spawn_turtle()

    def spawn_turtle(self):
        self.future = self.spawn_client.call_async(self.request)
        rclpy.spin_until_future_complete(self,self.future)

        if self.future.result() is not None:
            self.get_logger().info(f'Successfully spawned {self.future.result().name}')
            self.turtle2_publisher_ = self.create_publisher(Twist,'/turtle2/cmd_vel',10)
            self.turtle2_timer = self.create_timer(0.5, self.send_turtle2_velocity)
            self.turtle2_velocity = Twist()
            self.turtle2_velocity.linear.x = 1.5
            self.turtle2_velocity.angular.z = -1.0
        else:
            self.get_logger().info('Failed to spawn turtle.')
        pass

    def send_turtle1_velocity(self):
        self.turtle1_publisher_.publish(self.turtle1_velocity)
        # self.get_logger().info('Turtle1 Publishing: linear.x: {:.2f},angular.z : {:.2f}'.format(self.velocity.linear.x,self.velocity.angular.z))


    def send_turtle2_velocity(self):
        self.turtle2_publisher_.publish(self.turtle2_velocity)
        # self.get_logger().info('Turtle2 Publishing: linear.x: {:.2f},angular.z : {:.2f}'.format(self.velocity.linear.x,self.velocity.angular.z))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()