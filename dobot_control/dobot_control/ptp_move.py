from dobot_msgs.action import PointToPoint
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

class PTP_MOVE(Node):
    def __init__(self):
        super().__init__('dobot_PTP_client')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        self.goal_index = 0

        # 이동할 목표 리스트
        self.targets = [
            [150.0, 50.0, 100.0, 0.0],
            [200.0, -50.0, 100.0, 0.0],
            [150.0, 50.0, 100.0, 0.0],
            [200.0, -50.0, 100.0, 0.0]
        ]

    # --------------------
    # Action callbacks
    # --------------------
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.goal_handle = goal_handle

        # 결과 기다리기
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Goal result received.')

        # 다음 Goal로 넘어가기
        self.goal_index += 1

        if self.goal_index < len(self.targets):
            self.send_goal(self.targets[self.goal_index], mode=1)
        else:
            self.get_logger().info("All goals completed.")
            rclpy.shutdown()

    def feedback_callback(self, feedback):
        self.get_logger().info(
            f"Feedback pose: {feedback.feedback.current_pose}"
        )

    # --------------------
    # Send goal
    # --------------------
    def send_goal(self, target, mode):
        self.get_logger().info(f"Sending goal #{self.goal_index+1}: {target}")

        self._action_client.wait_for_server()
        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = target
        goal_msg.motion_type = mode

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    node = PTP_MOVE()

    # 첫 번째 goal 실행
    node.send_goal(node.targets[0], mode=1)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
