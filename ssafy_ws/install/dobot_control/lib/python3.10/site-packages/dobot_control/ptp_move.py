from dobot_msgs.action import PointToPoint

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

class PTP_MOVE(Node):
    def __init__(self):
        super().__init__('dobot_PTP_client')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        # dobot에서 로봇 제어할 땐 action
        # 목표에 도달했는지 여부, 중간 피드백 결과를 알 수 있어서 로봇 제어할 땐 action 사용
        # 명령 제어 방법: topic service action


    # 아래는 action을 사용하기 위한 구조 ()
    def cancel_done(self, future):
        cancel_response = future.result()
        # 중간에 취소 명령이 들어왔을 때 처리 
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')   
        else:
            self.get_logger().info('Goal failed to cancel')
            rclpy.shutdown()
        

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.goal_handle = goal_handle

        self.get_logger().info('Goal accepted :)')
            
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()


    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.current_pose))


    def timer_callback(self):
        self.get_logger().info('Canceling goal')

        future = self._goal_handle.cancel_goal_async()   
        future.add_done_callback(self.cancel_done)

        self._timer.cancel()


    # target: 내가 가고 싶은 위치
    # mode: 모션 번호 (1~4)
    def send_goal(self, target, mode):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = target
        goal_msg.motion_type = mode
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback   # ← 오타 수정
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

def main(args=None):
    rclpy.init(args=args)
    action_client = PTP_MOVE()
    action_client.send_goal(target=[150.0, 50.0, 100.0, 0.0], mode = 1)
    # 로봇 이동하는 시간 대기
    time.sleep(2)
    
    action_client.send_goal(target=[200.0, -50.0, 100.0, 0.0], mode = 1)
    # 로봇 이동하는 시간 대기
    time.sleep(2)

    action_client.send_goal(target=[150.0, 50.0, 100.0, 0.0], mode = 1)
    # 로봇 이동하는 시간 대기
    time.sleep(2)

    action_client.send_goal(target=[200.0, -50.0, 100.0, 0.0], mode = 1)
    # 로봇 이동하는 시간 대기
    time.sleep(2)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
