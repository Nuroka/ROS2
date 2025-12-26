import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # 로봇이 이동할 수 있는지 Check
    print("Checking Nav2 active status")
    navigator.waitUntilNav2Active()
    print("Nav2 is ACTIVE.")

    # set Goal (이동)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    # 시작 위치와 타겟 위치가 같으면 안움직임 
    # 때문에 x값과 y값 변경해줘야함 
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.w = 1.0

    # Goal 전달 완료 메시지 
    print(f"Sending Goal: (x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y})...")

    # 목표점 실제 이동 
    navigator.goToPose(goal_pose)

    # 현재 상태 monitoring
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print(f"Distance remaining: {feedback.distance_remaining: .2f} meters.")

            if feedback.distance_remaining < 0.1:
                print("Very close to target...")

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print("Success.")
    elif result == TaskResult.CANCELED:
        print("Canceld..")
    elif result == TaskResult.FAILED:
        print("Fail..")

    exit(0)

if __name__ == '__main__':
    main()