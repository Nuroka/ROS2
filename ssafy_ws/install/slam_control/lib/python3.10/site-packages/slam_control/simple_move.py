import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()

    navigator = BasicNavigator()

    print("Checking Nav2 action status...")
    navigator.waitUntilNav2Active()
    print("Nav2 is ACTIVE")

    # Set Goal
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.w = 1.0

    print(f"Sending Goal: (x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y})...")

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print(f"Distance remaining: {feedback.distance_remaining:.2f} meters.")

            if feedback.distance_remaining < 0.1:
                print("Very close to target...")

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print("Success..")
    elif result == TaskResult.CANCELED:
        print("Canceled..")
    elif result == TaskResult.FAILED:
        print("Failed..")

    exit(0)

if __name__ == '__main__':
    main()