import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

# Define location [A, B, C] ex.kitchen, room, livingroom
WAYPOINTS={
    "A": [1.5, 0.0, 0.0, 1.0],
    "B": [2.0, 1.0, 0.0, 1.0],
    "C": [0.0, 0.0, 0.0, 1.0]
}

def main():
    rclpy.init()
    navigator = BasicNavigator()

    navigator.waitUntilNav2Active()
    print("Nav2 is Ready!")

    while True:
        print("\n" + "="*40)
        print(f"Available Locations: {list(WAYPOINTS.keys())}")
        print("Enter location key (or 'q' to quit): ", end="")

        user_input = input()

        if user_input == 'q':
            print("Exiting...")
            break

        if user_input in WAYPOINTS:
            target = WAYPOINTS[user_input]
            target_x = target[0]
            target_y = target[1]
            target_w = target[3]

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = target_x
            goal_pose.pose.position.y = target_y
            goal_pose.pose.orientation.w = target_w

            print(f"Moving to '{user_input}' (x={target_x}, y={target_y})...")
            navigator.goToPose(goal_pose)

            while not navigator.isTaskComplete():
                pass

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print("Goal reached!")
            else:
                print("Goal failed or canceled!")
        else:
            print("Unknown location! Please try again.")

    exit(0)

if __name__ == '__main__':
    main()