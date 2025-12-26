import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

# Define location [A, B, C, kitchen, room, livingroom]
WAYPOINTS = {
    "A": [1.5, 0.0, 0.0, 1.0],
    "B": [2.0, 1.0, 0.0, 1.0],
    "C": [0.0, 0.0, 0.0, 1.0]
}

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Nav2 ready check
    navigator.waitUntilNav2Active()
    print("Nav2 is Ready!")

    valid_route = ["A", "B", "C"]
    
    while True:
        print("\n" + "=" * 40)
        print(f"Available Locations: {list(WAYPOINTS.keys())}")


        # Move through each waypoint
        for key in valid_route:
            target = WAYPOINTS[key]
            print(f"\n>>> Going to {key}...")

            # set Goal (이동)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()

            goal_pose.pose.position.x = target[0]
            goal_pose.pose.position.y = target[1]
            goal_pose.pose.orientation.w = target[3]

            navigator.goToPose(goal_pose)

            # Wait until arrival
            while not navigator.isTaskComplete():
                pass

            # Arrival result
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f"Arrived to {key}!")
            else:
                print(f"Failed to reach {key}. Stopping route.")
                break

    exit(0)


if __name__ == '__main__':
    main()
