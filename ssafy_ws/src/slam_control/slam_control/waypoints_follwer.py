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

    while True:
        print("\n" + "=" * 40)
        print(f"Available Locations: {list(WAYPOINTS.keys())}")
        print("Enter route (e.g: 'A B C' or 'q'): ", end="")

        user_input = input().strip().upper()

        # Quit
        if user_input == 'Q':
            print("Exiting...")
            break

        # Parse route list
        route_keys = user_input.split()
        valid_route = []

        for key in route_keys:
            if key in WAYPOINTS:
                valid_route.append(key)
            else:
                print(f"Ignoring unknown location '{key}'")

        if not valid_route:
            print("No valid waypoint included. Try again.")
            continue

        print(f"Starting Route: {' -> '.join(valid_route)}")

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
