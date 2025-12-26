import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

BATTERY_DRAIN_RATE = 0.5
BATTERY_THRESHOLD = 20.0    # 배터리가 20 이하로 떨어지면 복귀
CHARGINE_TIME = 5           # 5초 지나면 충전 완료됐다고 가정

# 순찰 루트 (A, B, C)
PATROL_ROUTE = [
    [1.5, 0.0, 0.0, 1.0],   # A
    [2.0, 1.0, 0.0, 1.0],   # B
    [0.0, 0.0, 0.0, 1.0]    # C
]

# 충전소 위치
CHARGINE_STATION = [0.0, 0.0, 0.0, 1.0]


def main():
    rclpy.init()
    navigator = BasicNavigator()

    navigator.waitUntilNav2Active()
    print("Nav2 is Ready!")

    current_battery = 100.0

    while True:
        print(f"\nStarting Patrol Loop. Current Battery: {current_battery:.1f}%")

        # 순찰 경로 이동
        for i, waypoint in enumerate(PATROL_ROUTE):

            # 배터리 확인
            if current_battery < BATTERY_THRESHOLD:
                print(f"Battery Low ({current_battery:.1f}%)! Returning to charger...")
                break
            
            print(f"\n>>> Heading to WayPoint {i+1}...")

            # set Goal (이동)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()

            goal_pose.pose.position.x = waypoint[0]
            goal_pose.pose.position.y = waypoint[1]
            goal_pose.pose.orientation.w = waypoint[3]

            navigator.goToPose(goal_pose)

            # 이동 중 배터리 감소
            while not navigator.isTaskComplete():
                current_battery -= BATTERY_DRAIN_RATE
                print(f"    >>> Moving... Battery: {current_battery:.1f}%", end='\r')
                time.sleep(0.1)

                if current_battery < BATTERY_THRESHOLD:
                    print("\nBattery Critical! Canceling current path!")
                    navigator.cancelTask()  # 이동 취소
                    break

            # 취소된 경우 바로 복귀
            if navigator.getResult() == TaskResult.CANCELED:
                break

            print(f"Waypoint {i+1} Reached.")
            time.sleep(1)

        # 충전 루틴
        if current_battery < BATTERY_THRESHOLD:
            print("\nMoving to charging station...")

            dock_pose = PoseStamped()
            dock_pose.header.frame_id = 'map'
            dock_pose.header.stamp = navigator.get_clock().now().to_msg()

            dock_pose.pose.position.x = CHARGINE_STATION[0]
            dock_pose.pose.position.y = CHARGINE_STATION[1]
            dock_pose.pose.orientation.w = CHARGINE_STATION[3]

            navigator.goToPose(dock_pose)

            while not navigator.isTaskComplete():
                pass

            print(f"Charging for {CHARGINE_TIME} seconds...")
            time.sleep(CHARGINE_TIME)

            current_battery = 100.0
            print("Battery full!")

    exit(0)


if __name__ == '__main__':
    main()