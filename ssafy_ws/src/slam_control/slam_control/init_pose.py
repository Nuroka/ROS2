import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

def main():
    # init ROS2
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp    = navigator.get_clock().now().to_msg()

    # 크게 고려할 것은 x, y값 (초기 위치)
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0   # 회전값
    initial_pose.pose.orientation.w = 1.0   # 회전값 

    print(f"Setting initial pose to (x={initial_pose.pose.position.x}, y={initial_pose.pose.position.y})...")
    # 초기 위치 버튼 
    navigator.setInitialPose(initial_pose)

    print("Waititing for Nav2 to activate...")
    navigator.waitUntilNav2Active()

    print("Nav2 is now Active and ready for commands.")
    exit(0)

if __name__ == '__main__':
    main()