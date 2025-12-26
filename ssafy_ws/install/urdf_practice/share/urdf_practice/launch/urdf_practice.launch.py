from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    #rviz_config_file = ''
    #xacro_file = '/home/ssafy/ssafy_ws/src/urdf_practice/urdf/two_link_robot.xacro'
    #doc = xacro.process_file(xacro_file)
    #robot_description = doc.toxml()

    urdf_file_1 = '/home/ssafy/ssafy_ws/src/urdf_practice/urdf/robot3.urdf'
    
    with open(urdf_file_1, 'r') as infp:
        robot_description_1 = infp.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="robot3",
            parameters=[{'robot_description': robot_description_1}]
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            namespace="robot3"
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            #arguments=['-d',urdf_file],
        )
    ])
