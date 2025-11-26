from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_file = '/home/ssafy/ssafy_ws/src/intergrate_prac/rviz'

    return LaunchDescription([
        #Rviz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments= ['-d', rviz_config_file]
        ),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
            remappings= [
                ('/image', '/camera/camera/color/image_raw')
            ]
        )
    ])