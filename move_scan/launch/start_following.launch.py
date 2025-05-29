from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='move_scan',
            executable='wall_following',
            output='screen'),
    ])
