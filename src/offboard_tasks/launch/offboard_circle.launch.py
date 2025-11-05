from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard_tasks',
            executable='offboard_circle',
            output='screen'
        )
    ])
