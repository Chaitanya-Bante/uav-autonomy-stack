from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_custom_nodes',
            executable='square_flight',
            name='square_flight_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]
        ),
    ])