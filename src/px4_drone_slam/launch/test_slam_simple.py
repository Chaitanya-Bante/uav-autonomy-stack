from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Just the SLAM controller (no camera bridge for now)
    slam_controller = Node(
        package='px4_drone_slam',
        executable='slam_controller',
        name='slam_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        slam_controller,
    ])
