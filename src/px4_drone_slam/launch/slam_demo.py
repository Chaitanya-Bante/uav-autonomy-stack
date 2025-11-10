from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    
    # SLAM controller (provides odometry and TF)
    slam_controller = Node(
        package='px4_drone_slam',
        executable='slam_controller',
        name='slam_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # RViz for visualization
    rviz_config = os.path.join(
        os.path.expanduser('~'), 'px4_ws', 'src', 
        'px4_drone_slam', 'config', 'slam_viz.rviz'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )
    
    return LaunchDescription([
        slam_controller,
        rviz,
    ])
