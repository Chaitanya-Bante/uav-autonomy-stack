from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    gz_bridge = Node(
        package='gz_lidar_bridge',
        executable='lidar_bridge_node',
        name='gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    odometry_publisher = Node(
        package='px4_2d_slam',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    slam_config = PathJoinSubstitution([
        FindPackageShare('px4_2d_slam'),
        'config',
        'slam_toolbox.yaml'
    ])
    
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': True}]
    )
    
    rviz_config = PathJoinSubstitution([
        FindPackageShare('px4_2d_slam'),
        'config',
        'slam_2d.rviz'
    ])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gz_bridge,
        odometry_publisher,
        slam_toolbox,
        rviz,
    ])
