from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='waypoints.yaml',
        description='Name of the waypoint file in config directory'
    )
    
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height',
        default_value='5.0',
        description='Takeoff height in meters'
    )
    
    # Get the path to the config directory
    config_file_path = PathJoinSubstitution([
        FindPackageShare('px4_custom_nodes'),
        'config',
        LaunchConfiguration('waypoint_file')
    ])
    
    # Waypoint navigator node
    waypoint_navigator_node = Node(
        package='px4_custom_nodes',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'waypoint_file': config_file_path,
            'takeoff_height': LaunchConfiguration('takeoff_height'),
            'waypoint_threshold': 1.0,
            'hover_duration': 2.0,
            'use_sim_time': True,
        }]
    )
    
    return LaunchDescription([
        waypoint_file_arg,
        takeoff_height_arg,
        waypoint_navigator_node,
    ])
