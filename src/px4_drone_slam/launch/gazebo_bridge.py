from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Bridge Gazebo topics to ROS2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            # Bridge camera image
            '/camera_imu@sensor_msgs/msg/Image@gz.msgs.Image',
            # Bridge camera info (if available)
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/camera_imu', '/camera/rgb/image_raw'),
        ]
    )
    
    # Camera info publisher (since Gazebo might not provide it)
    camera_info_pub = Node(
        package='px4_drone_slam',
        executable='camera_bridge',
        name='camera_info_publisher',
        output='screen',
    )
    
    # SLAM controller
    slam_controller = Node(
        package='px4_drone_slam',
        executable='slam_controller',
        name='slam_controller',
        output='screen',
    )
    
    return LaunchDescription([
        gz_bridge,
        camera_info_pub,
        slam_controller,
    ])
