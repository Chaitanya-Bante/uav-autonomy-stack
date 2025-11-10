from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Start in localization mode'
    )
    
    # Camera bridge node
    camera_bridge = Node(
        package='px4_drone_slam',
        executable='camera_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # SLAM controller node
    slam_controller = Node(
        package='px4_drone_slam',
        executable='slam_controller',
        name='slam_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # RTABMap SLAM node
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan_cloud': False,
            'approx_sync': True,
            'queue_size': 10,
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/NeighborLinkRefining': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.01',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'Optimizer/Slam2D': 'false',
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'false',
            'Grid/FromDepth': 'false',
            'Vis/MinInliers': '12',
        }],
        remappings=[
            ('rgb/image', '/camera/rgb/image_raw'),
            ('rgb/camera_info', '/camera/rgb/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('odom', '/drone/odometry'),
        ],
        arguments=['--delete_db_on_start']
    )
    
    # RTABMap visualization
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'subscribe_depth': True,
            'subscribe_odom_info': True,
            'frame_id': 'base_link',
            'approx_sync': True,
        }],
        remappings=[
            ('rgb/image', '/camera/rgb/image_raw'),
            ('rgb/camera_info', '/camera/rgb/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('odom', '/drone/odometry'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        localization_arg,
        camera_bridge,
        slam_controller,
        rtabmap_slam,
        rtabmap_viz,
    ])
