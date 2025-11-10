#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from cv_bridge import CvBridge
import cv2

class CameraBridge(Node):
    """
    Bridge Gazebo camera topics to ROS2 standard topics for SLAM
    """
    
    def __init__(self):
        super().__init__('camera_bridge')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # TF broadcaster for camera frame
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers to Gazebo topics
        self.rgb_sub = self.create_subscription(
            Image,
            '/rgbd_camera/image',
            self.rgb_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/rgbd_camera/depth_image',
            self.depth_callback,
            10
        )
        
        self.points_sub = self.create_subscription(
            PointCloud2,
            '/rgbd_camera/points',
            self.points_callback,
            10
        )
        
        # Publishers to standard ROS2 topics for SLAM
        self.rgb_pub = self.create_publisher(
            Image,
            '/camera/rgb/image_raw',
            10
        )
        
        self.depth_pub = self.create_publisher(
            Image,
            '/camera/depth/image_raw',
            10
        )
        
        self.points_pub = self.create_publisher(
            PointCloud2,
            '/camera/depth/points',
            10
        )
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/rgb/camera_info',
            10
        )
        
        self.depth_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/depth/camera_info',
            10
        )
        
        # Camera info (matching the SDF model)
        self.camera_info = self.create_camera_info()
        
        # Timer to publish camera info
        self.timer = self.create_timer(0.1, self.publish_camera_info)
        
        # Timer to publish TF
        self.tf_timer = self.create_timer(0.033, self.publish_camera_tf)
        
        self.get_logger().info('Camera bridge initialized')
        self.get_logger().info('Subscribing to Gazebo topics: /rgbd_camera/*')
        self.get_logger().info('Publishing to ROS2 topics: /camera/*')

    def create_camera_info(self):
        """Create camera info message."""
        info = CameraInfo()
        info.header.frame_id = 'camera_link'
        info.width = 640
        info.height = 480
        
        # Camera intrinsics (from SDF model)
        fx = 525.0
        fy = 525.0
        cx = 320.0
        cy = 240.0
        
        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = 'plumb_bob'
        
        return info

    def rgb_callback(self, msg):
        """Forward RGB image."""
        msg.header.frame_id = 'camera_link'
        self.rgb_pub.publish(msg)

    def depth_callback(self, msg):
        """Forward depth image."""
        msg.header.frame_id = 'camera_link'
        self.depth_pub.publish(msg)

    def points_callback(self, msg):
        """Forward point cloud."""
        msg.header.frame_id = 'camera_link'
        self.points_pub.publish(msg)

    def publish_camera_info(self):
        """Publish camera info messages."""
        stamp = self.get_clock().now().to_msg()
        
        self.camera_info.header.stamp = stamp
        self.camera_info_pub.publish(self.camera_info)
        
        depth_info = self.camera_info
        depth_info.header.frame_id = 'camera_link'
        self.depth_info_pub.publish(depth_info)

    def publish_camera_tf(self):
        """Publish camera transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        
        # Camera position (matching SDF model)
        t.transform.translation.x = 0.15
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05
        
        # Camera orientation (pointing forward)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    bridge = CameraBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info('Shutting down camera bridge')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()