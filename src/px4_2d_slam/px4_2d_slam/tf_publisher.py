#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish static transform: base_footprint -> base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.static_broadcaster.sendTransform(t)
        
        self.get_logger().info('Publishing static TF: base_footprint -> base_link')


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
