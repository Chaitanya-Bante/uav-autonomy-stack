#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.px4_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos_profile
        )
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        self.get_logger().info('Odometry publisher started (2D projection mode)')

    def position_callback(self, msg):
        # Publish odometry - PROJECT TO GROUND (z=0) for 2D SLAM
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Use XY position, but set Z to 0 (ground projection)
        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.position.z = 0.0  # Project to ground
        odom.pose.pose.orientation.w = 1.0
        
        # Only XY velocities for 2D SLAM
        odom.twist.twist.linear.x = msg.vx
        odom.twist.twist.linear.y = msg.vy
        odom.twist.twist.linear.z = 0.0  # Ignore vertical velocity
        
        self.odom_pub.publish(odom)
        
        # Publish TF: odom -> base_footprint (ground projection)
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0  # Always on ground
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish actual drone position: base_footprint -> base_link
        t2 = TransformStamped()
        t2.header.stamp = odom.header.stamp
        t2.header.frame_id = 'base_footprint'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = -msg.z  # Actual height above ground
        t2.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t2)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
