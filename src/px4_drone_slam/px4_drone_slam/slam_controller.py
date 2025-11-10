#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class SlamController(Node):
    def __init__(self):
        super().__init__('slam_controller')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.px4_odom_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.px4_position_callback, qos_profile)
        
        self.odom_pub = self.create_publisher(Odometry, '/drone/odometry', 10)
        self.path_pub = self.create_publisher(Path, '/drone/path', 10)
        
        self.px4_position = None
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.message_count = 0
        
        self.odom_timer = self.create_timer(0.05, self.publish_odometry)
        self.tf_timer = self.create_timer(0.033, self.publish_transforms)
        
        # Debug timer
        self.debug_timer = self.create_timer(2.0, self.debug_status)
        
        self.get_logger().info('SLAM controller initialized')
        self.get_logger().info('Waiting for PX4 position data on /fmu/out/vehicle_local_position_v1')

    def debug_status(self):
        """Print debug status every 2 seconds."""
        if self.px4_position is None:
            self.get_logger().warn('No PX4 data received yet. Check:')
            self.get_logger().warn('  1. Is PX4 SITL running?')
            self.get_logger().warn('  2. Is MicroXRCEAgent running?')
            self.get_logger().warn('  3. Is ROS_DOMAIN_ID=0 set?')
        else:
            self.get_logger().info(f'Received {self.message_count} PX4 messages. Drone at ({self.px4_position.x:.2f}, {self.px4_position.y:.2f}, {-self.px4_position.z:.2f})')

    def px4_position_callback(self, msg):
        self.px4_position = msg
        self.message_count += 1
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = msg.x
        pose.pose.position.y = msg.y
        pose.pose.position.z = -msg.z
        pose.pose.orientation.w = 1.0
        
        self.path.poses.append(pose)
        if len(self.path.poses) > 1000:
            self.path.poses.pop(0)

    def publish_odometry(self):
        if self.px4_position is None:
            return
            
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.px4_position.x
        odom.pose.pose.position.y = self.px4_position.y
        odom.pose.pose.position.z = -self.px4_position.z
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = self.px4_position.vx
        odom.twist.twist.linear.y = self.px4_position.vy
        odom.twist.twist.linear.z = -self.px4_position.vz
        self.odom_pub.publish(odom)

    def publish_transforms(self):
        if self.px4_position is None:
            return
            
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = self.px4_position.x
        t2.transform.translation.y = self.px4_position.y
        t2.transform.translation.z = -self.px4_position.z
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)
        
        # Also publish camera link
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'base_link'
        t3.child_frame_id = 'camera_link'
        t3.transform.translation.x = 0.15
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.05
        t3.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t3)
        
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    controller = SlamController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
