#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition

class TestConnection(Node):
    def __init__(self):
        super().__init__('test_connection')
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.callback,
            10)
        self.get_logger().info('Waiting for PX4 data...')
        
    def callback(self, msg):
        self.get_logger().info(f'✓ Receiving data! Position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')

def main():
    rclpy.init()
    node = TestConnection()
    rclpy.spin(node)

if __name__ == '__main__':
    main()