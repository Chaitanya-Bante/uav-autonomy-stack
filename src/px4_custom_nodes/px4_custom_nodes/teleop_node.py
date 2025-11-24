#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import String
import sys

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Subscribers
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.position_callback, qos_profile)
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.status_callback, qos_profile)
        
        # Subscribe to keyboard commands (from another node)
        self.key_sub = self.create_subscription(
            String, '/teleop/cmd', self.key_callback, 10)
        
        # State
        self.current_position = [0.0, 0.0, 0.0]
        self.target_position = [0.0, 0.0, -5.0]
        self.vehicle_status = None
        self.armed = False
        self.offboard_counter = 0
        
        # Movement
        self.step_xy = 1.0
        self.step_z = 0.5
        
        # Timer
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('=== PX4 Teleop Ready ===')
        self.get_logger().info('Waiting for commands on /teleop/cmd')
        self.get_logger().info('Run keyboard node: ros2 run px4_custom_nodes teleop_keyboard')

    def position_callback(self, msg):
        self.current_position = [msg.x, msg.y, msg.z]
        if not self.armed:
            self.target_position = [msg.x, msg.y, -5.0]

    def status_callback(self, msg):
        self.vehicle_status = msg
        self.armed = (msg.arming_state == 2)

    def key_callback(self, msg):
        key = msg.data
        
        if key == 'w':
            self.target_position[0] += self.step_xy
            self.get_logger().info(f'Forward -> {self.target_position[:2]}')
        elif key == 's':
            self.target_position[0] -= self.step_xy
            self.get_logger().info(f'Back -> {self.target_position[:2]}')
        elif key == 'a':
            self.target_position[1] -= self.step_xy
            self.get_logger().info(f'Left -> {self.target_position[:2]}')
        elif key == 'd':
            self.target_position[1] += self.step_xy
            self.get_logger().info(f'Right -> {self.target_position[:2]}')
        elif key == 'q':
            self.target_position[2] -= self.step_z
            self.get_logger().info(f'Up -> {-self.target_position[2]:.1f}m')
        elif key == 'e':
            self.target_position[2] += self.step_z
            self.get_logger().info(f'Down -> {-self.target_position[2]:.1f}m')
        elif key == 'space':
            self.target_position = list(self.current_position)
            self.get_logger().info('Hover')
        elif key == 't':
            self.arm()
            self.get_logger().info('Takeoff')
        elif key == 'l':
            self.land()
            self.get_logger().info('Landing...')
        elif key == 'k':
            self.disarm()
            self.get_logger().info('DISARM')

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def control_loop(self):
        self.publish_offboard_control_mode()
        
        if self.offboard_counter < 10:
            self.offboard_counter += 1
            if self.offboard_counter == 10:
                self.engage_offboard_mode()
        
        self.publish_position_setpoint(
            self.target_position[0],
            self.target_position[1],
            self.target_position[2]
        )


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
