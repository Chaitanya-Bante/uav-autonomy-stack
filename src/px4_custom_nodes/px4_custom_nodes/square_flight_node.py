#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import numpy as np

class SquareFlightNode(Node):
    """
    Custom node that commands a PX4 drone to fly in a square pattern
    """
    
    def __init__(self):
        super().__init__('square_flight_node')
        
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', 
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)

        # State variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0  # NED frame, negative is up
        
        # Square pattern waypoints (NED coordinates)
        self.square_size = 10.0  # meters
        self.waypoints = [
            [0.0, 0.0, self.takeoff_height],
            [self.square_size, 0.0, self.takeoff_height],
            [self.square_size, self.square_size, self.takeoff_height],
            [0.0, self.square_size, self.takeoff_height],
            [0.0, 0.0, self.takeoff_height],
        ]
        self.current_waypoint_index = 0
        self.waypoint_reached_threshold = 0.5  # meters

        # Create timer for control loop (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info('Square flight node initialized')

    def vehicle_local_position_callback(self, msg):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = msg

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_mode(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0  # Face north
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def is_waypoint_reached(self):
        """Check if current waypoint is reached."""
        current_wp = self.waypoints[self.current_waypoint_index]
        distance = np.sqrt(
            (self.vehicle_local_position.x - current_wp[0])**2 +
            (self.vehicle_local_position.y - current_wp[1])**2 +
            (self.vehicle_local_position.z - current_wp[2])**2
        )
        return distance < self.waypoint_reached_threshold

    def timer_callback(self):
        """Main control loop."""
        self.publish_offboard_control_mode()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            # Send initial setpoints before switching to offboard
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
        else:
            # Follow square pattern
            current_wp = self.waypoints[self.current_waypoint_index]
            self.publish_position_setpoint(current_wp[0], current_wp[1], current_wp[2])
            
            # Check if waypoint reached and move to next
            if self.is_waypoint_reached():
                self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached')
                self.current_waypoint_index += 1
                
                # If completed square, land
                if self.current_waypoint_index >= len(self.waypoints):
                    self.land()
                    self.get_logger().info('Square pattern completed, landing...')

        self.offboard_setpoint_counter += 1


def main(args=None):
    rclpy.init(args=args)
    square_flight_node = SquareFlightNode()
    rclpy.spin(square_flight_node)
    square_flight_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()