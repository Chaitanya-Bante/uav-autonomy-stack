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
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', 
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback, qos_profile)

        # State variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0  # NED frame, negative is up
        self.home_position_set = False
        self.home_position = [0.0, 0.0, 0.0]
        
        # Square pattern waypoints (NED coordinates relative to home)
        self.square_size = 10.0  # meters
        self.waypoints = [
            [0.0, 0.0, self.takeoff_height],
            [self.square_size, 0.0, self.takeoff_height],
            [self.square_size, self.square_size, self.takeoff_height],
            [0.0, self.square_size, self.takeoff_height],
            [0.0, 0.0, self.takeoff_height],
        ]
        self.current_waypoint_index = 0
        self.waypoint_reached_threshold = 1.0  # meters
        self.hover_time = 0
        self.hover_duration = 50  # cycles to hover at each waypoint (1 second at 50Hz)

        # Flight state machine
        self.flight_state = "INIT"  # INIT -> OFFBOARD -> ARMED -> TAKEOFF -> MISSION -> LAND -> LANDED
        
        # Diagnostics
        self.connection_check_counter = 0
        self.last_log_time = 0
        
        # Create timer for control loop (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info('Square flight node initialized')
        self.get_logger().info(f'Mission: Fly square pattern of {self.square_size}m at {-self.takeoff_height}m altitude')
        self.get_logger().info('Waiting for connection to PX4...')

    def vehicle_local_position_callback(self, msg):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = msg
        
        # Set home position on first callback
        if not self.home_position_set:
            self.home_position = [msg.x, msg.y, msg.z]
            self.home_position_set = True
            self.get_logger().info(f'Home position set: {self.home_position}')

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

    def get_distance_to_waypoint(self):
        """Calculate distance to current waypoint."""
        if not self.home_position_set:
            return float('inf')
            
        current_wp = self.waypoints[self.current_waypoint_index]
        target_x = self.home_position[0] + current_wp[0]
        target_y = self.home_position[1] + current_wp[1]
        target_z = self.home_position[2] + current_wp[2]
        
        distance = np.sqrt(
            (self.vehicle_local_position.x - target_x)**2 +
            (self.vehicle_local_position.y - target_y)**2 +
            (self.vehicle_local_position.z - target_z)**2
        )
        return distance

    def timer_callback(self):
        """Main control loop - state machine."""
        
        # Connection diagnostics
        self.connection_check_counter += 1
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if not self.home_position_set:
            # Check connection every 2 seconds
            if self.connection_check_counter % 100 == 0:
                self.get_logger().warn(
                    'No data from PX4! Check if:\n'
                    '  1. PX4 SITL is running\n'
                    '  2. MicroXRCEAgent is running (MicroXRCEAgent udp4 -p 8888)\n'
                    '  3. PX4 console shows "INFO [uxrce_dds_client] connected to server"'
                )
            # Still publish to keep trying
            self.publish_offboard_control_mode()
            return
        
        # Always publish offboard control mode
        self.publish_offboard_control_mode()
        
        if self.flight_state == "INIT":
            # Send setpoints for 2 seconds before engaging offboard
            if self.home_position_set:
                target_x = self.home_position[0] + self.waypoints[0][0]
                target_y = self.home_position[1] + self.waypoints[0][1]
                target_z = self.home_position[2] + self.waypoints[0][2]
                self.publish_position_setpoint(target_x, target_y, target_z)
                
                self.offboard_setpoint_counter += 1
                
                # After 100 cycles (2 seconds), switch to offboard
                if self.offboard_setpoint_counter > 100:
                    self.engage_offboard_mode()
                    self.flight_state = "OFFBOARD"
                    self.offboard_setpoint_counter = 0
                    
        elif self.flight_state == "OFFBOARD":
            # Continue sending setpoints
            target_x = self.home_position[0] + self.waypoints[0][0]
            target_y = self.home_position[1] + self.waypoints[0][1]
            target_z = self.home_position[2] + self.waypoints[0][2]
            self.publish_position_setpoint(target_x, target_y, target_z)
            
            self.offboard_setpoint_counter += 1
            
            # After 50 more cycles (1 second), arm
            if self.offboard_setpoint_counter > 50:
                self.arm()
                self.flight_state = "ARMED"
                self.offboard_setpoint_counter = 0
                
        elif self.flight_state == "ARMED":
            # Continue sending setpoints and wait for takeoff
            target_x = self.home_position[0] + self.waypoints[0][0]
            target_y = self.home_position[1] + self.waypoints[0][1]
            target_z = self.home_position[2] + self.waypoints[0][2]
            self.publish_position_setpoint(target_x, target_y, target_z)
            
            # Check if we've reached takeoff altitude
            if self.vehicle_local_position.z < (self.home_position[2] + self.takeoff_height + 0.5):
                self.get_logger().info('Takeoff complete, starting mission')
                self.flight_state = "MISSION"
                
        elif self.flight_state == "MISSION":
            # Execute square pattern
            current_wp = self.waypoints[self.current_waypoint_index]
            target_x = self.home_position[0] + current_wp[0]
            target_y = self.home_position[1] + current_wp[1]
            target_z = self.home_position[2] + current_wp[2]
            self.publish_position_setpoint(target_x, target_y, target_z)
            
            distance = self.get_distance_to_waypoint()
            
            # Check if waypoint reached
            if distance < self.waypoint_reached_threshold:
                self.hover_time += 1
                
                # Hover at waypoint for specified duration
                if self.hover_time >= self.hover_duration:
                    self.get_logger().info(
                        f'Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} reached. '
                        f'Distance: {distance:.2f}m. Moving to next waypoint.'
                    )
                    self.current_waypoint_index += 1
                    self.hover_time = 0
                    
                    # Check if mission complete
                    if self.current_waypoint_index >= len(self.waypoints):
                        self.get_logger().info('Mission complete! Landing...')
                        self.flight_state = "LAND"
            else:
                # Reset hover counter if not at waypoint
                self.hover_time = 0
                
        elif self.flight_state == "LAND":
            # Command landing
            self.land()
            self.flight_state = "LANDED"
            
        elif self.flight_state == "LANDED":
            # Mission finished
            pass


def main(args=None):
    rclpy.init(args=args)
    square_flight_node = SquareFlightNode()
    
    try:
        rclpy.spin(square_flight_node)
    except KeyboardInterrupt:
        square_flight_node.get_logger().info('Node stopped by user')
    finally:
        square_flight_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()