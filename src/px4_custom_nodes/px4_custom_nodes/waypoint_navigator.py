#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import numpy as np
import yaml
import os

class WaypointNavigator(Node):
    """
    Generic waypoint navigation node that reads waypoints from a YAML file
    """
    
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Declare parameters
        self.declare_parameter('waypoint_file', 'waypoints.yaml')
        self.declare_parameter('takeoff_height', 5.0)
        self.declare_parameter('waypoint_threshold', 1.0)
        self.declare_parameter('hover_duration', 2.0)  # seconds
        
        # Get parameters
        waypoint_file = self.get_parameter('waypoint_file').value
        self.takeoff_height = -self.get_parameter('takeoff_height').value  # NED
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        hover_duration_sec = self.get_parameter('hover_duration').value
        self.hover_duration = int(hover_duration_sec * 50)  # Convert to cycles at 50Hz
        
        # Configure QoS
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
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.home_position_set = False
        self.home_position = [0.0, 0.0, 0.0]
        
        # Load waypoints
        self.waypoints = self.load_waypoints(waypoint_file)
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded! Exiting...')
            raise RuntimeError('No waypoints available')
            
        self.current_waypoint_index = 0
        self.hover_time = 0
        
        # State machine
        self.flight_state = "INIT"
        self.offboard_setpoint_counter = 0
        
        # Timer
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info('Waypoint navigator initialized')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        self.get_logger().info('Waiting for PX4 connection...')

    def load_waypoints(self, filename):
        """Load waypoints from YAML file."""
        try:
            # Try loading from package share directory
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('px4_custom_nodes')
            filepath = os.path.join(pkg_dir, 'config', filename)
            
            if not os.path.exists(filepath):
                # Try current directory
                filepath = filename
                
            if not os.path.exists(filepath):
                self.get_logger().error(f'Waypoint file not found: {filename}')
                return []
                
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
                
            waypoints = []
            for wp in data.get('waypoints', []):
                # Convert to NED coordinates
                waypoints.append([
                    float(wp['x']),
                    float(wp['y']),
                    -float(wp['z'])  # Convert to NED (negative is up)
                ])
                
            self.get_logger().info(f'Loaded waypoints from: {filepath}')
            return waypoints
            
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            # Return default takeoff waypoint
            return [[0.0, 0.0, self.takeoff_height]]

    def vehicle_local_position_callback(self, msg):
        """Callback for vehicle position."""
        self.vehicle_local_position = msg
        if not self.home_position_set:
            self.home_position = [msg.x, msg.y, msg.z]
            self.home_position_set = True
            self.get_logger().info(f'Home position: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}]')

    def vehicle_status_callback(self, msg):
        """Callback for vehicle status."""
        self.vehicle_status = msg

    def arm(self):
        """Arm the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arming...')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Engaging offboard mode...")

    def land(self):
        """Land the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Landing...")

    def publish_offboard_control_mode(self):
        """Publish offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Publish trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Publish vehicle command."""
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
        self.vehicle_command_publisher.publish(msg)

    def get_distance_to_waypoint(self):
        """Calculate distance to current waypoint."""
        if not self.home_position_set:
            return float('inf')
            
        wp = self.waypoints[self.current_waypoint_index]
        target_x = self.home_position[0] + wp[0]
        target_y = self.home_position[1] + wp[1]
        target_z = self.home_position[2] + wp[2]
        
        return np.sqrt(
            (self.vehicle_local_position.x - target_x)**2 +
            (self.vehicle_local_position.y - target_y)**2 +
            (self.vehicle_local_position.z - target_z)**2
        )

    def timer_callback(self):
        """Main control loop."""
        
        if not self.home_position_set:
            self.publish_offboard_control_mode()
            return
        
        self.publish_offboard_control_mode()
        
        if self.flight_state == "INIT":
            wp = self.waypoints[0]
            target_x = self.home_position[0] + wp[0]
            target_y = self.home_position[1] + wp[1]
            target_z = self.home_position[2] + wp[2]
            self.publish_position_setpoint(target_x, target_y, target_z)
            
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter > 100:
                self.engage_offboard_mode()
                self.flight_state = "OFFBOARD"
                self.offboard_setpoint_counter = 0
                
        elif self.flight_state == "OFFBOARD":
            wp = self.waypoints[0]
            target_x = self.home_position[0] + wp[0]
            target_y = self.home_position[1] + wp[1]
            target_z = self.home_position[2] + wp[2]
            self.publish_position_setpoint(target_x, target_y, target_z)
            
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter > 50:
                self.arm()
                self.flight_state = "ARMED"
                self.offboard_setpoint_counter = 0
                
        elif self.flight_state == "ARMED":
            wp = self.waypoints[0]
            target_x = self.home_position[0] + wp[0]
            target_y = self.home_position[1] + wp[1]
            target_z = self.home_position[2] + wp[2]
            self.publish_position_setpoint(target_x, target_y, target_z)
            
            if self.vehicle_local_position.z < (self.home_position[2] + self.takeoff_height + 0.5):
                self.get_logger().info('Takeoff complete! Starting waypoint mission...')
                self.flight_state = "MISSION"
                
        elif self.flight_state == "MISSION":
            wp = self.waypoints[self.current_waypoint_index]
            target_x = self.home_position[0] + wp[0]
            target_y = self.home_position[1] + wp[1]
            target_z = self.home_position[2] + wp[2]
            self.publish_position_setpoint(target_x, target_y, target_z)
            
            distance = self.get_distance_to_waypoint()
            
            if distance < self.waypoint_threshold:
                self.hover_time += 1
                if self.hover_time >= self.hover_duration:
                    self.get_logger().info(
                        f'Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} complete '
                        f'(distance: {distance:.2f}m)'
                    )
                    self.current_waypoint_index += 1
                    self.hover_time = 0
                    
                    if self.current_waypoint_index >= len(self.waypoints):
                        self.get_logger().info('Mission complete!')
                        self.flight_state = "LAND"
            else:
                self.hover_time = 0
                
        elif self.flight_state == "LAND":
            self.land()
            self.flight_state = "LANDED"


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Stopped by user')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()