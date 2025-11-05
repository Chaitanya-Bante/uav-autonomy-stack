import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class OffboardCircle(Node):
    def __init__(self):
        super().__init__('offboard_circle')
        qos = 10
        self.ctrl_pub = self.create_publisher(OffboardControlMode, '/fmu/offboard_control_mode/in', qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/trajectory_setpoint/in', qos)
        self.cmd_pub  = self.create_publisher(VehicleCommand, '/fmu/vehicle_command/in', qos)

        self.t = 0.0
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

        # start streaming before switching to offboard
        self.stream_warmup = 40
        self.warmup_count = 0
        self.offboard_enabled = False
        self.armed = False

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def tick(self):
        # 1) publish control mode
        o = OffboardControlMode()
        o.position = True; o.velocity = False; o.acceleration = False; o.attitude = False; o.body_rate = False
        self.ctrl_pub.publish(o)

        # 2) publish a simple XY circle at z = -2 (NED in PX4; ROS is ENU—PX4 expects down positive)
        import math
        radius = 1.5
        self.t += 0.05
        x = radius * math.cos(self.t * 0.3)
        y = radius * math.sin(self.t * 0.3)
        z = -2.0  # down is positive in PX4 NED
        yaw = 0.0

        ts = TrajectorySetpoint()
        ts.position = [x, y, z]
        ts.yaw = yaw
        self.traj_pub.publish(ts)

        # 3) warmup stream, then switch to offboard, then arm
        if self.warmup_count < self.stream_warmup:
            self.warmup_count += 1
            return

        if not self.offboard_enabled:
            # VEHICLE_CMD_DO_SET_MODE (custom) is 176, but PX4 uses NAVIGATION mode setting via
            # VEHICLE_CMD_DO_SET_MODE + base mode/custom mode. Simpler: use dedicated command for Offboard = 92
            self.send_vehicle_command(command=92)  # VEHICLE_CMD_DO_SET_MODE: OFFBOARD
            self.offboard_enabled = True
            self.get_logger().info('Requested OFFBOARD')

        if self.offboard_enabled and not self.armed:
            self.send_vehicle_command(command=400, param1=1.0)  # VEHICLE_CMD_COMPONENT_ARM_DISARM, arm
            self.armed = True
            self.get_logger().info('Requested ARM')

def main():
    rclpy.init()
    node = OffboardCircle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
