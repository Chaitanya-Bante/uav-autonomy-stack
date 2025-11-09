# PX4 ROS2 Custom Workspace

Custom ROS2 nodes and autonomous navigation implementations for PX4 drones.

## Setup

### Prerequisites
- Ubuntu 22.04
- ROS2 Humble
- PX4 Autopilot v1.14+
- Gazebo Garden

### Installation

1. Clone this repository:
```bash
git clone https://github.com/YOUR_USERNAME/px4_ws.git
cd px4_ws
```

2. Build the workspace:
```bash
source setup_px4.bash
colcon build
```

### Usage

#### Running the square flight demo:
```bash
# Terminal 1: Start MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888

# Terminal 2: Start PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 3: Run the node
source ~/px4_ws/setup_px4.bash
ros2 run px4_custom_nodes square_flight
```

Or use the launch file:
```bash
ros2 launch px4_custom_nodes square_flight_launch.py
```

## Packages

### px4_custom_nodes
Custom control algorithms and flight patterns:
- `square_flight`: Autonomous square pattern flight
- `test_connection`: Diagnostic tool for PX4 connection

### px4_drone_slam (Coming Soon)
SLAM implementations for autonomous navigation

## Project Status

- [x] Basic PX4-ROS2 setup
- [x] Square flight pattern
- [ ] Waypoint navigation
- [ ] Obstacle avoidance
- [ ] Visual SLAM integration
- [ ] Path planning

## Notes

**Important**: Always set `ROS_DOMAIN_ID=0` before running nodes:
```bash
export ROS_DOMAIN_ID=0
```

Topic names use `_v1` suffix (e.g., `/fmu/out/vehicle_local_position_v1`)

## License
Apache-2.0
