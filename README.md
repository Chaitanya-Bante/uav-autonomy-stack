# PX4 ROS2 Custom Workspace

Custom ROS2 nodes for autonomous PX4 drone control and navigation.

## Features

✅ **Custom Control Nodes**
- Square flight pattern
- Waypoint navigation with YAML mission files
- Connection diagnostics

🚧 **In Progress**
- Visual SLAM integration
- Obstacle avoidance
- Path planning

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- PX4 Autopilot v1.14+
- Gazebo Garden

## Installation
```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/px4_ws.git
cd px4_ws

# Build the workspace
colcon build
source setup_px4.bash
```

## Quick Start

### 1. Start PX4 Simulation
```bash
# Terminal 1: MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888

# Terminal 2: PX4 SITL with Gazebo
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

### 2. Run Missions

**Square Flight Pattern:**
```bash
source ~/px4_ws/setup_px4.bash
ros2 run px4_custom_nodes square_flight
```

**Waypoint Navigation:**
```bash
source ~/px4_ws/setup_px4.bash
ros2 launch px4_custom_nodes waypoint_navigator_launch.py
```

**Custom Mission:**
```bash
ros2 launch px4_custom_nodes waypoint_navigator_launch.py waypoint_file:=patrol.yaml
```

## Available Missions

- `waypoints.yaml` - Basic square pattern
- `figure8.yaml` - Figure-8 flight pattern
- `climb.yaml` - Vertical climb and descent
- `patrol.yaml` - Large square patrol

## Creating Custom Missions

Create a YAML file in `src/px4_custom_nodes/config/`:
```yaml
waypoints:
  - {x: 0.0, y: 0.0, z: 5.0}    # Takeoff
  - {x: 10.0, y: 0.0, z: 5.0}   # Waypoint 1
  - {x: 10.0, y: 10.0, z: 5.0}  # Waypoint 2
  - {x: 0.0, y: 0.0, z: 5.0}    # Return home
```

Coordinates are in NED frame (North-East-Down):
- `x`: North (positive) / South (negative)
- `y`: East (positive) / West (negative)
- `z`: Altitude (always positive, up from ground)

## Project Structure
```
px4_ws/
├── src/
│   ├── px4_custom_nodes/
│   │   ├── px4_custom_nodes/
│   │   │   ├── square_flight_node.py
│   │   │   ├── waypoint_navigator.py
│   │   │   └── test_connection.py
│   │   ├── launch/
│   │   │   ├── square_flight_launch.py
│   │   │   └── waypoint_navigator_launch.py
│   │   ├── config/
│   │   │   ├── waypoints.yaml
│   │   │   ├── figure8.yaml
│   │   │   ├── climb.yaml
│   │   │   └── patrol.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   └── px4_drone_slam/          (Coming soon)
├── setup_px4.bash
└── README.md
```

## Troubleshooting

### No data from PX4

Ensure `ROS_DOMAIN_ID=0` is set:
```bash
export ROS_DOMAIN_ID=0
```

Check if topics are available:
```bash
ros2 topic list | grep fmu
```

### Topics have `_v1` suffix

Your code is updated for PX4 v1.14+ which uses versioned topics.

### Connection test
```bash
ros2 run px4_custom_nodes test_connection
```

## Development Roadmap

- [x] Basic offboard control
- [x] Square flight pattern
- [x] YAML-based waypoint navigation
- [ ] Camera/depth sensor integration
- [ ] Visual SLAM (RTABMap)
- [ ] Obstacle detection and avoidance
- [ ] Dynamic path planning
- [ ] Multi-drone coordination

## Contributing

1. Create a feature branch: `git checkout -b feature/new-feature`
2. Commit changes: `git commit -m "Add new feature"`
3. Push to branch: `git push origin feature/new-feature`
4. Create Pull Request

## License

Apache-2.0

## Author

Created for autonomous drone research and development.
