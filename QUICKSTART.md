# Quick Start Guide

## Start Simulation
```bash
# Terminal 1
MicroXRCEAgent udp4 -p 8888

# Terminal 2
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 3
source ~/px4_ws/setup_px4.bash
```

## Run Missions
```bash
# Square pattern
ros2 run px4_custom_nodes square_flight

# Default waypoints
ros2 launch px4_custom_nodes waypoint_navigator_launch.py

# Figure-8
ros2 launch px4_custom_nodes waypoint_navigator_launch.py waypoint_file:=figure8.yaml

# Patrol at 10m altitude
ros2 launch px4_custom_nodes waypoint_navigator_launch.py waypoint_file:=patrol.yaml takeoff_height:=10.0
```

## Monitor Topics
```bash
# Position
ros2 topic echo /fmu/out/vehicle_local_position_v1

# Status
ros2 topic echo /fmu/out/vehicle_status_v1

# List all
ros2 topic list | grep fmu
```

## Useful Commands
```bash
# Rebuild specific package
colcon build --packages-select px4_custom_nodes

# Clean rebuild
rm -rf build/ install/ log/ && colcon build

# Check package
ros2 pkg list | grep px4

# Test connection
ros2 run px4_custom_nodes test_connection
```
