#!/bin/bash

# PX4 ROS2 Environment Setup
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/px4_ws/install/setup.bash

echo "================================"
echo "PX4 ROS2 Environment Ready!"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Workspace: ~/px4_ws"
echo "================================"
