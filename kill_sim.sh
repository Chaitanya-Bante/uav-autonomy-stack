#!/bin/bash

echo "Killing PX4 and Gazebo processes..."

# Kill PX4
pkill -9 px4
pkill -9 -f px4_sitl

# Kill Gazebo
pkill -9 gz
pkill -9 gzserver
pkill -9 gzclient
pkill -9 ruby

# Kill MicroXRCEAgent
pkill -9 MicroXRCEAgent

echo "Done!"
