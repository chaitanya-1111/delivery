#!/bin/bash
# gazebo_launch.sh
# Quick launcher script for robot in Gazebo

set -e  # Exit on error

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🤖 Delivery Robot - Gazebo Simulation Launcher"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Setup ROS 2 environment
cd /home/chaitanya/delivery_bot_ws
source /home/chaitanya/delivery_bot_ws/.venv/bin/activate
source install/setup.bash

echo ""
echo "✓ Environment sourced"
echo ""
echo "Launching Gazebo with robot..."
echo ""

# Launch Gazebo with the robot
ros2 launch robot_description_pkg gazebo_bringup.launch.py "$@"
