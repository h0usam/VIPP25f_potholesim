#!/bin/bash
# Unified Pothole Visualization Launch Script
# Handles daemon, environment, and snap compatibility issues

set -e

WORKSPACE="/home/fyp/ros2_ws"

# Step 1: Kill all lingering processes
echo "🧹 Cleaning up old processes..."
pkill -9 ros2-daemon 2>/dev/null || true
pkill -9 gazebo 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
pkill -9 pothole_generator 2>/dev/null || true
pkill -9 pothole_visualization 2>/dev/null || true
sleep 2

# Step 2: Clear ROS_DOMAIN_ID issues
echo "🔧 Configuring ROS environment..."
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export DISPLAY=:0

# Step 3: Start fresh daemon (if needed)
echo "✨ Starting ROS2 daemon..."
source /opt/ros/humble/setup.bash
source $WORKSPACE/install/setup.bash
ros2 daemon start 2>/dev/null || true
sleep 2

# Step 4: Launch the unified visualization
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 LAUNCHING POTHOLE VISUALIZATION STACK"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Components launching:"
echo "  📍 Gazebo with pothole world"
echo "  🔧 Pothole generator (random spawning)"
echo "  👁️  Visualization node (marker publishing)"
echo "  🎨 RViz (world + markers display)"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

cd $WORKSPACE
ros2 launch pothole_sim full_visualization.launch.py
