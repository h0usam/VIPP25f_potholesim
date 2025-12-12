#!/bin/bash
# SIMPLE UNIFIED LAUNCH - Pothole Visualization Stack
# All components working together (RViz excluded due to snap/GLIBC)
# USAGE: bash run_pothole_visualization.sh

WORKSPACE="/home/fyp/ros2_ws"

# Clean
pkill -9 gazebo ros2-daemon pothole_generator pothole_visualization rviz2 2>/dev/null || true
sleep 2

# Setup
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Launch
source /opt/ros/humble/setup.bash
source $WORKSPACE/install/setup.bash

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 POTHOLE VISUALIZATION LAUNCHING"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

cd $WORKSPACE
ros2 launch pothole_sim full_visualization.launch.py
