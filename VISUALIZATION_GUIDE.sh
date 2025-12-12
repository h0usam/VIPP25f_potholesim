#!/bin/bash
# Quick Visualization Guide for Pothole Simulator
# This script demonstrates how to launch the visualization with different options

echo "=========================================="
echo "Pothole Simulator - Visualization Guide"
echo "=========================================="
echo ""

# Source the ROS2 setup
source /home/fyp/ros2_ws/install/setup.bash

echo "[Option 1] Launch WITHOUT RViz (fastest, most reliable)"
echo "  Command: ros2 launch pothole_sim complete_visualization.launch.py"
echo "  This starts Gazebo, generator, and visualization markers"
echo ""

echo "[Option 2] Monitor markers without GUI (useful for headless systems)"
echo "  Terminal 1: ros2 launch pothole_sim complete_visualization.launch.py"
echo "  Terminal 2: ros2 run pothole_sim marker_visualizer"
echo ""

echo "[Option 3] Try RViz (if snap/GLIBC issues don't occur)"
echo "  Terminal 1: ros2 launch pothole_sim complete_visualization.launch.py launch_rviz:=true"
echo "  Or run RViz separately in another terminal:"
echo "    ros2 run rviz2 rviz2 -d \$(ros2 pkg prefix pothole_sim)/share/pothole_sim/config/pothole_rviz_config.rviz"
echo ""

echo "[Option 4] Monitor metrics while visualization runs"
echo "  Terminal 1: ros2 launch pothole_sim complete_visualization.launch.py"
echo "  Terminal 2: ros2 run pothole_sim pothole_metrics_reader"
echo ""

echo "=========================================="
echo "All nodes available via ros2 run:"
echo "  - pothole_generator (included in launch)"
echo "  - pothole_visualization (marker publisher)"
echo "  - pothole_metrics_reader (metrics subscriber)"
echo "  - marker_visualizer (lightweight marker monitor)"
echo "=========================================="
