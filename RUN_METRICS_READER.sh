#!/bin/bash
# Quick test script for pothole metrics reader

# In terminal 1, launch the simulation:
# cd /home/fyp/ros2_ws
# source install/setup.bash
# ros2 launch pothole_sim spawn_potholes.launch.py

# In terminal 2 (after waiting ~20 seconds for sim to initialize), run the metrics reader:
# cd /home/fyp/ros2_ws
# source install/setup.bash
# ros2 run pothole_sim pothole_metrics_reader

# The metrics reader will continuously subscribe to /potholes and print:
# - Pothole ID
# - Position (x, y, z)
# - Severity score
# - Ellipse area and semi-axes
# - Max depth
# - Edge angle
# - Yaw orientation

echo "Steps to test pothole metrics reader:"
echo ""
echo "1. In one terminal, launch the simulation:"
echo "   cd /home/fyp/ros2_ws"
echo "   source install/setup.bash"
echo "   ros2 launch pothole_sim spawn_potholes.launch.py"
echo ""
echo "2. In another terminal (after ~20 seconds), run the metrics reader:"
echo "   cd /home/fyp/ros2_ws"
echo "   source install/setup.bash"
echo "   ros2 run pothole_sim pothole_metrics_reader"
echo ""
echo "The metrics reader will display all pothole parameters as they are published."
