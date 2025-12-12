#!/usr/bin/env python3
"""
Simple test script to run the pothole metrics reader directly
"""
import subprocess
import sys
import time
import os

# Ensure we're in the right directory
os.chdir('/home/fyp/ros2_ws')

# Source the setup
setup_cmd = 'source install/setup.bash'

# Start the launch in the background
print("[Test] Launching simulation...")
launch_proc = subprocess.Popen(
    f'{setup_cmd} && ros2 launch pothole_sim spawn_potholes.launch.py',
    shell=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    executable='/bin/bash'
)

# Wait for simulation to initialize
print("[Test] Waiting 20 seconds for simulation to initialize...")
time.sleep(20)

# Run the metrics reader for 30 seconds
print("[Test] Starting metrics reader...")
metrics_proc = subprocess.Popen(
    f'{setup_cmd} && timeout 30 ros2 run pothole_sim pothole_metrics_reader',
    shell=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    executable='/bin/bash'
)

# Capture output from metrics reader
try:
    output, _ = metrics_proc.communicate(timeout=35)
    print(output.decode('utf-8', errors='ignore'))
except subprocess.TimeoutExpired:
    metrics_proc.kill()
    output, _ = metrics_proc.communicate()
    print(output.decode('utf-8', errors='ignore'))

print("[Test] Cleaning up launch process...")
launch_proc.terminate()
try:
    launch_proc.wait(timeout=5)
except subprocess.TimeoutExpired:
    launch_proc.kill()

print("[Test] Test complete!")
