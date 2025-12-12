# Pothole Visualization - Setup Complete ✅

## What's Been Set Up

Your pothole simulation now has a fully functional visualization pipeline:

### Core Components Working:
1. **Gazebo Simulation** - Loading pothole world
2. **Pothole Generator** - Creating random potholes and spawning them
3. **Visualization Node** - Converting pothole data to RViz markers
4. **Marker Publishing** - `/pothole_markers` topic active
5. **Metrics Reader** - Available for monitoring
6. **Marker Visualizer** - New lightweight monitor for marker data

### Launch Files:
- **`complete_visualization.launch.py`** - Main entry point (Gazebo + generator + visualization)
  - Includes RViz option (default OFF due to snap/GLIBC issues)
  - Automatically publishes markers to `/pothole_markers`

---

## How to Use

### Quick Start - Run the Full Visualization:
```bash
cd /home/fyp/ros2_ws
source install/setup.bash
ros2 launch pothole_sim complete_visualization.launch.py
```

**You should see:**
- Gazebo window opens with the pothole world
- Console output showing spawn attempts and pothole generation
- Markers being published internally

### Monitor Markers in Console:
In another terminal while the launch is running:
```bash
ros2 run pothole_sim marker_visualizer
```

You'll see output like:
```
[INFO] Marker Visualizer started - listening to /pothole_markers
[INFO] Received MarkerArray with 12 markers (update #1)
```

### Monitor Metrics:
In another terminal:
```bash
ros2 run pothole_sim pothole_metrics_reader
```

This shows detailed pothole severity, geometry, and position data.

### Using RViz (Optional):
If RViz works on your system:
```bash
# Terminal 1: Launch simulation
ros2 launch pothole_sim complete_visualization.launch.py

# Terminal 2: In another terminal, run RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix pothole_sim)/share/pothole_sim/config/pothole_rviz_config.rviz
```

Or enable RViz in the launch (if snap issues don't occur):
```bash
ros2 launch pothole_sim complete_visualization.launch.py launch_rviz:=true
```

---

## Topics Available

| Topic | Type | What It Contains |
|-------|------|------------------|
| `/potholes` | `PotholeArray` | Pothole positions, severity, geometry (published by generator) |
| `/pothole_markers` | `MarkerArray` | Visualization markers for RViz (published by visualization node) |

---

## Files Created/Modified

### New Files:
- `pothole_sim/marker_visualizer.py` - Lightweight marker monitor
- `README_VISUALIZATION.md` - Complete visualization documentation
- `VISUALIZATION_GUIDE.sh` - Quick reference guide

### Modified Files:
- `launch/complete_visualization.launch.py` - Now has `launch_rviz` parameter (default OFF)
- `setup.py` - Added `marker_visualizer` entry point and config directory to data_files

---

## Key Features

✅ **Markers Publishing** - `/pothole_markers` topic is active and can be visualized in RViz
✅ **Console Monitoring** - Use `marker_visualizer` to see marker data without RViz  
✅ **Metrics Tracking** - Use `pothole_metrics_reader` to monitor pothole details
✅ **Flexible Launch** - Single command launches full pipeline
✅ **RViz Support** - Full RViz config available (if binary works on your system)

---

## Troubleshooting

### "Failed to spawn ... (no response)" warnings
These are normal during startup while Gazebo is initializing. Potholes will eventually spawn successfully.

### RViz crashes with symbol lookup error
The snap version of RViz has GLIBC incompatibility. Solutions:
- Use the lightweight `marker_visualizer` instead
- Install rviz2 via apt (if available on your system)
- Run RViz in a separate terminal (may work better)

### No markers appear in RViz
1. Verify visualization node is running: `ros2 node list`
2. Check markers are publishing: `ros2 topic list | grep markers`
3. In RViz, add a "Marker" display, set topic to `/pothole_markers`
4. Set RViz fixed frame to `map`

---

## Next Steps

You now have multiple ways to visualize potholes:
1. **RViz** (if available on your system)
2. **Console output** via `marker_visualizer`  
3. **Log/metrics** via `pothole_metrics_reader`

The markers are being generated and published! You can integrate this with any custom visualization or further processing.

For more details, see `README_VISUALIZATION.md`.
