# Pothole Simulation & Visualization

A ROS 2 Humble simulation system for generating and visualizing pothole defects in Gazebo with real-time metrics publishing.

## Quick Start

### Build the Workspace
```bash
cd /home/fyp/ros2_ws
colcon build --packages-select pothole_sim
source install/setup.bash
```

### Launch the Simulation
```bash
# Default: Gazebo + generator + visualization markers (no RViz)
ros2 launch pothole_sim complete_visualization.launch.py

# With RViz enabled (may fail on systems with snap/GLIBC conflicts)
ros2 launch pothole_sim complete_visualization.launch.py launch_rviz:=true
```

## Visualization Options

### Option 1: Standard Launch (Recommended)
Starts Gazebo, pothole generator, and visualization node without RViz:
```bash
ros2 launch pothole_sim complete_visualization.launch.py
```

**What you'll see:**
- Gazebo window with the pothole world
- Gazebo console showing generator spawn attempts
- Markers published to `/pothole_markers` (viewable in RViz if you add it)

### Option 2: Headless Marker Monitor
Run the lightweight marker visualizer to see marker data in console:
```bash
# Terminal 1: Launch simulation
ros2 launch pothole_sim complete_visualization.launch.py

# Terminal 2: Monitor markers
ros2 run pothole_sim marker_visualizer
```

### Option 3: With Metrics Reader
Monitor pothole metrics in real-time:
```bash
# Terminal 1: Launch simulation
ros2 launch pothole_sim complete_visualization.launch.py

# Terminal 2: Display metrics
ros2 run pothole_sim pothole_metrics_reader
```

### Option 4: RViz Visualization (If Available)
If RViz works on your system, launch it separately:
```bash
# Terminal 1: Launch simulation
ros2 launch pothole_sim complete_visualization.launch.py launch_rviz:=false

# Terminal 2: Launch RViz (apt-installed version)
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix pothole_sim)/share/pothole_sim/config/pothole_rviz_config.rviz
```

**Note:** On systems with snap-installed RViz, you may encounter GLIBC symbol errors. Use an apt-installed version or use the headless marker visualizer instead.

## System Components

### 1. Pothole Generator (`pothole_generator`)
- Generates random pothole positions and geometries
- Spawns them into Gazebo using `/spawn_entity` service
- Publishes pothole data to `/potholes` topic as `PotholeArray`
- Runs automatically in the complete visualization launch

**Manual run:**
```bash
ros2 run pothole_sim pothole_generator
```

### 2. Pothole Visualization Node (`pothole_visualization`)
- Subscribes to `/potholes` topic
- Generates visualization markers (cylinders + text labels)
- Publishes to `/pothole_markers` for RViz display
- Runs automatically in the complete visualization launch

**Manual run:**
```bash
ros2 run pothole_sim pothole_visualization_node
```

### 3. Metrics Reader (`pothole_metrics_reader`)
- Subscribes to `/potholes` topic
- Displays pothole severity, geometry, and position data
- Useful for monitoring while simulation runs

**Manual run:**
```bash
ros2 run pothole_sim pothole_metrics_reader
```

### 4. Marker Visualizer (`marker_visualizer`)
- Lightweight monitor of `/pothole_markers` topic
- Logs marker details without requiring RViz
- Useful for headless or low-resource systems

**Manual run:**
```bash
ros2 run pothole_sim marker_visualizer
```

## Custom Launch File

To launch individual components without the complete visualization:
```bash
# Just Gazebo + generator
ros2 launch pothole_sim spawn_potholes.launch.py

# Run visualization separately
ros2 run pothole_sim pothole_visualization_node
```

## Topics

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/potholes` | `PotholeArray` | `pothole_generator` | `pothole_visualization`, `pothole_metrics_reader` |
| `/pothole_markers` | `MarkerArray` | `pothole_visualization` | RViz, `marker_visualizer` |

## Services

| Service | Type | Provider |
|---------|------|----------|
| `/spawn_entity` | `gazebo_msgs/SpawnEntity` | Gazebo (factory plugin) |

## Files Structure

```
src/pothole_sim/
├── launch/
│   ├── spawn_potholes.launch.py          # Gazebo + generator only
│   └── complete_visualization.launch.py  # Full pipeline
├── pothole_sim/
│   ├── pothole_generator.py              # Generator node
│   ├── pothole_visualization_node.py     # Marker publisher
│   ├── pothole_metrics_reader.py         # Metrics logger
│   └── marker_visualizer.py              # Lightweight marker monitor
├── models/
│   └── pothole_patch/
│       └── model.sdf                     # Pothole model template
├── worlds/
│   └── pothole_world.sdf                 # Gazebo world file
├── config/
│   └── pothole_rviz_config.rviz          # RViz visualization config
├── setup.py                              # Package setup with entry points
└── package.xml
```

## Troubleshooting

### "Failed to spawn ... (no response)"
The `/spawn_entity` service may be slow to respond during Gazebo initialization. This is usually transient. Potholes should still spawn after a few attempts. If spawning fails completely, ensure Gazebo has fully loaded (wait for "Connected to gazebo master" message).

### RViz Symbol Lookup Error
```
symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol
```
**Solution:** Use the apt-installed rviz2 instead of snap version, or use `marker_visualizer` for a headless alternative.

### Gazebo Port Already in Use
```
EXCEPTION: Unable to start server[bind: Address already in use]
```
**Solution:** Kill lingering processes:
```bash
pkill -9 gazebo
pkill -9 gzserver
sleep 2
# Try launch again
```

### No Markers Visible in RViz
1. Ensure the visualization node is running: `ros2 node list | grep visualization`
2. Check the topic exists: `ros2 topic list | grep markers`
3. In RViz, add a "Marker" display and set topic to `/pothole_markers`
4. Set the fixed frame to `map`

## Environment

- **OS:** Ubuntu 20.04+ (tested on Linux)
- **ROS 2:** Humble
- **Gazebo:** 11.10.2
- **Python:** 3.10+
- **DDS Middleware:** CycloneDDS (default) or Fast-RTPS (can be set via `RMW_IMPLEMENTATION`)

## License

See LICENSE file in the repository.
