#!/bin/bash
# Run RViz with snap libraries bypassed
# This avoids the GLIBC_PRIVATE symbol lookup error
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu
export PATH=/opt/ros/humble/bin:$PATH
exec /opt/ros/humble/lib/rviz2/rviz2 "$@"
