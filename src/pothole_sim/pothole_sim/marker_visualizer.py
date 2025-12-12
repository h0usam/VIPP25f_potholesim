#!/usr/bin/env python3
"""
Standalone Marker Visualizer for Potholes
Subscribes to /pothole_markers and logs/displays them without requiring RViz.
Can be used as a lightweight alternative when RViz snap/GLIBC issues occur.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray


class MarkerVisualizer(Node):
    """Simple marker subscriber that logs marker data"""

    def __init__(self):
        super().__init__('marker_visualizer')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/pothole_markers',
            self.marker_callback,
            10
        )
        self.get_logger().info("Marker Visualizer started - listening to /pothole_markers")
        self.marker_count = 0

    def marker_callback(self, msg):
        """Callback when markers are received"""
        self.marker_count += 1
        self.get_logger().info(
            f"Received MarkerArray with {len(msg.markers)} markers (update #{self.marker_count})"
        )
        for i, marker in enumerate(msg.markers):
            self.get_logger().debug(
                f"  Marker {i}: id={marker.id}, type={marker.type}, "
                f"pose=({marker.pose.position.x:.2f}, {marker.pose.position.y:.2f}), "
                f"scale=({marker.scale.x:.2f}, {marker.scale.y:.2f}, {marker.scale.z:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
