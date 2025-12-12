#!/usr/bin/env python3
"""
Pothole Visualization Node
Subscribes to /potholes topic and publishes RViz markers with color-coded severity
Colors: Green (<0.6), Yellow (0.6-0.78), Red (>0.78)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from pothole_interfaces.msg import PotholeArray, Pothole
import math

class PotholeVisualizationNode(Node):
    def __init__(self):
        super().__init__('pothole_visualization_node')
        
        # Parameters
        self.declare_parameter('marker_lifetime', 0.5)
        self.declare_parameter('marker_frame_id', 'map')
        self.declare_parameter('pothole_topic', '/potholes')
        self.declare_parameter('marker_topic', '/pothole_markers')
        
        # Get parameters
        marker_lifetime = self.get_parameter('marker_lifetime').value
        marker_frame_id = self.get_parameter('marker_frame_id').value
        pothole_topic = self.get_parameter('pothole_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        
        # Create publisher for RViz markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            marker_topic,
            10
        )
        
        # Create subscriber for potholes
        self.pothole_sub = self.create_subscription(
            PotholeArray,
            pothole_topic,
            self.pothole_callback,
            10
        )
        
        # Severity color mapping
        self.severity_colors = {
            'low': ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7),     # Green
            'medium': ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7),  # Yellow
            'high': ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)     # Red
        }
        
        self.get_logger().info('Pothole Visualization Node started')
        self.get_logger().info(f'Subscribing to: {pothole_topic}')
        self.get_logger().info(f'Publishing to: {marker_topic}')
    
    def get_severity_category(self, severity):
        """Categorize pothole based on severity thresholds"""
        if severity < 0.6:
            return 'low'
        elif severity <= 0.78:
            return 'medium'
        else:
            return 'high'
    
    def create_ellipse_marker(self, pothole, marker_id):
        """Create an ellipse-shaped marker for a pothole"""
        marker = Marker()
        
        # Basic marker properties
        marker.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='map'
        )
        marker.ns = "potholes"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position = pothole.center
        
        # Set orientation from yaw (rotation around Z)
        yaw = pothole.yaw
        marker.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw/2.0),
            w=math.cos(yaw/2.0)
        )
        
        # Set scale (CYLINDER uses x and y as diameters)
        marker.scale.x = 2.0 * pothole.a    # Major axis diameter
        marker.scale.y = 2.0 * pothole.b    # Minor axis diameter
        marker.scale.z = 0.1  # Height for visibility (not actual depth)
        
        # Set color based on severity
        severity_category = self.get_severity_category(pothole.severity)
        marker.color = self.severity_colors[severity_category]
        
        return marker
    
    def create_text_marker(self, pothole, marker_id_offset):
        """Create text marker showing severity value"""
        marker = Marker()
        
        marker.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='map'
        )
        marker.ns = "pothole_labels"
        marker.id = marker_id_offset
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position text above pothole
        marker.pose.position = Point(
            x=pothole.center.x,
            y=pothole.center.y,
            z=pothole.center.z + 0.15  # 15cm above
        )
        marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Text properties
        marker.text = f"{pothole.severity:.2f}"
        marker.scale.z = 0.1  # Text height
        
        # Black text for contrast
        marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        
        return marker
    
    def pothole_callback(self, msg):
        """Callback when new pothole data is received"""
        self.get_logger().info(f'Received {len(msg.potholes)} potholes')
        
        marker_array = MarkerArray()
        
        for i, pothole in enumerate(msg.potholes):
            # Create ellipse marker
            ellipse_marker = self.create_ellipse_marker(pothole, i)
            marker_array.markers.append(ellipse_marker)
            
            # Create text marker
            text_marker = self.create_text_marker(pothole, i + 1000)
            marker_array.markers.append(text_marker)
        
        # Publish markers
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PotholeVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down visualization node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()