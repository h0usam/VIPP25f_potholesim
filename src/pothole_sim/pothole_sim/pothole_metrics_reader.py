import rclpy
from rclpy.node import Node
from pothole_interfaces.msg import PotholeArray

class PotholeMetricsReader(Node):
    def __init__(self):
        super().__init__('pothole_metrics_reader')
        
        # Create a subscription to /potholes topic
        self.subscription = self.create_subscription(
            PotholeArray, 
            '/potholes', 
            self.pothole_callback, 
            10  # Queue size
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Pothole Metrics Reader initialized. Waiting for /potholes topic...")

    def pothole_callback(self, msg):
        # Loop through all potholes in the PotholeArray
        for pothole in msg.potholes:
            # Extract relevant metrics from each pothole
            pothole_id = pothole.id
            pothole_position = pothole.center
            pothole_severity = pothole.severity
            pothole_area = pothole.a * pothole.b  # Area of the ellipse
            pothole_depth = pothole.depth_max
            pothole_edge_angle = pothole.edge_angle
            pothole_yaw = pothole.yaw
            
            # Print pothole metrics in a clear format
            self.get_logger().info(
                f"\n{'='*70}\n"
                f"Pothole ID: {pothole_id}\n"
                f"  Position: X={pothole_position.x:.2f}m, Y={pothole_position.y:.2f}m, Z={pothole_position.z:.4f}m\n"
                f"  Severity: {pothole_severity:.3f}\n"
                f"  Dimensions: Semi-axes a={pothole.a:.3f}m, b={pothole.b:.3f}m | Area={pothole_area:.4f}m²\n"
                f"  Depth (max): {pothole_depth:.4f}m\n"
                f"  Edge Angle: {pothole_edge_angle:.4f}rad ({pothole_edge_angle*180/3.14159:.1f}°)\n"
                f"  Yaw Rotation: {pothole_yaw:.4f}rad ({pothole_yaw*180/3.14159:.1f}°)\n"
                f"{'='*70}"
            )

def main(args=None):
    rclpy.init(args=args)

    pothole_metrics_reader = PotholeMetricsReader()

    rclpy.spin(pothole_metrics_reader)

    # Destroy the node after use
    pothole_metrics_reader.destroy_node()
    rclpy.shutdown()
