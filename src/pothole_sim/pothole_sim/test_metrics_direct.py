#!/usr/bin/env python3
"""
Direct test of pothole metrics reader
"""
import rclpy
from rclpy.node import Node
from pothole_interfaces.msg import PotholeArray
import sys

class TestMetricsReader(Node):
    def __init__(self):
        super().__init__('test_metrics_reader')
        self.pothole_count = 0
        self.subscription = self.create_subscription(
            PotholeArray,
            '/potholes',
            self.callback,
            10
        )

    def callback(self, msg):
        if self.pothole_count == 0:
            print("\n" + "="*80)
            print("POTHOLE METRICS")
            print("="*80)
        
        for pothole in msg.potholes:
            self.pothole_count += 1
            area = pothole.a * pothole.b
            print(f"\nPothole {pothole.id}:")
            print(f"  Position: ({pothole.center.x:.2f}, {pothole.center.y:.2f}, {pothole.center.z:.4f})")
            print(f"  Severity: {pothole.severity:.3f}")
            print(f"  Area (a*b): {area:.3f} m²")
            print(f"  Semi-axes: a={pothole.a:.3f}, b={pothole.b:.3f}")
            print(f"  Depth (max): {pothole.depth_max:.4f} m")
            print(f"  Edge angle: {pothole.edge_angle:.4f} rad ({pothole.edge_angle * 180 / 3.14159:.1f}°)")
            print(f"  Yaw: {pothole.yaw:.4f} rad")

if __name__ == '__main__':
    rclpy.init()
    reader = TestMetricsReader()
    
    print("Waiting for pothole messages (subscribing to /potholes)...")
    print("Press Ctrl+C to stop.\n")
    
    try:
        rclpy.spin(reader)
    except KeyboardInterrupt:
        pass
    finally:
        reader.destroy_node()
        rclpy.shutdown()
        print(f"\n\nTotal potholes received: {reader.pothole_count}")
