#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pothole_interfaces.msg import PotholeArray, PotholeDecision, DecisionArray
from geometry_msgs.msg import Polygon, Point32
import math

class PotholeDecisionNode(Node):
    def __init__(self):
        super().__init__('pothole_decision_node')
        
        # Subscriber for potholes
        self.pothole_sub = self.create_subscription(
            PotholeArray,
            '/potholes',
            self.pothole_callback,
            10
        )
        
        # Publisher for decisions
        self.decision_pub = self.create_publisher(
            DecisionArray,
            '/pothole_decisions',
            10
        )
        
        self.get_logger().info('Pothole Decision Node started')
    
    def pothole_callback(self, msg):
        decisions = DecisionArray()
        decisions.header = msg.header
        
        for pothole in msg.potholes:
            decision = PotholeDecision()
            decision.header = msg.header
            decision.pothole_id = pothole.id
            
            # Apply rule-based decisions
            if pothole.severity < 0.6:
                decision.action = "IGNORE"
                decision.velocity_scale = 1.0
            elif pothole.severity <= 0.78:
                decision.action = "SLOW_DOWN"
                decision.velocity_scale = 0.7
            else:
                decision.action = "AVOID"
                decision.velocity_scale = 0.0
                
                # Create avoidance polygon (circle around pothole)
                avoidance_poly = Polygon()
                radius = max(pothole.a, pothole.b) * 1.5  # Safety margin
                for angle in range(0, 360, 30):
                    x = pothole.center.x + radius * math.cos(math.radians(angle))
                    y = pothole.center.y + radius * math.sin(math.radians(angle))
                    avoidance_poly.points.append(Point32(x=x, y=y, z=0.0))
                decision.avoidance_polygon = avoidance_poly
            
            decisions.decisions.append(decision)
            
            self.get_logger().info(
                f'Pothole {pothole.id}: severity={pothole.severity:.2f} -> {decision.action}'
            )
        
        # Publish decisions
        self.decision_pub.publish(decisions)

def main(args=None):
    rclpy.init(args=args)
    node = PotholeDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()