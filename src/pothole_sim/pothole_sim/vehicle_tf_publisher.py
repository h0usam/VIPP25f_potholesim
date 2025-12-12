#!/usr/bin/env python3
"""
Vehicle TF Publisher for Gazebo Classic
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
from tf2_ros import TransformBroadcaster

class GazeboVehicleTfPublisher(Node):
    def __init__(self):
        super().__init__('gazebo_vehicle_tf_publisher')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to Gazebo model states
        self.model_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        
        self.get_logger().info('Gazebo Vehicle TF Publisher started')
    
    def model_states_callback(self, msg):
        try:
            vehicle_idx = msg.name.index('simple_vehicle')
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'vehicle'
            
            t.transform.translation.x = msg.pose[vehicle_idx].position.x
            t.transform.translation.y = msg.pose[vehicle_idx].position.y
            t.transform.translation.z = msg.pose[vehicle_idx].position.z
            
            t.transform.rotation.x = msg.pose[vehicle_idx].orientation.x
            t.transform.rotation.y = msg.pose[vehicle_idx].orientation.y
            t.transform.rotation.z = msg.pose[vehicle_idx].orientation.z
            t.transform.rotation.w = msg.pose[vehicle_idx].orientation.w
            
            self.tf_broadcaster.sendTransform(t)
            
        except ValueError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = GazeboVehicleTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
