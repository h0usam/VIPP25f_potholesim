import os
import math
import random

import rclpy
import time
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from pothole_interfaces.msg import Pothole, PotholeArray


class PotholeGenerator(Node):
    def __init__(self):
        super().__init__('pothole_generator')

        # Parameters (can be overridden via launch)
        self.declare_parameter('num_potholes', 12)
        self.declare_parameter('x_min', 0.0)
        self.declare_parameter('x_max', 150.0)
        self.declare_parameter('lane_width', 3.5)
        self.declare_parameter('road_center_y', 0.0)
        self.declare_parameter('lane_count', 2)

        self.num_potholes = self.get_parameter(
            'num_potholes').get_parameter_value().integer_value
        self.x_min = self.get_parameter(
            'x_min').get_parameter_value().double_value
        self.x_max = self.get_parameter(
            'x_max').get_parameter_value().double_value
        self.lane_width = self.get_parameter(
            'lane_width').get_parameter_value().double_value
        self.road_center_y = self.get_parameter(
            'road_center_y').get_parameter_value().double_value
        self.lane_count = self.get_parameter(
            'lane_count').get_parameter_value().integer_value

        # Gazebo spawn service client
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        # Publisher for pothole parameters
        self.pothole_pub = self.create_publisher(
            PotholeArray, '/potholes', 10)

        # Spawn once after a delay so Gazebo is ready
        self.already_spawned = False
        self.timer = self.create_timer(3.0, self.spawn_and_publish_once)

    # --------- Geometry helpers ----------

    def sample_pothole(self, idx: int) -> Pothole:
        """
        Sample a random pothole on a 1-way 2-lane road.

        - x in [x_min, x_max]
        - lane centers at ± lane_width / 2 around road_center_y
        - small lateral jitter per lane
        - ellipse radii a, b
        - random yaw
        - depth, edge_angle, severity heuristic
        """
        # Longitudinal position along the road
        x = random.uniform(self.x_min, self.x_max)

        # Choose lane: 0 or 1 for two lanes
        lane_idx = random.randint(0, self.lane_count - 1)
        lane_sign = -1.0 if lane_idx == 0 else 1.0
        lane_center_y = self.road_center_y + lane_sign * (self.lane_width / 2.0)

        # Small lateral jitter around lane center
        lateral_jitter = random.uniform(-0.5, 0.5)
        y = lane_center_y + lateral_jitter

        # Clamp within road width (road is lane_count * lane_width)
        road_half_width = self.lane_width * self.lane_count / 2.0
        margin = 0.2
        y = max(-road_half_width + margin, min(road_half_width - margin, y))

        # Randomized values
        center_x = random.uniform(self.x_min, self.x_max)
        center_y = random.uniform(-road_half_width, road_half_width)
        a = random.uniform(0.3, 0.8)  # Semi-axis a
        b = random.uniform(0.2, 0.6)  # Semi-axis b
        yaw = random.uniform(0, 2 * math.pi)  # Random rotation
        depth_max = random.uniform(0.1, 0.18)  # Random depth

        # Computed values
        area = a * b * math.pi  # Ellipse area formula

        # Edge angle computed from depth and dimensions
        edge_angle = math.atan(depth_max / ((a + b) / 2))

        # Severity = function of (depth, area, edge_angle)
        # Normalized combination of these factors
        severity = (
            (depth_max / 0.18) * 0.4 +      # 40% weight on depth
            (area / 0.8) * 0.2 +             # 40% weight on area
            (edge_angle / 1.0) * 0.4     # 20% weight on edge angle
        )
        severity = min(severity, 1.0)  # Clamp to [0, 1]

        # Road model: top surface at z ≈ 0.02 -> put patch slightly above
        road_top_z = 0.02
        patch_epsilon = 0.002

        pothole = Pothole()
        pothole.header = Header()
        pothole.header.stamp = self.get_clock().now().to_msg()
        pothole.header.frame_id = 'map'

        pothole.center = Point(
            x=center_x,
            y=center_y,
            z=road_top_z + patch_epsilon
        )
        pothole.a = float(a)
        pothole.b = float(b)
        pothole.yaw = float(yaw)
        pothole.depth_max = float(depth_max)
        pothole.edge_angle = float(edge_angle)
        pothole.severity = float(severity)
        pothole.id = idx

        return pothole

    def make_scaled_sdf(self, base_xml: str, pothole: Pothole) -> str:
        """
        Scale the base pothole model so the cylinder becomes an ellipse
        with semi-axes a, b.

        Base model uses radius = 0.5, so:
          effective_radius_x = radius * scale_x = a
          effective_radius_y = radius * scale_y = b
        => scale_x = a / 0.5, scale_y = b / 0.5
        """
        base_radius = 0.5
        scale_x = pothole.a / base_radius
        scale_y = pothole.b / base_radius

        scaled_xml = base_xml.replace(
            "<scale>1 1 1</scale>",
            f"<scale>{scale_x:.3f} {scale_y:.3f} 1.0</scale>"
        )

        return scaled_xml

    # --------- Main spawn + publish logic ----------

    def spawn_and_publish_once(self):
        if self.already_spawned:
            return

        pothole_list = []

        # Load base model XML
        pkg_share = get_package_share_directory('pothole_sim')
        model_path = os.path.join(
            pkg_share, 'models', 'pothole_patch', 'model.sdf')
        with open(model_path, 'r') as f:
            pothole_sdf = f.read()

        for i in range(self.num_potholes):
            pothole = self.sample_pothole(i)

            # Scale the visual to match this pothole's ellipse (a, b)
            scaled_sdf = self.make_scaled_sdf(pothole_sdf, pothole)

            self.spawn_single_pothole(pothole, scaled_sdf)
            pothole_list.append(pothole)
            # brief pause between spawn requests to avoid overwhelming the factory
            time.sleep(0.2)

        msg = PotholeArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.potholes = pothole_list

        self.pothole_pub.publish(msg)
        self.get_logger().info(
            f"Spawned and published {len(pothole_list)} potholes.")
        self.already_spawned = True

    def spawn_single_pothole(self, pothole: Pothole, sdf_xml: str):
        # Ensure service is available
        if not self.spawn_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                "Spawn service /spawn_entity not available (timeout)")
            return

        req = SpawnEntity.Request()
        req.name = f"pothole_{pothole.id}"
        req.xml = sdf_xml
        req.robot_namespace = ""
        req.reference_frame = "world"

        # Pose from pothole.center + yaw (z rotation only)
        cy = math.cos(pothole.yaw * 0.5)
        sy = math.sin(pothole.yaw * 0.5)

        req.initial_pose.position.x = pothole.center.x
        req.initial_pose.position.y = pothole.center.y
        req.initial_pose.position.z = pothole.center.z

        req.initial_pose.orientation.x = 0.0
        req.initial_pose.orientation.y = 0.0
        req.initial_pose.orientation.z = sy
        req.initial_pose.orientation.w = cy

        self.get_logger().info(
            f"Spawning pothole_{pothole.id} at "
            f"({pothole.center.x:.2f}, {pothole.center.y:.2f}), "
            f"yaw={pothole.yaw:.2f}"
        )

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(
                f"Spawn call exception for pothole_{pothole.id}: {e}")
            result = None

        if result is None:
            self.get_logger().warn(
                f"Failed to spawn pothole_{pothole.id} (no response)")
            # Log a short XML snippet for debugging (first 400 chars)
            snippet = sdf_xml.replace('\n', ' ')[:400]
            self.get_logger().info(
                f"Pothole XML snippet (len={len(sdf_xml)}): {snippet}...")
        else:
            if result.success:
                self.get_logger().info(
                    f"Spawned pothole_{pothole.id}: {result.status_message}")
            else:
                self.get_logger().warn(
                    f"Failed to spawn pothole_{pothole.id}: {result.status_message}")
                # Log a short XML snippet to help diagnose malformed SDF
                snippet = sdf_xml.replace('\n', ' ')[:400]
                self.get_logger().info(
                    f"Pothole XML snippet (len={len(sdf_xml)}): {snippet}...")


def main(args=None):
    rclpy.init(args=args)
    node = PotholeGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

