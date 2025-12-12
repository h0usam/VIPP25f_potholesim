#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Simple TF bridge for vehicle
    tf_publisher = Node(
        package='pothole_sim',
        executable='vehicle_tf_publisher',
        name='vehicle_tf_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        tf_publisher
    ])
