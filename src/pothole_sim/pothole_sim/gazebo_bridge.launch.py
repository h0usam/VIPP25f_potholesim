#!/usr/bin/env python3
"""
Launches bridge between Gazebo (Ignition) and ROS 2
Bridges SDF models to RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Bridge for model states (poses)
    bridge_model_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='model_states_bridge',
        arguments=[
            '/world/pothole_world/model_states@ignition.msgs.ModelStates[ros_gz_interfaces/msg/ModelStates',
            '--ros-args', '-r', '/world/pothole_world/model_states:=/model_states'
        ],
        output='screen'
    )
    
    # Bridge for vehicle pose specifically
    bridge_vehicle_pose = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vehicle_pose_bridge',
        arguments=[
            '/model/simple_vehicle/pose@ignition.msgs.Pose[geometry_msgs/msg/PoseStamped',
            '--ros-args', '-r', '/model/simple_vehicle/pose:=/vehicle/pose'
        ],
        output='screen'
    )
    
    # Bridge for vehicle TF
    bridge_vehicle_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vehicle_tf_bridge',
        arguments=[
            '/model/simple_vehicle/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '--ros-args', '-r', '/model/simple_vehicle/tf:=/tf_gz'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        bridge_model_states,
        bridge_vehicle_pose,
        bridge_vehicle_tf,
    ])