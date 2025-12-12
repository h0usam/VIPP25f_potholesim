#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pothole_sim_dir = get_package_share_directory('pothole_sim')
    
    # 1. Start Gazebo with pothole world
    world_file = os.path.join(pothole_sim_dir, 'worlds', 'pothole_world.sdf')
    # Ensure the preferred RMW implementation is used (avoid CycloneDDS issues)
    rmw_env = SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_fastrtps_cpp')

    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        shell=True
    )
    
    # 2. Start visualization node (creates markers from potholes)
    visualization_node = Node(
        package='pothole_sim',
        executable='pothole_visualization_node',
        name='pothole_visualization',
        output='screen'
    )
    
    # 3. Start RViz (will be empty, configure manually)
    rviz_node = ExecuteProcess(
        cmd=['rviz2'],
        output='screen',
        shell=True
    )
    
    # 4. Start pothole generator after 5-second delay (let Gazebo initialize)
    pothole_generator = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='pothole_sim',
                executable='pothole_generator',
                name='pothole_generator',
                output='screen',
                parameters=[{'num_potholes': 12}]
            )
        ]
    )
    
    return LaunchDescription([
        rmw_env,
        gazebo_process,
        visualization_node,
        rviz_node,
        pothole_generator
    ])
