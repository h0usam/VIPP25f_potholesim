from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('pothole_sim')

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'pothole_world.launch.py')
        )
    )

    pothole_generator = Node(
        package='pothole_sim',
        executable='pothole_generator',
        name='pothole_generator',
        output='screen',
        parameters=[{
            'num_potholes': 12,
            'x_min': 0.0,
            'x_max': 150.0,
            'lane_width': 3.5,
            'road_center_y': 0.0,
            'lane_count': 2,
        }]
    )

    # Start generator a few seconds after Gazebo
    delayed_generator = TimerAction(
        period=5.0,
        actions=[pothole_generator]
    )

    return LaunchDescription([
        world_launch,
        delayed_generator
    ])

