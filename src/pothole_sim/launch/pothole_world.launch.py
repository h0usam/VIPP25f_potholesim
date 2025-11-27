from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('pothole_sim')
    world_file = os.path.join(pkg_share, 'worlds', 'pothole_world.sdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Full path to world file'
        ),

        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                LaunchConfiguration('world'),
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),
    ])

