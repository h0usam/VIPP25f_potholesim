from setuptools import setup
import os
from glob import glob

package_name = 'pothole_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models/pothole_patch'), glob('models/pothole_patch/*')),
        (os.path.join('share', package_name, 'models/simple_vehicle'), glob('models/simple_vehicle/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),  # THIS LINE MUST EXIST
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'pothole_generator = pothole_sim.pothole_generator:main',
            'pothole_metrics_reader = pothole_sim.pothole_metrics_reader:main',
            'pothole_visualization_node = pothole_sim.pothole_visualization_node:main',
            'pothole_visualization = pothole_sim.pothole_visualization_node:main',
            'marker_visualizer = pothole_sim.marker_visualizer:main',
            'vehicle_tf_publisher = pothole_sim.vehicle_tf_publisher:main'
        ],
    },
)
