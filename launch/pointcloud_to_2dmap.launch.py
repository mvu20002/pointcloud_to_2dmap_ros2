#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('pointcloud_to_2dmap_ros2'),
        'config',
        'config.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='pointcloud_to_2dmap_ros2',
            executable='pointcloud_to_2dmap_node',
            name='pointcloud_to_2dmap_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                # Add any topic remappings here if needed
            ]
        )
    ])
