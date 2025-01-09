# !/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    node1 = Node(
                package='occupancy_map_visualizer',
                executable='octomap_visualizer_node',
                name='octomap_visualizer_node',
                parameters=[
                    {'octomap_filename': os.path.join(get_package_share_directory('occupancy_map_visualizer'), 'samples', 'octree.bt')},
                    {'fixed_frame_id': "map"}
                ]
            )
    
    node2 = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', [os.path.join(get_package_share_directory('occupancy_map_visualizer'), 'rviz', 'octomap.rviz')]]
            )
    
    ld.add_action(node1)
    ld.add_action(node2)

    return ld
