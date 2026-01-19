#!/usr/bin/env python3
"""
ArUco Patrol Launch File

Launches:
- ArUco detector node
- Patrol navigator node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_aruco = get_package_share_directory('ubot_aruco')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    marker_size = LaunchConfiguration('marker_size', default='0.15')
    
    # ArUco Detector Node
    aruco_detector = Node(
        package='ubot_aruco',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'marker_size': marker_size,
            'dictionary_id': 0,  # DICT_4X4_50
            'camera_frame': 'camera_rgb_frame',
            'image_topic': '/camera/rgb/image_raw',
            'camera_info_topic': '/camera/rgb/camera_info',
        }]
    )

    # Patrol Navigator Node
    patrol_navigator = Node(
        package='ubot_aruco',
        executable='patrol_navigator.py',
        name='patrol_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'marker_sequence': [0, 1, 2, 3, 4],
            'approach_distance': 0.5,
            'search_angular_speed': 0.4,
            'approach_linear_speed': 0.25,
            'approach_angular_speed': 0.3,
            'loop': True,
            'pause_duration': 2.0,
            'camera_frame': 'camera_rgb_frame',
            'centering_threshold': 0.1,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'marker_size',
            default_value='0.15',
            description='ArUco marker size in meters'
        ),
        aruco_detector,
        patrol_navigator,
    ])
