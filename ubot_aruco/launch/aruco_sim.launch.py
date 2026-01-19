#!/usr/bin/env python3
"""
ArUco Simulation Launch File

Launches the full simulation with ArUco world:
- Gazebo with ArUco markers
- Robot spawn
- ros_gz_bridge
- ArUco detector
- Patrol navigator
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_description = get_package_share_directory('ubot_description')
    pkg_bringup = get_package_share_directory('ubot_bringup')
    pkg_aruco = get_package_share_directory('ubot_aruco')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot description with Gazebo enabled
    robot_description_config = Command([
        'xacro ',
        os.path.join(pkg_description, 'urdf', 'body', 'ubot_robot.urdf.xacro'),
        ' use_gazebo:=true'
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True
        }]
    )

    # Gazebo with ArUco world
    world_file = os.path.join(pkg_bringup, 'worlds', 'aruco_world.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        remappings=[
            ('/camera/image', '/camera/rgb/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
            ('/camera/camera_info', '/camera/rgb/camera_info'),
            ('/imu', '/imu/data')
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ubot',
            '-world', 'aruco_world',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
    )

    # Controller Spawners
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file',
                   os.path.join(pkg_bringup, 'config', 'ubot_controllers.yaml')],
        parameters=[{'use_sim_time': True}]
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--param-file',
                   os.path.join(pkg_bringup, 'config', 'ubot_controllers.yaml')],
        parameters=[{'use_sim_time': True}]
    )


    # ArUco Detector
    aruco_detector = Node(
        package='ubot_aruco',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'marker_size': 0.2,  # 20cm markers in sim
            'dictionary_id': 0,
            'camera_frame': 'camera_rgb_frame',
            'image_topic': '/camera/rgb/image_raw',
            'camera_info_topic': '/camera/rgb/camera_info',
        }]
    )

    # Patrol Navigator
    patrol_navigator = Node(
        package='ubot_aruco',
        executable='patrol_navigator.py',
        name='patrol_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'marker_sequence': [0, 1, 2, 3, 4],
            'approach_distance': 0.5,
            'search_angular_speed': 0.4,
            'approach_linear_speed': 0.25,
            'loop': True,
            'pause_duration': 2.0,
            'camera_frame': 'camera_rgb_frame',
        }]
    )

    # RViz
    rviz_config = os.path.join(pkg_description, 'rviz', 'slam.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo,
        bridge,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    joint_state_broadcaster,
                    diff_drive_controller,
                    aruco_detector,
                    patrol_navigator,
                ],
            )
        ),
        rviz,
    ])
