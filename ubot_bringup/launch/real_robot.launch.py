import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_description = get_package_share_directory('ubot_description')
    pkg_bringup = get_package_share_directory('ubot_bringup')

    robot_description_config = ParameterValue(
        Command([
            'xacro ',
            os.path.join(pkg_description, 'urdf', 'body', 'ubot_robot.urdf.xacro'),
            ' use_gazebo:=false'
        ]),
        value_type=str
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False
        }]
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_config},
            os.path.join(pkg_bringup, 'config', 'ubot_controllers.yaml'),
            {'use_sim_time': False}
        ],
        output='screen'
    )

    # Joint State Broadcaster — delayed 5s to allow hardware interface to configure
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            parameters=[{'use_sim_time': False}]
        )]
    )

    # Diff Drive Controller — delayed 6s to start after broadcaster
    diff_drive_controller_spawner = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            parameters=[{'use_sim_time': False}]
        )]
    )

    # RPLIDAR A1
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type':     'serial',
            'serial_port':      '/dev/ttyUSB0',
            'serial_baudrate':  115200,
            'frame_id':         'lidar_link',
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'Sensitivity',
        }]
    )

    # Twist Stamper — converts plain /cmd_vel to stamped /diff_drive_controller/cmd_vel
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'base_footprint'
        }],
        remappings=[
            ('/cmd_vel_in',  '/cmd_vel'),
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        twist_stamper,
        rplidar_node,
    ])
