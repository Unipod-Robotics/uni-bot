import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_description = get_package_share_directory('ubot_description')
    pkg_bringup = get_package_share_directory('ubot_bringup')
    
    # Process URDF with use_gazebo:=false (real hardware)
    robot_description_config = ParameterValue(
        Command([
            'xacro ',
            os.path.join(pkg_description, 'urdf', 'body', 'ubot_robot.urdf.xacro'),
            ' use_gazebo:=false'
        ]),
        value_type=str
    )
    
    # Robot State Publisher - NO sim_time
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False  # Real hardware uses wall time
        }]
    )
    
    # Controller Manager - NO sim_time
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
    
    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': False}]
    )
    
    # Diff Drive Controller Spawner
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        parameters=[{'use_sim_time': False}]
    )

    # IMU Sensor Broadcaster Spawner
    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster'],
        parameters=[{'use_sim_time': False}]
    )
    
    # Twist Stamper (for teleop)
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'base_footprint'
        }],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        node_robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        imu_sensor_broadcaster_spawner,
        twist_stamper,
    ])