import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

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

    # # RPLIDAR A1
    # rplidar_node = Node(
    #     package='sllidar_ros2',
    #     executable='sllidar_node',
    #     name='sllidar_node',
    #     output='screen',
    #     parameters=[{
    #         'channel_type':     'serial',
    #         'serial_port':      '/dev/ttyUSB1',
    #         'serial_baudrate':  115200,
    #         'frame_id':         'lidar_link',
    #         'inverted':         False,
    #         'angle_compensate': True,
    #         'scan_mode':        'Sensitivity',
    #     }]
    # )

    # LIDAR
    lidar_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',
        name='MS200',
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': 'lidar_link'},
            {'scan_topic': '/scan'},
            {'port_name': '/dev/ttyACM0'},
            {'baudrate': 230400},
            {'angle_min': 0.0},
            {'angle_max': 360.0},
            {'range_min': 0.1},
            {'range_max': 20.0},
            {'clockwise': False},
            {'motor_speed': 10},
        ]
    )

    # ldlidar_node = Node(
    #     package='ldlidar_stl_ros2',
    #     executable='ldlidar_stl_ros2_node',
    #     name='ldlidar_node',
    #     output='screen',
    #     parameters=[{
    #         'product_name':           'LDLiDAR_LD19',
    #         'topic_name':             'scan',
    #         'frame_id':               'lidar_link',
    #         'port_name':              '/dev/ttyUSB1',
    #         'port_baudrate':          230400,
    #         'laser_scan_dir':         True,
    #         'enable_angle_crop_func': False,
    #         'angle_crop_min':         0.0,
    #         'angle_crop_max':         0.0,
    #     }]
    # )

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


    # EKF parameter file in ubot_bringup/config:
    #   real_ekf.yaml          wheel odometry + IMU yaw rate (default)
    #   real_ekf_imu_yaw.yaml  wheel velocities + IMU absolute yaw (magnetometer) and yaw rate
    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value='real_ekf.yaml',
        description='EKF parameter file in ubot_bringup/config'
    )

    # BNO085 IMU — I2C, publishes /imu with frame_id imu_link
    bno085_node = Node(
        package='bno08x_driver',
        executable='bno08x_driver',
        name='bno08x_driver',
        output='screen',
        parameters=[os.path.join(pkg_bringup, 'config', 'bno085_params.yaml')]
    )

    # EKF — fuses wheel odometry with the IMU into /odometry/filtered and publishes
    # odom -> base_footprint (the diff drive controller's odom TF is disabled)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('ekf_config')])]
    )

    oak_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("depthai_ros_driver_v3"),
                "launch",
                "driver.launch.py",
            ])
        ),
        launch_arguments={
            "name": "oak",
            "parent_frame": "oak_camera_link",

            "cam_pos_x": "0.0",
            "cam_pos_y": "0.0",
            "cam_pos_z": "0.0",

            "cam_roll": "0.0",
            "cam_pitch": "0.0",
            "cam_yaw": "0.0",

            "publish_tf_from_calibration": "true",
            "pointcloud.enable": "true",

            "use_rviz": "false",
        }.items(),
    )

    return LaunchDescription([
        ekf_config_arg,
        node_robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        twist_stamper,
        lidar_node,
        # diag_publisher,
        bno085_node,
        ekf_node,
    ])
