import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('ubot_description')
    
    # 1. Process URDF with use_gazebo:=true
    robot_description_config = Command([
        'xacro ', 
        os.path.join(pkg_description, 'urdf', 'body', 'ubot_robot.urdf.xacro'), 
        ' use_gazebo:=true'
    ])
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config, 
            'use_sim_time': True
        }]
    )
    
    # 2. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r -v 4 sensors.sdf'}.items(),
    )
    
    # 3. ROS-Gazebo Bridge (Clock, Cmd_vel, Odom, TF, Lidar, Camera)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            
            # FIXED RGB-D BRIDGE MAPPING
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/camera/image', '/camera/rgb/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
            ('/camera/camera_info', '/camera/rgb/camera_info'),
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 4. Spawn Robot Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'ubot', 
            '-z', '0.1'
        ],
    )
    
    # 5. Controller Spawners
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", 
            "--param-file", 
            os.path.join(get_package_share_directory('ubot_bringup'), 
                        'config', 'ubot_controllers.yaml')
        ],
        parameters=[{'use_sim_time': True}]
    )
    
    load_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller", 
            "--param-file", 
            os.path.join(get_package_share_directory('ubot_bringup'), 
                        'config', 'ubot_controllers.yaml')
        ],
        parameters=[{'use_sim_time': True}]
    )
    
    # 6. Twist Stamper (for teleop compatibility)
    node_twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_footprint'
        }],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        bridge,
        spawn_entity,
        node_twist_stamper,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster, load_diff_drive_controller],
            )
        ),
    ])

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command
# from launch_ros.actions import Node

# def generate_launch_description():
#     pkg_description = get_package_share_directory('ubot_description')
    
#     # 1. Process URDF with use_gazebo:=true
#     robot_description_config = Command([
#         'xacro ', 
#         os.path.join(pkg_description, 'urdf', 'body', 'ubot_robot.urdf.xacro'), 
#         ' use_gazebo:=true'
#     ])
    
#     node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'robot_description': robot_description_config, 
#             'use_sim_time': True
#         }]
#     )
    
#     # 2. Gazebo
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
#         launch_arguments={'gz_args': '-r -v 4 sensors.sdf'}.items(),
#     )
    
#     # 3. ROS-Gazebo Bridge (Clock, Cmd_vel, Odom, TF, Lidar, Camera)
#     bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         arguments=[
#             '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
#             '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
#             '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
#             '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
#             '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            
#             # EXACT RGB-D BRIDGE MAPPING FOR HARMONIC
#             '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
#             '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
#             '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
#             '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
#         ],
#         output='screen',
#         parameters=[{'use_sim_time': True}]
#     )
    
#     # 4. Spawn Robot Entity
#     spawn_entity = Node(
#         package='ros_gz_sim',
#         executable='create',
#         output='screen',
#         arguments=[
#             '-topic', 'robot_description', 
#             '-name', 'ubot', 
#             '-z', '0.1'
#         ],
#     )
    
#     # 5. Controller Spawners
#     load_joint_state_broadcaster = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "joint_state_broadcaster", 
#             "--param-file", 
#             os.path.join(get_package_share_directory('ubot_bringup'), 
#                         'config', 'ubot_controllers.yaml')
#         ],
#         parameters=[{'use_sim_time': True}]
#     )
    
#     load_diff_drive_controller = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "diff_drive_controller", 
#             "--param-file", 
#             os.path.join(get_package_share_directory('ubot_bringup'), 
#                         'config', 'ubot_controllers.yaml')
#         ],
#         parameters=[{'use_sim_time': True}]
#     )
    
#     # 6. Twist Stamper (for teleop compatibility)
#     node_twist_stamper = Node(
#         package='twist_stamper',
#         executable='twist_stamper',
#         parameters=[{
#             'use_sim_time': True,
#             'frame_id': 'base_footprint'
#         }],
#         remappings=[
#             ('/cmd_vel_in', '/cmd_vel'),
#             ('/cmd_vel_out', '/diff_drive_controller/cmd_vel'),
#         ]
#     )
    
#     return LaunchDescription([
#         node_robot_state_publisher,
#         gazebo,
#         bridge,
#         spawn_entity,
#         node_twist_stamper,
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=spawn_entity,
#                 on_exit=[load_joint_state_broadcaster, load_diff_drive_controller],
#             )
#         ),
#     ])


