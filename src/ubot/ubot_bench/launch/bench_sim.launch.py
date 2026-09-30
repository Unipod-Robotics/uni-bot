"""One benchmark trial's simulation stack (headless by default).

  ros2 launch ubot_bench bench_sim.launch.py world:=arena_5x5 stack:=MS200 condition:=nominal \
      seed:=1 phase:=mapping
  ros2 launch ubot_bench bench_sim.launch.py ... phase:=navigation map:=/path/to/map.yaml

Composes, identically for every stack except the range-sensor chain:
  Gazebo world (ubot_worlds, condition overlay) + ubot (sensor_profile of the stack, bench:=true)
  ros_gz bridge (clock, imu, the stack's raw range data) + controllers + EKF (sim_ekf.yaml)
  range chain:  LiDAR stacks  gpu_lidar /scan_raw -> scan_model -> /scan
                OAKD          depth image -> depth_to_scan (height band) -> /scan_raw
                              -> scan_model -> /scan
  gt_publisher (ground truth, never on TF)
  phase:=mapping     slam_toolbox (online async, mapping); the robot is driven by mapping_runner
  phase:=navigation  map_server + AMCL on the stack's own map, Nav2 (nav2_bench.yaml)

Condition 'degraded' (C3) sets scan_model dropout 0.20, outlier 0.02, noise x3.
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

from ubot_bench.profiles import STACKS, load_profiles

DEGRADED = {'dropout': 0.20, 'outlier': 0.02, 'noise_scale': 3.0}


def share(pkg, *p):
    return os.path.join(get_package_share_directory(pkg), *p)


def _gz_world():
    spec = importlib.util.spec_from_file_location(
        'gz_world', share('ubot_worlds', 'launch', 'gz_world.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    world, stack, condition = arg('world'), arg('stack'), arg('condition')
    seed, phase = int(arg('seed')), arg('phase')
    headless = arg('headless')
    if stack not in STACKS or STACKS[stack] is None:
        raise RuntimeError(f'stack {stack!r} has no range sensor; choose from '
                           f'{[k for k, v in STACKS.items() if v]}')
    profile_name = STACKS[stack]
    profile = load_profiles()[profile_name]
    is_depth = profile['kind'] == 'depth'
    x, y, yaw = _gz_world().spawn_pose(world)
    sim = {'use_sim_time': True}

    description = ParameterValue(Command([
        'xacro ', share('ubot_description', 'urdf', 'body', 'ubot_robot.urdf.xacro'),
        f' use_gazebo:=true bench:=true sensor_profile:={profile_name}']), value_type=str)

    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU']
    if is_depth:
        bridge_args += ['/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']
    else:
        bridge_args += ['/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']

    ctrl_yaml = share('ubot_bringup', 'config', 'sim_ubot_controllers.yaml')
    spawner = lambda name: Node(  # noqa: E731
        package='controller_manager', executable='spawner',
        arguments=[name, '--param-file', ctrl_yaml], parameters=[sim])
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'ubot', '-world', world,
                            '-x', str(x), '-y', str(y), '-z', '0.1', '-Y', str(yaw)])

    degr = DEGRADED if condition == 'degraded' else {'dropout': 0.0, 'outlier': 0.0,
                                                    'noise_scale': 1.0}
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share('ubot_worlds', 'launch', 'gz_world.launch.py')),
            launch_arguments={'world': world, 'condition': condition, 'headless': headless,
                              'physics': arg('physics')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, **sim}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridge_args,
             parameters=[sim]),
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[
            spawner('joint_state_broadcaster'), spawner('diff_drive_controller')])),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[share('ubot_bringup', 'config', 'sim_ekf.yaml')]),
        Node(package='twist_stamper', executable='twist_stamper',
             parameters=[{**sim, 'frame_id': 'base_footprint'}],
             remappings=[('/cmd_vel_in', '/cmd_vel'),
                         ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')]),
        Node(package='ubot_bench', executable='scan_model', name='scan_model',
             parameters=[{**sim, 'profile': profile_name, 'seed': seed, 'noise': True,
                          **degr}]),
        Node(package='ubot_bench', executable='gt_publisher', name='gt_publisher',
             parameters=[{**sim, 'world': world}]),
    ]
    if is_depth:
        half = 1.211 / 2 * 180 / 3.141592653589793      # sim RGB-D HFOV (ubot_gazebo.urdf.xacro)
        actions.append(Node(
            package='ubot_mono_nav', executable='depth_to_scan', name='depth_to_scan',
            parameters=[{**sim, 'base_frame': 'base_footprint',
                         'height_min': 0.05, 'height_max': 0.60,
                         'trusted_range': float(profile['range_max']),
                         'min_depth': float(profile['range_min']),
                         'range_min': float(profile['range_min']),
                         'angle_min_deg': -half, 'angle_max_deg': half,
                         'angle_increment_deg': 0.5, 'stride': 2, 'min_points': 2,
                         # camera_depth_frame is not an optical frame (see scan_geometry.py);
                         # use the measured mount, which equals the URDF camera pose.
                         'use_tf': False, 'fallback_pitch_up_deg': -4.09,
                         'fallback_xyz': [0.168, 0.0, 0.152]}],
            remappings=[('~/depth', '/camera/depth_image'),
                        ('~/depth/camera_info', '/camera/camera_info'),
                        ('~/scan', '/scan_raw')]))

    rng_max = float(profile['range_max'])
    if phase == 'mapping':
        slam_params = RewrittenYaml(
            source_file=share('ubot_bench', 'config', 'slam_bench.yaml'),
            param_rewrites={'max_laser_range': str(rng_max), 'use_sim_time': 'True'},
            convert_types=True)
        # slam_toolbox is a lifecycle node on Jazzy; its own launch file configures and
        # activates it (autostart).
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share('slam_toolbox', 'launch',
                                                'online_async_launch.py')),
            launch_arguments={'slam_params_file': slam_params, 'use_sim_time': 'true',
                              'autostart': 'true'}.items()))
    elif phase == 'navigation':
        map_yaml = arg('map')
        if not map_yaml or not os.path.exists(map_yaml):
            raise RuntimeError(f'navigation phase needs map:=<existing yaml>, got {map_yaml!r}')
        nav_params = RewrittenYaml(
            source_file=share('ubot_bench', 'config', 'nav2_bench.yaml'),
            param_rewrites={'laser_max_range': str(rng_max), 'yaml_filename': map_yaml,
                            'use_sim_time': 'True'},
            convert_types=True)
        nb = get_package_share_directory('nav2_bringup')
        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nb, 'launch',
                                                           'localization_launch.py')),
                launch_arguments={'map': map_yaml, 'use_sim_time': 'true',
                                  'params_file': nav_params, 'autostart': 'true'}.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nb, 'launch',
                                                           'navigation_launch.py')),
                launch_arguments={'use_sim_time': 'true', 'params_file': nav_params,
                                  'autostart': 'true'}.items()),
        ]
    else:
        raise RuntimeError(f'phase must be mapping or navigation, got {phase!r}')
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='arena_5x5'),
        DeclareLaunchArgument('stack', default_value='MS200',
                              description=f'one of {[k for k, v in STACKS.items() if v]}'),
        DeclareLaunchArgument('condition', default_value='nominal'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('phase', default_value='mapping'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('physics', default_value='dartsim'),
        OpaqueFunction(function=setup),
    ])
