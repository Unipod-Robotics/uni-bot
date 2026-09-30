"""Start Gazebo Harmonic on a ubot benchmark world.

Renders worlds/<world>.sdf.xacro with the requested condition into a temp .sdf and starts
gz sim on it. Included by ubot_bringup/sim.launch.py and ubot_bench/bench_sim.launch.py.

Args:
  world      arena_5x5 | small_house | bookstore | small_warehouse
  condition  nominal | glass | dynamic | degraded | low_light
  headless   true -> server only (-s), no GUI
"""
import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

WORLDS = ('arena_5x5', 'small_house', 'bookstore', 'small_warehouse')
CONDITIONS = ('nominal', 'glass', 'dynamic', 'degraded', 'low_light')


def render_world(world, condition, physics='dartsim'):
    """Return the path of a rendered .sdf for world/condition (world name inside == world)."""
    if world not in WORLDS:
        raise RuntimeError(f'unknown world {world!r}; expected one of {WORLDS}')
    if condition not in CONDITIONS:
        raise RuntimeError(f'unknown condition {condition!r}; expected one of {CONDITIONS}')
    src = os.path.join(get_package_share_directory('ubot_worlds'), 'worlds', f'{world}.sdf.xacro')
    sdf = xacro.process_file(src, mappings={'condition': condition, 'physics': physics}
                             ).toprettyxml(indent='  ')
    out_dir = os.path.join(tempfile.gettempdir(), f'ubot_worlds_{os.getuid()}')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f'{world}__{condition}__{physics}.sdf')
    with open(out, 'w') as f:
        f.write(sdf)
    return out


def spawn_pose(world):
    """(x, y, yaw) spawn pose from missions/<world>.yaml."""
    import yaml
    path = os.path.join(get_package_share_directory('ubot_worlds'), 'missions', f'{world}.yaml')
    with open(path) as f:
        s = yaml.safe_load(f)['spawn']
    return float(s['x']), float(s['y']), float(s['yaw'])


def _launch(context):
    world = LaunchConfiguration('world').perform(context)
    condition = LaunchConfiguration('condition').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('1', 'true', 'yes')
    sdf = render_world(world, condition, LaunchConfiguration('physics').perform(context))
    flags = '-s -r -v 1' if headless else '-r -v 3'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{flags} {sdf}', 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='small_house', choices=WORLDS),
        DeclareLaunchArgument('condition', default_value='nominal', choices=CONDITIONS),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('physics', default_value='dartsim',
                              description='gz-physics engine: dartsim | bullet-featherstone'),
        OpaqueFunction(function=_launch),
    ])
