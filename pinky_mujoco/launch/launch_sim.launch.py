"""MuJoCo counterpart of pinky_gz_sim/launch/launch_sim.launch.xml.

Starts the MuJoCo bridge and robot_state_publisher with use_sim_time.
Afterwards the existing simulation launches in pinky_navigation
(gz_map_building, gz_map_view, gz_bringup_launch, gz_nav2_view) work unchanged,
because the bridge publishes the same /clock, /odom, /tf, /scan, /joint_states.

By default the viewer window and the front camera are on but drawn with Mesa software
OpenGL (llvmpipe), so every PC behaves the same and no GPU is needed; shadows,
reflections and MSAA are dropped to keep that cheap. Physics never uses a GPU.
  use_gpu:=true        hardware OpenGL with full quality (needs a GPU driver)
  viewer:=false        no MuJoCo window
  camera:=false        no /camera/image_raw

World selection mirrors the Gazebo launch:
  world_name:=pinky_factory.world   (file in pinky_gz_sim/worlds, default)
  world_name:=pinky_map.world
  world_name:=room.yaml             (simple box list in pinky_mujoco/worlds)
  world:=/abs/path/to/file.world    (overrides world_name)
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_world(context):
    explicit = LaunchConfiguration('world').perform(context)
    if explicit:
        return Path(explicit).expanduser()
    name = LaunchConfiguration('world_name').perform(context)
    if name.endswith('.yaml'):
        return Path(get_package_share_directory('pinky_mujoco')) / 'worlds' / name
    return Path(get_package_share_directory('pinky_gz_sim')) / 'worlds' / name


def setup(context):
    from pinky_mujoco.description import expand_urdf

    mappings = {'cam_tilt_deg': LaunchConfiguration('cam_tilt_deg').perform(context),
                'screen_tilt_deg': LaunchConfiguration('screen_tilt_deg').perform(context)}
    urdf = expand_urdf(mappings=mappings)
    model_dir = Path(os.environ.get('PINKY_MODEL_DIR', str(Path.home() / '.cache/pinky_mujoco')))
    model_dir.mkdir(parents=True, exist_ok=True)
    urdf_path = model_dir / 'robot.urdf'
    urdf_path.write_text(urdf)
    world = resolve_world(context)
    if not world.is_file():
        raise FileNotFoundError(f'World file not found: {world}')

    use_gpu = LaunchConfiguration('use_gpu').perform(context).lower() == 'true'
    viewer = LaunchConfiguration('viewer').perform(context).lower() == 'true'
    camera = LaunchConfiguration('camera').perform(context).lower() == 'true'
    env = {}
    if not use_gpu:
        # Force Mesa's CPU rasterizer even when a vendor driver (e.g. NVIDIA) is installed.
        env = {'MUJOCO_GL': 'glfw', 'LIBGL_ALWAYS_SOFTWARE': '1', '__GLX_VENDOR_LIBRARY_NAME': 'mesa'}
    bridge_args = ['--urdf-file', str(urdf_path), '--output-dir', str(model_dir), '--world', str(world)]
    if viewer:
        bridge_args.append('--viewer')
    if not camera:
        bridge_args.append('--no-camera')
    if not use_gpu:
        bridge_args.append('--simple-render')
    print(f'[pinky_mujoco] world={world.name} viewer={viewer} camera={camera} '
          f"render={'hardware OpenGL (use_gpu:=true)' if use_gpu else 'software OpenGL, no GPU (use_gpu:=false)'}")

    return [
        Node(package='pinky_mujoco', executable='bridge', name='pinky_mujoco',
             arguments=bridge_args, output='screen', additional_env=env,
             parameters=[{'use_sim_time': True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_gpu', default_value='false',
                              description='true: hardware OpenGL with shadows/reflections. false: Mesa software rendering'),
        DeclareLaunchArgument('viewer', default_value='true', description='Open the MuJoCo viewer window'),
        DeclareLaunchArgument('camera', default_value='true', description='Publish /camera/image_raw'),
        DeclareLaunchArgument('world_name', default_value='pinky_factory.world',
                              description='File in pinky_gz_sim/worlds, or *.yaml in pinky_mujoco/worlds'),
        DeclareLaunchArgument('world', default_value='', description='Absolute world path (overrides world_name)'),
        DeclareLaunchArgument('cam_tilt_deg', default_value='8'),
        DeclareLaunchArgument('screen_tilt_deg', default_value='-25'),
        OpaqueFunction(function=setup),
    ])
