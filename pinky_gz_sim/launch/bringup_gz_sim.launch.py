import os
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_resource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    world_name = DeclareLaunchArgument("world_name", default_value="default.sdf")

    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '1'
    resource_path = get_package_share_directory('pinky_description')
    resource_path = resource_path[:len(resource_path) - len('pinky_description')]

    plugin_path = get_package_share_directory('pinky_gz_sim')
    plugin_path = plugin_path[:len(plugin_path) - len('pinky_gz_sim')]

    model_path = get_package_share_directory('pinky_gz_sim')
    environ['GZ_SIM_RESOURCE_PATH'] =  \
        "$GZ_SIM_RESOURCE_PATH"  \
        + pathsep + resource_path \
        + pathsep + plugin_path \
        + pathsep + model_path + "/models"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            "gz_args": [
                '-r -v3',
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('pinky_gz_sim'),
                    "worlds",
                    LaunchConfiguration('world_name')]
                )
            ]
        }.items()
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        name="ros_gz_bridge_clock",
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        world_name,
        gz_bridge,
        gz_sim,
    ])