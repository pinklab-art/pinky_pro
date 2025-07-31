import os
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    namespace = DeclareLaunchArgument("namespace", default_value="")
    is_sim = DeclareLaunchArgument("is_sim", default_value="false")

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'ignore_timestamp': False,
            "use_sim_time": LaunchConfiguration('is_sim'),
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        get_package_share_directory('pinky_description'),
                        'urdf/robot.urdf.xacro',
                    ]),
                    ' namespace:=', LaunchConfiguration('namespace'),
                ]),
            'frame_prefix': [LaunchConfiguration('namespace'), '/'],
        }]
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            "source_list": ['joint_states'],
            "rate": 20.0,
            "use_sim_time": LaunchConfiguration('is_sim'),
        }],
        remappings=[
            ('/robot_descrption', 'robot_descrpition'),
        ],
        output='screen'
    )

    ld.add_action(namespace)
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)

    return ld