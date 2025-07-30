from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def static_lidar_link(context):
    nodes = []
    if LaunchConfiguration('namespace').perform(context) == "":
        nodes.append(
            Node(
                package='tf2_ros',
                name="static_lidar_tf_pub",
                executable='static_transform_publisher',
                arguments=[
                    '--frame-id', [LaunchConfiguration('namespace'), '/rplidar_link'],
                    '--child-frame-id', 'pinky/base_footprint/gpu_lidar',
                ],
                parameters=[{
                    "use_sim_time": LaunchConfiguration('is_sim')
                }],
                output='screen'
            )
        )
    else:
        nodes.append(
            Node(
                package='tf2_ros',
                name="static_lidar_tf_pub",
                executable='static_transform_publisher',
                arguments=[
                    '--frame-id', [LaunchConfiguration('namespace'), '/rplidar_link'],
                    '--child-frame-id', [LaunchConfiguration('namespace'), '/base_footprint/gpu_lidar'],
                ],
                parameters=[{
                    "use_sim_time": LaunchConfiguration('is_sim')
                }],
                output='screen'
            )
        )

    return nodes

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    namespace_launch_arg = DeclareLaunchArgument("namespace", default_value="")
    world = DeclareLaunchArgument("world", default_value="default.sdf")

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('pinky_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
            'is_sim': "true",
            'sim_type': "gz_sim",
            'namespace': LaunchConfiguration('namespace'),
        }.items()
    )

    bringup_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('pinky_gz_sim'),
            '/launch/bringup_gz_sim.launch.py']
        ),
        launch_arguments = {
            'world' : LaunchConfiguration('world')
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=LaunchConfiguration('namespace'),
        output='both',
        arguments=[
            '-name', LaunchConfiguration('namespace'),
            '-topic', 'robot_description',
            '-allow_renaming', 'true'
            '-x', '0.0',
            '-y', '0.0',
        ],
        parameters=[{
            "use_sim_time": LaunchConfiguration('is_sim')
        }],
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output='screen',
        namespace=LaunchConfiguration('namespace'),
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )

    load_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('namespace'),
        arguments=["base_controller", "--controller-manager", "controller_manager"],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        name="ros_gz_bridge_sensors",
        executable='parameter_bridge',
        arguments=[
            [LaunchConfiguration('namespace'), '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        ],
        output='screen'
    )

    # ros_gz_image_bridge = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     arguments=[
    #         [LaunchConfiguration('namespace'), '/camera/image'],
    #         [LaunchConfiguration('namespace'), '/camera/depth_image'],
    #     ],
    #     output='screen'
    # )

    return LaunchDescription([
        namespace_launch_arg,
        world,
        upload_robot,
        bringup_gz_sim,
        spawn_robot,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[
                    LogInfo(msg='joint state broadcaster spawned, spawn base_controller'),
                    load_base_controller,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_base_controller,
                on_exit=[
                    LogInfo(msg='base_controller spawned, spawn gz_bridge'),
                    ros_gz_bridge,
                    # ros_gz_image_bridge,
                    OpaqueFunction(function=static_lidar_link)
                ]
            )
        ),
    ])