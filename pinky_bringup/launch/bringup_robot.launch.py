from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="robot namespace",
        )
    )
    namespace = LaunchConfiguration("namespace")

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('pinky_bringup'),
        "config",
        "pinky_controllers.yaml"
    ])

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('pinky_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
            'namespace': LaunchConfiguration('namespace'),
        }.items()
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            ParameterFile(
                RewrittenYaml(
                    source_file=robot_controllers,
                    param_rewrites={},
                    root_key=LaunchConfiguration('namespace')
                ),
                allow_substs=True
            ),
        ],
        output="both",
        respawn=False,
        remappings=[
            ('~/robot_description',  'robot_description'),
            ('base_controller/cmd_vel', 'cmd_vel'),
            ('base_controller/odom', 'odom'),
        ]
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

    load_gpio_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('namespace'),
        arguments=["gpio_controller", "--controller-manager", "controller_manager"],
    )

    delay_gpio_after_base_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_base_controller,
            on_exit=[load_gpio_controller],
        )
    )

    delay_joint_state_broadcaster_after_gpio_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_gpio_controller,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    imu_node = Node(
        package="pinky_imu_bno055",
        executable="main_node",
        namespace=LaunchConfiguration('namespace'),
        respawn=True,
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[{
            "frame_id": "imu_link",
            "interface": "/dev/i2c-0",
            "rate": 100.0
        }]
    )

    sensor_adc_node = Node(
        package="pinky_sensor_adc",
        executable="main_node",
        namespace=LaunchConfiguration('namespace'),
        respawn=True,
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[{
            "interface": "/dev/i2c-1",
            "rate": 20.0
        }]
    )

    lamp_control_node = Node(
        package="pinky_lamp_control",
        executable="main_node",
        namespace=LaunchConfiguration('namespace'),
        respawn=True,
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[{
        }]
    )

    screen_control_node = Node(
        package="pinky_screen_control",
        executable="main_node",
        namespace=LaunchConfiguration('namespace'),
        respawn=True,
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[{
        }]
    )

    rplidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('sllidar_ros2'),
            '/launch/sllidar_c1_launch.py']
        ),
        launch_arguments = {
            'serial_port': "/dev/ttyAMA0",
            'frame_id': "rplidar_link",
        }.items()
    )

    nodes = [
        upload_robot,
        controller_manager,
        load_base_controller,
        delay_gpio_after_base_controller_spawner,
        delay_joint_state_broadcaster_after_gpio_controller_spawner,
        imu_node,
        sensor_adc_node,
        lamp_control_node,
        screen_control_node,
        rplidar_bringup,
    ]

    return LaunchDescription(declared_arguments + nodes)

