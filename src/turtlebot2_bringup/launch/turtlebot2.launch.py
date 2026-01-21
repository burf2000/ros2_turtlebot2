import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_bringup = get_package_share_directory('turtlebot2_bringup')
    pkg_description = get_package_share_directory('turtlebot2_description')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')
    launch_kobuki = LaunchConfiguration('launch_kobuki', default='true')
    launch_camera = LaunchConfiguration('launch_camera', default='true')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    declare_launch_kobuki = DeclareLaunchArgument(
        'launch_kobuki',
        default_value='true',
        description='Launch Kobuki base driver'
    )

    declare_launch_camera = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Launch Astra camera'
    )

    # Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_description, '/launch/robot_state_publisher.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Kobuki Base Node
    # Note: This assumes kobuki_node is installed. If using kobuki_ros, adjust package name.
    kobuki_node = Node(
        condition=IfCondition(launch_kobuki),
        package='kobuki_node',
        executable='kobuki_ros_node',
        name='kobuki_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'kobuki.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odom', 'odom'),
            ('cmd_vel', 'cmd_vel'),
        ]
    )

    # Astra Camera Node
    # Note: Uses OrbbecSDK_ROS2 package
    astra_node = Node(
        condition=IfCondition(launch_camera),
        package='orbbec_camera',
        executable='orbbec_camera_node',
        name='camera',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'astra.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('color/image_raw', 'camera/rgb/image_raw'),
            ('depth/image_raw', 'camera/depth/image_raw'),
            ('depth/points', 'camera/depth/points'),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        declare_launch_kobuki,
        declare_launch_camera,

        robot_state_publisher,
        kobuki_node,
        astra_node,
    ])
