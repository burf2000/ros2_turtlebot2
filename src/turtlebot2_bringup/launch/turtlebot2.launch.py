import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_bringup = get_package_share_directory('turtlebot2_bringup')
    pkg_description = get_package_share_directory('turtlebot2_description')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_kobuki = LaunchConfiguration('launch_kobuki', default='true')
    launch_camera = LaunchConfiguration('launch_camera', default='true')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_launch_kobuki = DeclareLaunchArgument(
        'launch_kobuki',
        default_value='true',
        description='Launch Kobuki base driver'
    )

    declare_launch_camera = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Launch Xtion camera'
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

    # ASUS Xtion Pro Camera using openni2_camera
    xtion_container = ComposableNodeContainer(
        condition=IfCondition(launch_camera),
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='openni2_camera',
                plugin='openni2_wrapper::OpenNI2Driver',
                name='camera',
                parameters=[
                    os.path.join(pkg_bringup, 'config', 'xtion.yaml'),
                    {'use_sim_time': use_sim_time}
                ],
                remappings=[
                    ('depth/image_raw', 'camera/depth/image_raw'),
                    ('depth/camera_info', 'camera/depth/camera_info'),
                    ('rgb/image_raw', 'camera/rgb/image_raw'),
                    ('rgb/camera_info', 'camera/rgb/camera_info'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_launch_kobuki,
        declare_launch_camera,

        robot_state_publisher,
        kobuki_node,
        xtion_container,
    ])
