import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
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
    launch_robot_state_publisher = LaunchConfiguration('launch_robot_state_publisher', default='false')

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

    declare_launch_robot_state_publisher = DeclareLaunchArgument(
        'launch_robot_state_publisher',
        default_value='true',
        description='Launch robot state publisher (requires xacro)'
    )

    # Robot State Publisher (optional, requires xacro)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_description, '/launch/robot_state_publisher.launch.py'
        ]),
        condition=IfCondition(launch_robot_state_publisher),
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
            ('commands/velocity', 'cmd_vel'),
            ('sensors/imu_data', 'imu'),
            ('sensors/imu_data_raw', 'imu/raw'),
        ]
    )

    # ASUS Xtion Pro Camera using openni2_camera (standalone node)
    # Note: Using standalone node instead of ComposableNodeContainer
    # as the composable approach has USB timeout issues on Jetson Nano
    xtion_node = Node(
        condition=IfCondition(launch_camera),
        package='openni2_camera',
        executable='openni2_camera_driver',
        name='camera',
        output='screen',
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
    )

    # Point cloud generation from depth + RGB using depth_image_proc
    # (ROS2 openni2_camera does not publish point clouds natively like ROS1)
    point_cloud_container = ComposableNodeContainer(
        condition=IfCondition(launch_camera),
        name='point_cloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb',
                remappings=[
                    ('rgb/camera_info', 'camera/rgb/camera_info'),
                    ('rgb/image_rect_color', 'camera/rgb/image_raw'),
                    ('depth_registered/image_rect', 'camera/depth/image_raw'),
                    ('points', 'camera/depth/points'),
                ],
            ),
        ],
        output='screen',
    )

    # Convert depth image to laser scan for Nav2/SLAM
    # (TurtleBot 2 has no lidar, so we generate a virtual scan from the Xtion)
    depthimage_to_laserscan_node = Node(
        condition=IfCondition(launch_camera),
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'scan_height': 1,
            'scan_time': 0.033,
            'range_min': 0.45,
            'range_max': 8.0,
            'output_frame_id': 'camera_depth_optical_frame',
        }],
        remappings=[
            ('depth', 'camera/depth/image_raw'),
            ('depth_camera_info', 'camera/depth/camera_info'),
            ('scan', 'scan'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_launch_kobuki,
        declare_launch_camera,
        declare_launch_robot_state_publisher,

        robot_state_publisher,
        kobuki_node,
        xtion_node,
        point_cloud_container,
        depthimage_to_laserscan_node,
    ])
