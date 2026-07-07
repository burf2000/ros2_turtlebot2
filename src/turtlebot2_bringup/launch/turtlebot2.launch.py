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
    # DEFAULT OFF: the ldlidar_stl_ros2 SDK (below) is unreliable on this Jetson
    # Nano (times out / "communication abnormal" on the FTDI/PL2303, latency).
    # The REAL /scan comes from the custom raw-read node scripts/ld06_scan_node.py,
    # started by scripts/start_turtlebot2.sh (which also publishes the lidar TF).
    launch_lidar = LaunchConfiguration('launch_lidar', default='false')
    launch_depth_scan = LaunchConfiguration('launch_depth_scan', default='false')
    launch_robot_state_publisher = LaunchConfiguration('launch_robot_state_publisher', default='false')
    launch_point_cloud = LaunchConfiguration('launch_point_cloud', default='false')

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

    declare_launch_point_cloud = DeclareLaunchArgument(
        'launch_point_cloud',
        default_value='false',
        description='Launch point cloud generation (disable to save CPU and bandwidth)'
    )

    declare_launch_lidar = DeclareLaunchArgument(
        'launch_lidar',
        default_value='false',
        description='Launch the ldlidar_stl_ros2 SDK LiDAR node + its static TF. '
                    'DEFAULT OFF: the SDK is unreliable on this Nano; the working '
                    '/scan is the custom scripts/ld06_scan_node.py (raw FTDI read), '
                    'run by scripts/start_turtlebot2.sh which also publishes the '
                    'base_footprint->lidar_link TF slam_toolbox needs.'
    )

    declare_launch_depth_scan = DeclareLaunchArgument(
        'launch_depth_scan',
        default_value='false',
        description='Launch the LEGACY depth-image-derived /scan '
                    '(depthimage_to_laserscan). Off by default now the LD06 '
                    'owns /scan — never enable together with launch_lidar or '
                    'two publishers fight over /scan.'
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
        condition=IfCondition(launch_point_cloud),
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

    # Okdo LD06 LiDAR — the REAL /scan source (replaces the depth-derived scan).
    # Config (by-id serial port, port_baudrate 230400, frame lidar_link) in
    # config/ldlidar.yaml.
    ldlidar_node = Node(
        condition=IfCondition(launch_lidar),
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'ldlidar.yaml'),
            {'use_sim_time': use_sim_time}
        ],
    )

    # Static mount transform for the LD06: base_footprint -> lidar_link.
    # (Parented on base_footprint, not base_link, so the scan TF chain works
    # even if robot_state_publisher is off — odom_tf_bridge + slam both use
    # base_footprint.)
    # ⚠⚠ PLACEHOLDER — MEASURE ME ⚠⚠  Assumes the LD06 sits centred over the
    # base on the top plate, ~0.35 m off the floor, connector-forward (yaw 0).
    # Measure the real offsets (x fwd, y left, z up, metres; yaw rad) and fix
    # THIS line:
    lidar_tf_args = ['--x', '0', '--y', '0', '--z', '0.35', '--yaw', '0', '--pitch', '0', '--roll', '0']
    lidar_static_tf = Node(
        condition=IfCondition(launch_lidar),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        output='screen',
        arguments=lidar_tf_args + ['--frame-id', 'base_footprint', '--child-frame-id', 'lidar_link'],
    )

    # LEGACY: depth image -> virtual laser scan (pre-LD06). Kept behind
    # launch_depth_scan (default OFF) in case the LiDAR is ever removed.
    depthimage_to_laserscan_node = Node(
        condition=IfCondition(launch_depth_scan),
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
        declare_launch_lidar,
        declare_launch_depth_scan,
        declare_launch_robot_state_publisher,
        declare_launch_point_cloud,

        robot_state_publisher,
        kobuki_node,
        xtion_node,
        ldlidar_node,
        lidar_static_tf,
        point_cloud_container,
        depthimage_to_laserscan_node,
    ])
