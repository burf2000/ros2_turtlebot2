import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory('turtlebot2_bringup')

    # Try to get slam_toolbox package
    try:
        pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    except Exception:
        pkg_slam_toolbox = None

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration('slam_params_file',
        default=os.path.join(pkg_bringup, 'config', 'slam_params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_bringup, 'config', 'slam_params.yaml'),
            description='Full path to SLAM parameters file'
        ),

        # SLAM Toolbox (async mode for online mapping)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_slam_toolbox, '/launch/online_async_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
            }.items()
        ) if pkg_slam_toolbox else Node(
            package='ros2cli',
            executable='ros2',
            arguments=['topic', 'list'],
            output='screen'
        ),
    ])
