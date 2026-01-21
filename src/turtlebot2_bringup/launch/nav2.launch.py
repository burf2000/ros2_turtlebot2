import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory('turtlebot2_bringup')

    # Try to get nav2_bringup package
    try:
        pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    except Exception:
        pkg_nav2_bringup = None

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map', default='')
    params_file = LaunchConfiguration('params_file',
        default=os.path.join(pkg_bringup, 'config', 'nav2_params.yaml'))
    autostart = LaunchConfiguration('autostart', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_bringup, 'config', 'nav2_params.yaml'),
            description='Full path to Nav2 parameters file'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start Nav2 stack'
        ),

        # Nav2 Bringup (if available)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_nav2_bringup, '/launch/bringup_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_file,
                'params_file': params_file,
                'autostart': autostart,
            }.items()
        ) if pkg_nav2_bringup else Node(
            package='ros2cli',
            executable='ros2',
            arguments=['topic', 'echo', '/rosout'],
            output='screen'
        ),
    ])
