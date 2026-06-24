"""burf.launch.py — TurtleBot2 on the Burf Platform.

Starts:
  1. image_transport republish: /camera/rgb/image_raw (raw) ->
     /camera/rgb/image_raw/compressed  (the driver needs the compressed topic;
     the openni2 camera only publishes raw).
  2. The burf_platform_driver, wired for this robot (robot_id turtlebot2-01,
     map relay + goto enabled). Camera/topic config comes from the driver's
     burf_driver.yaml; robot_id + enable flags are forced here because the
     driver launch's arg defaults would otherwise override the yaml.

Run SLAM first (slam.launch.py) so the map->base_footprint TF exists for goto.

Server-managed map LOAD (manage_slam:=true): instead of launching SLAM
separately, let the driver own the slam_toolbox lifecycle. The driver then
starts SLAM in mapping mode on boot and, on a server `cmd_map_load`, relaunches
it in localization mode pointed at the loaded posegraph — making "Load map"
fully server-triggered with no hand-typed CLI. When manage_slam is true, do NOT
also run slam.launch.py yourself; the driver does it. Default is false to keep
the existing "launch SLAM separately" flow working.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_driver = get_package_share_directory('burf_platform_driver')

    manage_slam = LaunchConfiguration('manage_slam')

    republish = Node(
        package='image_transport',
        executable='republish',
        name='rgb_compressed_republish',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/rgb/image_raw'),
            ('out/compressed', '/camera/rgb/image_raw/compressed'),
        ],
        output='screen',
    )

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_driver, 'launch', 'burf_driver.launch.py')
        ),
        launch_arguments={
            'robot_id': 'turtlebot2-01',
            'enable_map_relay': 'true',
            'enable_goto': 'true',
            # When true the driver owns slam_toolbox (starts mapping on boot,
            # relaunches in localization mode on cmd_map_load). Default false.
            'manage_slam': manage_slam,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'manage_slam',
            default_value='false',
            description="If true, the driver owns the slam_toolbox lifecycle so "
                        "the server can load maps (relaunch in localization "
                        "mode) without a hand-typed CLI. Don't also run "
                        "slam.launch.py yourself when this is true."
        ),
        republish,
        driver,
    ])
