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
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_driver = get_package_share_directory('burf_platform_driver')

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
        }.items()
    )

    return LaunchDescription([republish, driver])
