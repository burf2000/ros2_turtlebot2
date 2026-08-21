"""burf.launch.py — TurtleBot2 on the Burf Platform.

Starts the burf_platform_driver, wired for this robot (robot_id turtlebot2-01,
map relay + goto enabled). Camera/topic config comes from the driver's
burf_driver.yaml; robot_id + enable flags are forced here because the driver
launch's arg defaults would otherwise override the yaml.

THE REPUBLISHER IS GONE, and that is the point of this file's history.

There used to be an `image_transport republish` here turning raw RGB into a
compressed topic for the driver. It subscribed to /camera/rgb/image_raw
UNCONDITIONALLY, and openni2 streams lazily off subscriber count — so the
republisher alone kept the Xtion streaming full time, for nobody.

That is not a CPU problem. The Nano has ample compute for a camera and a LiDAR;
measured with the camera off, core 0 was still 74% busy and /scan was perfect.
It is USB 2.0 BANDWIDTH: the Xtion and the LD06's 12 Mbps FTDI share one 480
Mbps bus, and a permanently streaming depth camera starved the LiDAR's bulk
endpoint to the point of corrupting bytes — 0 valid packets/s with 0 CRC
failures, /scan at 0.28 Hz with 15 second gaps.

So the driver now subscribes to RAW directly and only while a viewer is
connected. That removes the last unconditional subscriber, and also deletes a
JPEG encode here and a JPEG decode in the driver that cancelled each other out.

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
            # Raw, not compressed: see the note at the top. The driver
            # subscribes on demand, so nothing holds the camera open.
            'camera_topic': '/camera/rgb/image_raw',
            'use_compressed': 'false',
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
        driver,
    ])
