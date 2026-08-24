"""nav2.launch.py — Nav2 for the TurtleBot2, on top of slam_toolbox.

WHAT THIS LAUNCHES, AND WHAT IT DELIBERATELY DOES NOT
-----------------------------------------------------
This includes nav2_bringup's **navigation_launch.py**, NOT its
bringup_launch.py. The difference matters:

  bringup_launch.py  = map_server + AMCL + the navigation servers
  navigation_launch.py =                     the navigation servers only
                         (controller, smoother, planner, behaviors,
                          bt_navigator, waypoint_follower, velocity_smoother,
                          lifecycle_manager_navigation)

On this robot **slam_toolbox already owns /map and the map->odom transform**
(scripts/start_slam_burf.sh launches slam.launch.py first, in mapping or
localization mode). Adding Nav2's map_server and AMCL would give you two
publishers on /map and two things asserting map->odom — the classic "the robot
teleports / the costmap is garbage" failure. So: SLAM localizes, Nav2 drives.

That is also why this file has no `map:=` argument. There is nothing to load;
the map arrives live from slam_toolbox and the global costmap's static_layer
picks it up over a transient_local subscription.

CMD_VEL OWNERSHIP — READ THIS BEFORE TOUCHING THE TOGGLE
-------------------------------------------------------
Nav2's velocity_smoother is remapped so the chain ends
`controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel`, i.e. Nav2
publishes /cmd_vel at ~20 Hz and owns the Kobuki base while a goal is running.

burf_platform_driver normally publishes its own 10 Hz /cmd_vel heartbeat — a
deadman that repeats the last command so the base halts if the link drops. Two
publishers on /cmd_vel fight and the robot stutters; it reads as "Nav2 is
broken" when it is really a topic fight. The driver therefore has an
`enable_nav2` parameter which stands the heartbeat down.

The inverse is the DANGEROUS case: Nav2 not running but the driver told
`enable_nav2:=true` gives you no Nav2 and no deadman. So Nav2 and the driver's
enable_nav2 must never be set independently — both are driven from the single
flag file /etc/turtlebot2/nav2.enabled, read once by scripts/start_slam_burf.sh.
Flip it with scripts/nav2_toggle.sh. Do not launch this file by hand unless you
have also dealt with the heartbeat.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory('turtlebot2_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_params = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the Nav2 parameters file. The bundled '
                        'config/nav2_params.yaml is tuned for the Kobuki base '
                        '(base_footprint, robot_radius 0.22, /scan, '
                        'max_vel_x 0.26).',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Let lifecycle_manager_navigation configure+activate '
                        'the stack automatically. On this Nano the whole stack '
                        'reaches active in roughly 20-30 s.',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Respawn individual Nav2 servers if they crash.',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': LaunchConfiguration('autostart'),
                'use_respawn': LaunchConfiguration('use_respawn'),
                # Composition off: the composed container hides which server
                # died, and on a 4-core Nano the win is not worth the loss of
                # per-node logs.
                'use_composition': 'False',
            }.items(),
        ),
    ])
