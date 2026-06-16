from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the config file
    pkg_share = get_package_share_directory('burf_platform_driver')
    config_file = os.path.join(pkg_share, 'config', 'burf_driver.yaml')

    # Declare launch arguments
    server_url_arg = DeclareLaunchArgument(
        'server_url',
        default_value='wss://platform.burf.co/ws/robot',
        description='WebSocket server URL'
    )

    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='ros2-rover-01',
        description='Unique robot identifier'
    )

    local_arg = DeclareLaunchArgument(
        'local',
        default_value='false',
        description='Use local development server (ws://192.168.0.50:8081/ws/robot)'
    )

    enable_map_relay_arg = DeclareLaunchArgument(
        'enable_map_relay',
        default_value='false',
        description='Subscribe to /map and relay as map_update messages. Only set '
                    'true on robots actually running SLAM. The browser shows its '
                    'map panel only when the driver advertises the "map" '
                    'capability — which requires this flag to be true.'
    )

    enable_goto_arg = DeclareLaunchArgument(
        'enable_goto',
        default_value='false',
        description='Wire up the on-robot single-waypoint controller. Driver '
                    'subscribes to cmd_goal / cmd_cancel_goal messages and ticks '
                    'GoToController at 10 Hz, publishing /cmd_vel to drive toward '
                    'the goal. The browser shows the click-to-waypoint UI only '
                    'when the driver advertises the "goto" capability — which '
                    'requires this flag (and a usable map->base_link TF, '
                    'typically meaning SLAM is also running).'
    )

    # Create the node
    burf_driver_node = Node(
        package='burf_platform_driver',
        executable='burf_driver',
        name='burf_driver',
        output='screen',
        parameters=[
            config_file,
            {
                'server_url': LaunchConfiguration('server_url'),
                'robot_id': LaunchConfiguration('robot_id'),
                'local': LaunchConfiguration('local'),
                'enable_map_relay': LaunchConfiguration('enable_map_relay'),
                'enable_goto': LaunchConfiguration('enable_goto'),
            }
        ],
        remappings=[
            # Remap topics if needed
            # ('/camera/image_raw', '/your_camera_topic'),
        ]
    )

    return LaunchDescription([
        server_url_arg,
        robot_id_arg,
        local_arg,
        enable_map_relay_arg,
        enable_goto_arg,
        burf_driver_node
    ])
