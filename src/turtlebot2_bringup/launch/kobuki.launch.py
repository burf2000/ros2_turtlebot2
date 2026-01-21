import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory('turtlebot2_bringup')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    device_port = LaunchConfiguration('device_port', default='/dev/ttyUSB0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'device_port',
            default_value='/dev/ttyUSB0',
            description='Kobuki USB device port'
        ),

        # Kobuki Base Node
        Node(
            package='kobuki_node',
            executable='kobuki_ros_node',
            name='kobuki_node',
            output='screen',
            parameters=[
                os.path.join(pkg_bringup, 'config', 'kobuki.yaml'),
                {
                    'use_sim_time': use_sim_time,
                    'device_port': device_port,
                }
            ],
            remappings=[
                ('odom', 'odom'),
                ('cmd_vel', 'cmd_vel'),
                ('joint_states', 'joint_states'),
            ]
        ),
    ])
