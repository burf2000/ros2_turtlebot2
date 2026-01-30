import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory('turtlebot2_bringup')

    # Launch configurations
    rviz_config = LaunchConfiguration('rviz_config')
    joy_dev = LaunchConfiguration('joy_dev')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_bringup, 'config', 'rviz', 'turtlebot2.rviz'),
        description='Path to RViz2 config file'
    )

    declare_joy_dev = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # Joystick driver
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        remappings=[
            ('joy', 'joy'),
        ],
    )

    # Teleop twist joy - converts joystick input to cmd_vel
    # PS2 controller mapping:
    #   Left stick vertical (axis 1) = linear forward/back
    #   Left stick horizontal (axis 0) = angular left/right
    #   L1 (button 4) = enable (deadman switch, normal speed)
    #   L2 (button 6) = enable turbo (deadman switch, turbo speed)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{
            # Axis mapping
            'axis_linear.x': 1,       # Left stick vertical
            'axis_angular.yaw': 0,    # Left stick horizontal

            # Normal speed (hold L1)
            'scale_linear.x': 0.2,    # 0.2 m/s
            'scale_angular.yaw': 1.0, # 1.0 rad/s

            # Turbo speed (hold L2)
            'scale_linear_turbo.x': 0.5,    # 0.5 m/s
            'scale_angular_turbo.yaw': 2.0, # 2.0 rad/s

            # Deadman switches
            'enable_button': 4,        # L1
            'enable_turbo_button': 6,  # L2

            'require_enable_button': True,
        }],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
        ],
    )

    return LaunchDescription([
        declare_rviz_config,
        declare_joy_dev,
        rviz_node,
        joy_node,
        teleop_node,
    ])
