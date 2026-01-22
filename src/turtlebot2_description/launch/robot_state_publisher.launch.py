import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_description = get_package_share_directory('turtlebot2_description')

    # URDF file path
    urdf_file = os.path.join(pkg_description, 'urdf', 'turtlebot2.urdf.xacro')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Process xacro to URDF with package path argument
    xacro_cmd = f'xacro {urdf_file} pkg_description:={pkg_description}'
    robot_description = Command([xacro_cmd])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }]
        ),

        # Joint State Publisher (publishes fixed joint states)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),
    ])
