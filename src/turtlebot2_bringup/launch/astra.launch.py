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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Orbbec Astra Camera Node
        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='camera',
            output='screen',
            parameters=[
                os.path.join(pkg_bringup, 'config', 'astra.yaml'),
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                # Remap to standard TurtleBot topic names
                ('color/image_raw', 'camera/rgb/image_raw'),
                ('color/camera_info', 'camera/rgb/camera_info'),
                ('depth/image_raw', 'camera/depth/image_raw'),
                ('depth/camera_info', 'camera/depth/camera_info'),
                ('depth/points', 'camera/depth/points'),
                ('ir/image_raw', 'camera/ir/image_raw'),
            ]
        ),
    ])
