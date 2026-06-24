import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _launch_setup(context, *args, **kwargs):
    """Resolve launch configs at runtime so we can branch on slam_mode and,
    in localization mode, rewrite the params file with the requested
    map_file_name before handing it to slam_toolbox."""
    pkg_bringup = get_package_share_directory('turtlebot2_bringup')

    try:
        pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    except Exception:
        pkg_slam_toolbox = None

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    slam_mode = LaunchConfiguration('slam_mode').perform(context).strip().lower()
    map_file_name = LaunchConfiguration('map_file_name').perform(context).strip()
    slam_params_file = LaunchConfiguration('slam_params_file').perform(context)

    # Pick the params file by mode unless the caller passed an explicit override.
    # (slam_params_file defaults to the empty string; non-empty means "use this".)
    if not slam_params_file:
        if slam_mode == 'localization':
            slam_params_file = os.path.join(
                pkg_bringup, 'config', 'slam_params_localization.yaml')
        else:
            slam_params_file = os.path.join(
                pkg_bringup, 'config', 'slam_params.yaml')

    # In localization mode, if a map_file_name was supplied, rewrite the params
    # file's `map_file_name:` line into a temp copy so the SAME localization yaml
    # serves any saved map without editing it on disk. slam_toolbox reads
    # map_file_name from its params, not from a launch arg, so this is the
    # dependency-free way to make it per-map. (No extension — slam_toolbox adds
    # .posegraph/.data.)
    if slam_mode == 'localization' and map_file_name:
        with open(slam_params_file, 'r') as f:
            lines = f.readlines()
        out_lines = []
        replaced = False
        for line in lines:
            stripped = line.lstrip()
            indent = line[:len(line) - len(stripped)]
            if stripped.startswith('map_file_name:'):
                out_lines.append(f'{indent}map_file_name: {map_file_name}\n')
                replaced = True
            else:
                out_lines.append(line)
        if not replaced:
            # No map_file_name key in the file — append one under ros__parameters.
            # Best-effort: insert after the `mode:` line.
            tmp = []
            for line in out_lines:
                tmp.append(line)
                if line.lstrip().startswith('mode:'):
                    indent = line[:len(line) - len(line.lstrip())]
                    tmp.append(f'{indent}map_file_name: {map_file_name}\n')
            out_lines = tmp
        fd, tmp_path = tempfile.mkstemp(
            prefix='slam_params_loc_', suffix='.yaml')
        with os.fdopen(fd, 'w') as f:
            f.writelines(out_lines)
        slam_params_file = tmp_path

    nodes = []
    if pkg_slam_toolbox:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_slam_toolbox, '/launch/online_async_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
            }.items()
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        # 'mapping' (default) | 'localization'. In localization mode the launch
        # picks slam_params_localization.yaml and (if map_file_name is given)
        # rewrites it to point at the requested saved posegraph.
        DeclareLaunchArgument(
            'slam_mode',
            default_value='mapping',
            description="SLAM mode: 'mapping' (build a new map) or "
                        "'localization' (relocalize against a saved posegraph)."
        ),
        # Saved map basename WITHOUT extension, e.g.
        # /home/burf2000/maps/ground-floor . Only used in localization mode.
        DeclareLaunchArgument(
            'map_file_name',
            default_value='',
            description='Path (no extension) to the serialized posegraph to '
                        'load in localization mode. slam_toolbox appends '
                        '.posegraph/.data.'
        ),
        # Explicit params-file override. Empty => auto-selected from slam_mode.
        DeclareLaunchArgument(
            'slam_params_file',
            default_value='',
            description='Full path to SLAM parameters file. Empty selects '
                        'mapping/localization params from slam_mode.'
        ),

        # Kobuki publishes /odom but NOT the odom->base_footprint TF (even with
        # publish_tf:true), so slam_toolbox drops every scan. This bridge supplies
        # that transform (now-stamped). MUST run with SLAM in either mode.
        Node(
            package='turtlebot2_bringup',
            executable='odom_tf_bridge.py',
            name='odom_tf_bridge',
            output='screen',
        ),

        OpaqueFunction(function=_launch_setup),
    ])
