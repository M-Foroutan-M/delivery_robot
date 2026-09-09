"""
Brings up Nav2 (map_server + amcl + planner + controller + bt_navigator +
behavior_server + waypoint_follower + velocity_smoother + lifecycle
manager) using nav2_bringup's standard bringup_launch.py, pointed at our
saved map and one of our two controller configs.

Run AFTER gazebo.launch.py is already up (SLAM/teleop no longer needed --
we now localize against the saved map with AMCL instead of building a new
one).

Usage:
  ros2 launch augdd_navigation navigation.launch.py \
      map:=/absolute/path/to/augdd_map.yaml \
      params_file:=/absolute/path/to/nav2_params_dwb.yaml

  # or swap the last arg to nav2_params_rpp.yaml for the other controller
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav_pkg = get_package_share_directory('augdd_navigation')

    default_map = os.path.expanduser('~/projects/ros2/augdd_ws/maps/augdd_map.yaml')
    default_params = os.path.join(nav_pkg, 'config', 'nav2_params_dwb.yaml')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map,
                               description='Full path to the saved map yaml'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                               description='Full path to nav2 params (dwb or rpp variant)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items()
        ),
    ])
