"""
Launches slam_toolbox in online-async mapping mode using our tuned params.

Run AFTER gazebo.launch.py is already up:
  ros2 launch augdd_navigation slam.launch.py

Then teleop the robot through the whole corridor + branch + room slowly,
covering every area at least once, ideally crossing your own path near the
T-junction once so slam_toolbox gets a loop closure.

Watch it build in RViz2:
  rviz2 -d $(ros2 pkg prefix augdd_navigation)/share/augdd_navigation/rviz/slam.rviz
(or just open rviz2 manually and add a Map display on topic /map, fixed
frame "map")

When the map looks complete and clean, save it (Step 3b, separate terminal):
  ros2 run nav2_map_server map_saver_cli -f ~/projects/ros2/augdd_ws/maps/augdd_map
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav_pkg = get_package_share_directory('augdd_navigation')
    params_file = os.path.join(nav_pkg, 'config', 'slam_toolbox_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
    ])
