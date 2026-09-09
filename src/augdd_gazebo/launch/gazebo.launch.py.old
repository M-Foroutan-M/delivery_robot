"""
Launches Gazebo Classic with the AUGDD hospital-corridor world, publishes
robot_description from the xacro, and spawns the robot into the sim.

Run:
  ros2 launch augdd_gazebo gazebo.launch.py

Then in another terminal, drive it manually to confirm everything moves:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
(install with: sudo apt install ros-humble-teleop-twist-keyboard)
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    desc_pkg = get_package_share_directory('augdd_description')
    gz_pkg = get_package_share_directory('augdd_gazebo')

    world_path = os.path.join(gz_pkg, 'worlds', 'hospital.world')
    xacro_path = os.path.join(desc_pkg, 'urdf', 'augdd.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    # Spawn the robot ~4.5m into the west end of the main corridor, facing +x
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'augdd',
            '-x', '-4.5', '-y', '0.0', '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
