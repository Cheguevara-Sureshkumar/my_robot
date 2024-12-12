import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name='bot_world'

    try:
        # Log package directory for debugging
        pkg_dir = get_package_share_directory(package_name)
        print(f"Package directory: {pkg_dir}")
    except Exception as e:
        print(f"Error finding package directory: {e}")
        sys.exit(1)

    try:
        rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('bot_spawn'),'launch','rsp.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
        )
    except Exception as e:
        print(f"Error loading robot state publisher launch file: {e}")
        sys.exit(1)

    try:
        # Include the Gazebo launch file, provided by the gazebo_ros package
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')}.items(),
        )
    except Exception as e:
        print(f"Error loading Gazebo launch file: {e}")
        sys.exit(1)

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'bot_' + str(os.getpid())  # Use a unique entity name
        ],
        output='screen',
        on_exit=[LogInfo(msg='Spawn entity process completed')]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output='screen',
        on_exit=[LogInfo(msg='Diff drive controller spawned')]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output='screen',
        on_exit=[LogInfo(msg='Joint broad controller spawned')]
    )
    
    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
    ])