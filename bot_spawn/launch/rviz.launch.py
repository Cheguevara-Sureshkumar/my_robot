import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'bot_spawn'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Configure the RViz node
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name), 
        'rviz', 
        'view_bot.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    # Add Differential Drive Controller Spawner
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output='screen'
    )

    # Add Joint Broadcaster Spawner
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output='screen'
    )

    camera_node = Node(
        package='image_transport',
        executable='republish',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/image_raw'),
            ('out', '/camera/image_compressed')
        ],
        parameters=[{'use_sim_time': True}]
    )

    lidar_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('cloud_in', '/scan'),
            ('scan', '/processed_scan')
        ]
    )

    # Launch them all
    return LaunchDescription([
        rsp,
        rviz_node,
        diff_drive_spawner,
        joint_broad_spawner,
        lidar_node,
        camera_node
    ])