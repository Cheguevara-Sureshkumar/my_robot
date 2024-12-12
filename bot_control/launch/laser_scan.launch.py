import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('bot_control'),
        'config',
        'laser_view.rviz'
    )

    return LaunchDescription([
        # Laser Scan Filter Node
        Node(
            package='bot_control',
            executable='reading_laser',
            name='laser_scan_filter'
        ),
        
        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}]
        )
    ])