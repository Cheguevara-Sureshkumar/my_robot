import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('bot_spawn'))
    xacro_file = os.path.join(pkg_path, 'robot', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )

    use_joint_state_publisher = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='true',
        description='Use joint_state_publisher if true'
    )

    params = {'robot_description': robot_description, 'use_sim_time': LaunchConfiguration('use_sim_time')}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )


    '''node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )'''

    return LaunchDescription([
        use_sim_time,
        use_joint_state_publisher,
        robot_state_publisher,
        #node_joint_state_publisher,
        fake_odom
    ])