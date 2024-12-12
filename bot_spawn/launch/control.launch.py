from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 
                 '/opt/ros/humble/lib/teleop_twist_keyboard/teleop_twist_keyboard',
                 'cmd_vel:=/diff_cont/cmd_vel_unstamped'],
            name='teleop',
            output='screen'
        )
    ])