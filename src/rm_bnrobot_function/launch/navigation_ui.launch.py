from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('navigation_mode', default_value='prior', choices=['prior','explore']),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='rm_bnrobot_function', executable='nav_client_ui.py', name='nav_client_ui',
             output='screen', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'navigation_mode': LaunchConfiguration('navigation_mode')}]),
    ])
