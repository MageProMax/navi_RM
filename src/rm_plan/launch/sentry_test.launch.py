from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_plan',
            executable='fake_referee',
            name='fake_referee',
            output='screen'
        ),
        Node(
            package='rm_plan',
            executable='decision_node',
            name='decision_node',
            output='screen'
        ),
        Node(
            package='rm_plan',
            executable='sentry_info_receiver',
            name='sentry_info_receiver',
            output='screen'
        ),
    ])
 
