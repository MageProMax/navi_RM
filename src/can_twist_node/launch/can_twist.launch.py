import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for can_twist_node
    Starts the CAN motor control and referee system message handler
    """
    
    can_twist_node = Node(
        package='can_twist',
        executable='can_twist_node',
        name='can_twist_node',
        output='screen',
        emulate_tty=True,
        parameters=[],
    )

    return LaunchDescription([
        can_twist_node,
    ])
