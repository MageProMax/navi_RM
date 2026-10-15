import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Combined launch file for robot control system
    Starts both can_twist_node and nav_plan_node
    """
    
    can_twist_node = Node(
        package='can_twist',
        executable='can_twist_node',
        name='can_twist_node',
        output='screen',
        emulate_tty=True,
    )
    
    nav_plan_node = Node(
        package='nav_plan',
        executable='nav_plan_node',
        name='nav_plan_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        can_twist_node,
        nav_plan_node,
    ])
