"""View the saved 2-D/3-D map pair without robot or navigation nodes."""
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    folder = Path(__file__).resolve().parent
    return LaunchDescription([
        Node(
            package='pcl_ros', executable='pcd_to_pointcloud',
            namespace='map_preview', name='pcd_publisher', output='screen',
            parameters=[{'file_name': str(folder / 'map.pcd'),
                         'tf_frame': 'map', 'publishing_period_ms': 2000}],
        ),
        Node(
            package='nav2_map_server', executable='map_server',
            namespace='map_preview', name='map_server', output='screen',
            parameters=[{'yaml_filename': str(folder / 'map.yaml'),
                         'frame_id': 'map', 'topic_name': '/map_preview/map',
                         'use_sim_time': False}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            namespace='map_preview', name='lifecycle_manager', output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server'],
                         'use_sim_time': False}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='saved_map_rviz',
            arguments=['-d', str(folder / 'view_map.rviz')], output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])
