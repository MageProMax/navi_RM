import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch Gazebo and the real-equivalent localization/navigation stack."""
    sim_path = get_package_share_directory('rm_bnrobot_sim')
    nav_path = get_package_share_directory('rm_bnrobot_nav')

    default_world = os.path.join(sim_path, 'world', 'custom_room.world')
    default_model = os.path.join(
        sim_path, 'urdf', 'bngu_sentinel', 'bnbot.urdf.xacro')
    default_map = os.path.join(nav_path, 'maps', 'room.yaml')

    declare_world = launch.actions.DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Gazebo world file')
    declare_model = launch.actions.DeclareLaunchArgument(
        'model', default_value=default_model,
        description='Simulation robot xacro')
    declare_map = launch.actions.DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Map yaml corresponding to the Gazebo world')
    declare_gui = launch.actions.DeclareLaunchArgument(
        'gui', default_value='false',
        description='Start the Gazebo graphical client (gzclient)')
    declare_initial_pose_x = launch.actions.DeclareLaunchArgument(
        'initial_pose_x', default_value='-1.96')
    declare_initial_pose_y = launch.actions.DeclareLaunchArgument(
        'initial_pose_y', default_value='-1.85')
    declare_initial_pose_yaw = launch.actions.DeclareLaunchArgument(
        'initial_pose_yaw', default_value='0.0')

    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_path, 'launch', 'gazebo_sim.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'model': LaunchConfiguration('model'),
            'gui': LaunchConfiguration('gui'),
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
        }.items(),
    )

    navigation = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_path, 'launch', 'nav_load.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'use_fast_lio': 'true',
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
        }.items(),
    )

    return launch.LaunchDescription([
        declare_world,
        declare_model,
        declare_map,
        declare_gui,
        declare_initial_pose_x,
        declare_initial_pose_y,
        declare_initial_pose_yaw,
        gazebo,
        navigation,
    ])
