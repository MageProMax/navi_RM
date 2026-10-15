import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start the real Mid-360, 3D relocalization and Nav2 stack."""
    bringup_path = get_package_share_directory('real_bringup')
    nav_path = get_package_share_directory('rm_bnrobot_nav')
    livox_path = get_package_share_directory('livox_ros_driver2')

    declare_map = launch.actions.DeclareLaunchArgument(
        'map', default_value=os.path.join(nav_path, 'maps', 'room.yaml'),
        description='2D occupancy map aligned with the 3D PCD map')
    declare_pcd_map = launch.actions.DeclareLaunchArgument(
        'pcd_map',
        # room.yaml and simmap_normal.pcd share custom_room world coordinates.
        default_value=os.path.join(nav_path, 'maps', 'pcd', 'simmap_normal.pcd'),
        description='PCD map used by the 3D ICP localizer')
    declare_use_rviz = launch.actions.DeclareLaunchArgument(
        'use_rviz', default_value='true')

    livox = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_path, 'launch_ROS2', 'msg_MID360_launch.py')))

    navigation = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path, 'launch', 'real_nav_3d.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'pcd_map': LaunchConfiguration('pcd_map'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_nav_plan': 'true',
            'navigation_mode': LaunchConfiguration('navigation_mode'),
            'pgo_params_file': LaunchConfiguration('pgo_params_file'),
            'localizer_backend': LaunchConfiguration('localizer_backend'),
            'localizer_accumulation_seconds': LaunchConfiguration('localizer_accumulation_seconds'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'exploration_params_file': LaunchConfiguration('exploration_params_file'),
            'use_ui': LaunchConfiguration('use_ui'),
            'patrol_file': LaunchConfiguration('patrol_file'),
            'patrol_mode': LaunchConfiguration('patrol_mode'),
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
        }.items(),
    )

    can_twist = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_path, 'launch', 'motion_arbiter.launch.py')),
        launch_arguments={'backend': 'real', 'use_sim_time': 'false',
                          'spin_speed': LaunchConfiguration('spin_speed'),
                          'spin_duration': LaunchConfiguration('spin_duration')}.items())
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('navigation_mode', default_value='prior', choices=['prior', 'explore']),
        launch.actions.DeclareLaunchArgument('localizer_backend', default_value='gicp', choices=['gicp', 'icp']),
        launch.actions.DeclareLaunchArgument('localizer_accumulation_seconds', default_value='0.5'),
        launch.actions.DeclareLaunchArgument('slam_params_file', default_value=os.path.join(bringup_path, 'config', 'exploration_slam.yaml')),
        launch.actions.DeclareLaunchArgument('pgo_params_file', default_value=os.path.join(bringup_path, 'config', 'exploration_pgo.yaml')),
        launch.actions.DeclareLaunchArgument('exploration_params_file', default_value=os.path.join(bringup_path, 'config', 'exploration.yaml')),
        launch.actions.DeclareLaunchArgument('use_ui', default_value='false'),
        launch.actions.DeclareLaunchArgument('patrol_mode', default_value='formal', choices=['formal', 'test']),
        launch.actions.DeclareLaunchArgument('patrol_file', default_value=PathJoinSubstitution([get_package_share_directory('nav_plan'), 'config', 'patrol', PythonExpression(["'custom_room_test.yaml' if '", LaunchConfiguration('patrol_mode'), "' == 'test' else 'custom_room.yaml'"])])),
        launch.actions.DeclareLaunchArgument('spin_speed', default_value='1.0'),
        launch.actions.DeclareLaunchArgument('spin_duration', default_value='10.0'),
        launch.actions.DeclareLaunchArgument('initial_pose_x', default_value='-1.96'),
        launch.actions.DeclareLaunchArgument('initial_pose_y', default_value='-1.85'),
        launch.actions.DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        declare_map,
        declare_pcd_map,
        declare_use_rviz,
        livox,
        launch.actions.TimerAction(period=3.0, actions=[navigation]),
        launch.actions.TimerAction(period=25.0, actions=[can_twist]),
    ])
