import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description():
    """Launch Gazebo and the Nav2 stack using 3D ICP localization."""
    sim_path = get_package_share_directory('rm_bnrobot_sim')
    nav_path = get_package_share_directory('rm_bnrobot_nav')
    bringup_path = get_package_share_directory('real_bringup')

    default_world = os.path.join(sim_path, 'world', 'custom_room.world')
    default_model = os.path.join(
        sim_path, 'urdf', 'bngu_sentinel', 'bnbot.urdf.xacro')
    default_map = os.path.join(nav_path, 'maps', 'room.yaml')
    default_pcd_map = os.path.join(
        nav_path, 'maps', 'pcd', 'simmap_normal.pcd')

    declared_arguments = [
        launch.actions.DeclareLaunchArgument(
            'world', default_value=default_world),
        launch.actions.DeclareLaunchArgument(
            'model', default_value=default_model),
        launch.actions.DeclareLaunchArgument(
            'map', default_value=default_map,
            description='2D map paired with the simulated PCD'),
        launch.actions.DeclareLaunchArgument(
            'pcd_map', default_value=default_pcd_map,
            description='3D map used by simulated ICP localization'),
        launch.actions.DeclareLaunchArgument(
            'gui', default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'use_rviz', default_value='true'),
        launch.actions.DeclareLaunchArgument(
            'initial_pose_x', default_value='-1.96'),
        launch.actions.DeclareLaunchArgument(
            'initial_pose_y', default_value='-1.85'),
        launch.actions.DeclareLaunchArgument(
            'initial_pose_yaw', default_value='0.0'),
    ]

    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_path, 'launch', 'gazebo_sim.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'model': LaunchConfiguration('model'),
            'gui': LaunchConfiguration('gui'),
            'cmd_vel_topic': '/cmd_vel_sim',
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
        }.items(),
    )

    navigation = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_path, 'launch', 'nav_load_3d.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'pcd_map': LaunchConfiguration('pcd_map'),
            'use_sim_time': 'true',
            'use_fast_lio': 'true',
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
            'use_rviz': LaunchConfiguration('use_rviz'),
            # Use the same values for spawning and the ICP initial guess.
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
        }.items(),
    )

    motion = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_path, 'launch', 'motion_arbiter.launch.py')),
        launch_arguments={'backend': 'sim', 'use_sim_time': 'true',
                          'spin_speed': LaunchConfiguration('spin_speed'),
                          'spin_duration': LaunchConfiguration('spin_duration')}.items())
    return launch.LaunchDescription(
        [launch.actions.DeclareLaunchArgument('navigation_mode', default_value='prior', choices=['prior', 'explore']),
        launch.actions.DeclareLaunchArgument('localizer_backend', default_value='gicp', choices=['gicp', 'icp']),
        launch.actions.DeclareLaunchArgument('localizer_accumulation_seconds', default_value='0.5'),
        launch.actions.DeclareLaunchArgument('slam_params_file', default_value=os.path.join(bringup_path, 'config', 'exploration_slam.yaml')),
        launch.actions.DeclareLaunchArgument('pgo_params_file', default_value=os.path.join(bringup_path, 'config', 'exploration_pgo.yaml')),
        launch.actions.DeclareLaunchArgument('exploration_params_file', default_value=os.path.join(bringup_path, 'config', 'exploration.yaml')),
        launch.actions.DeclareLaunchArgument('use_ui', default_value='false'),
        launch.actions.DeclareLaunchArgument('patrol_mode', default_value='formal', choices=['formal', 'test']),
        launch.actions.DeclareLaunchArgument('patrol_file', default_value=PathJoinSubstitution([get_package_share_directory('nav_plan'), 'config', 'patrol', PythonExpression(["'custom_room_test.yaml' if '", LaunchConfiguration('patrol_mode'), "' == 'test' else 'custom_room.yaml'"])])),
        launch.actions.DeclareLaunchArgument('spin_speed', default_value='1.0'),
         launch.actions.DeclareLaunchArgument('spin_duration', default_value='10.0')] +
        declared_arguments + [gazebo, navigation,
                              launch.actions.TimerAction(period=25.0, actions=[motion])])
