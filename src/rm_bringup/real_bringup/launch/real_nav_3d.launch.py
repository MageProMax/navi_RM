import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description():
    bringup = get_package_share_directory('real_bringup')
    nav = get_package_share_directory('rm_bnrobot_nav')
    sim = get_package_share_directory('rm_bnrobot_sim')
    defaults = {
        'navigation_mode': 'prior',
        'pgo_params_file': os.path.join(bringup, 'config', 'exploration_pgo.yaml'),
        'localizer_backend': 'gicp',
        'localizer_accumulation_seconds': '0.5',
        'slam_params_file': os.path.join(bringup, 'config', 'exploration_slam.yaml'),
        'exploration_params_file': os.path.join(bringup, 'config', 'exploration.yaml'),
        'use_ui': 'false',
        'ui_start_empty': 'false',
        'patrol_mode': 'formal',
        'patrol_file': PathJoinSubstitution([get_package_share_directory('nav_plan'), 'config', 'patrol', PythonExpression(["'custom_room_test.yaml' if '", LaunchConfiguration('patrol_mode'), "' == 'test' else 'custom_room.yaml'"])]),
        'use_nav_plan': 'false', 'use_sim_time': 'false', 'use_fast_lio': 'true', 'use_rviz': 'true',
        'map': os.path.join(nav, 'maps', 'room.yaml'),
        'pcd_map': os.path.join(nav, 'maps', 'pcd', 'simmap_normal.pcd'),
        'param_file': os.path.join(bringup, 'config', 'real_nav2_params_3d.yaml'),
        'lio_config': os.path.join(get_package_share_directory('fastlio2'), 'config', 'lio.yaml'),
        'model': os.path.join(sim, 'urdf', 'bngu_sentinel', 'bnbot_real.xacro'),
        'rviz_config_file': os.path.join(bringup, 'rviz', 'nav_final.rviz'),
        'initial_pose_x': '-1.96', 'initial_pose_y': '-1.85', 'initial_pose_yaw': '0.0',
    }
    declared = [launch.actions.DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()]
    description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
    robot = launch_ros.actions.Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': description,
                     'use_sim_time': LaunchConfiguration('use_sim_time')}])
    core = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'navigation_3d_core.launch.py')),
        launch_arguments={k: LaunchConfiguration(k) for k in defaults if k != 'model'}.items())
    return launch.LaunchDescription(declared + [robot, core])
