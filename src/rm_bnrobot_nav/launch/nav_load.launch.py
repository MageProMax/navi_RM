import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Start the simulation localization/navigation stack used by the real robot."""
    nav_package_path = get_package_share_directory('rm_bnrobot_nav')
    real_bringup_path = get_package_share_directory('real_bringup')
    nav2_bringup_path = get_package_share_directory('nav2_bringup')

    default_map_path = os.path.join(nav_package_path, 'maps', 'room.yaml')
    default_params_path = os.path.join(
        real_bringup_path, 'config', 'real_nav2_params.yaml')
    nav_to_pose_bt_path = os.path.join(
        real_bringup_path, 'behavior_trees', 'navigate_to_pose_no_yaw.xml')
    nav_through_poses_bt_path = os.path.join(
        real_bringup_path, 'behavior_trees',
        'navigate_through_poses_no_yaw.xml')
    rviz_path = os.path.join(real_bringup_path, 'rviz', 'nav_final.rviz')

    declare_use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the Gazebo simulation clock')
    declare_map_path = launch.actions.DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='Map yaml used by map_server and AMCL')
    declare_params_path = launch.actions.DeclareLaunchArgument(
        'param_file', default_value=default_params_path,
        description='Nav2 parameters; defaults to the real-robot parameters')
    declare_use_fast_lio = launch.actions.DeclareLaunchArgument(
        'use_fast_lio', default_value='true',
        description='Run FAST-LIO with the simulated Mid-360 and IMU')
    declare_use_rviz = launch.actions.DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz2')
    declare_initial_pose_x = launch.actions.DeclareLaunchArgument(
        'initial_pose_x', default_value='-1.96',
        description='AMCL initial x coordinate in the simulation map')
    declare_initial_pose_y = launch.actions.DeclareLaunchArgument(
        'initial_pose_y', default_value='-1.85',
        description='AMCL initial y coordinate in the simulation map')
    declare_initial_pose_yaw = launch.actions.DeclareLaunchArgument(
        'initial_pose_yaw', default_value='0.0',
        description='AMCL initial yaw in the simulation map')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path = LaunchConfiguration('map')
    params_path = LaunchConfiguration('param_file')
    use_fast_lio = LaunchConfiguration('use_fast_lio')
    use_rviz = LaunchConfiguration('use_rviz')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    # Share real tuning and only rewrite simulation clock and no-yaw BT paths.
    sim_nav_params = RewrittenYaml(
        source_file=params_path,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_path,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt_path,
        },
        convert_types=True,
    )

    fastlio_config_path = PathJoinSubstitution(
        [FindPackageShare('rm_bnrobot_nav'), 'config', 'lio_sim.yaml'])
    fast_lio_node = launch_ros.actions.Node(
        package='fastlio2',
        namespace='fastlio2',
        executable='lio_node',
        name='lio_node',
        output='screen',
        parameters=[{
            'config_path': fastlio_config_path,
            'use_sim_time': use_sim_time,
        }],
        condition=launch.conditions.IfCondition(use_fast_lio),
    )

    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.5,
            # Reject the floor and its LIO/noise band around z=0.  The cloud
            # is already expressed in the ground-referenced base_link frame.
            'min_height': 0.10,
            'max_height': 0.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 10.0,
            'use_inf': False,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/cloud_in', '/fastlio2/body_cloud'),
            ('/scan', '/scan'),
        ],
        condition=launch.conditions.IfCondition(use_fast_lio),
    )

    navigation_group = launch.actions.GroupAction([
        # Gazebo /odom remains diagnostic; Nav2 consumes FAST-LIO odometry.
        SetRemap(src='/odom', dst='/fastlio2/lio_odom'),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_path, 'launch',
                             'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': sim_nav_params,
                'autostart': 'True',
                'use_composition': 'False',
            }.items(),
        ),
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[sim_nav_params, {
                'yaml_filename': map_path,
                'use_sim_time': use_sim_time,
            }],
        ),
        # AMCL alone owns map -> odom; FAST-LIO owns odom -> base_link.
        launch_ros.actions.Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[sim_nav_params, {
                'use_sim_time': use_sim_time,
                # Gazebo has a deterministic spawn pose. Seed AMCL from it so
                # map -> odom exists without requiring an RViz click.
                'set_initial_pose': True,
                'always_reset_initial_pose': True,
                'initial_pose.x': initial_pose_x,
                'initial_pose.y': initial_pose_y,
                'initial_pose.z': 0.0,
                'initial_pose.yaw': initial_pose_yaw,
            }],
        ),
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ])

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=launch.conditions.IfCondition(use_rviz),
    )

    return launch.LaunchDescription([
        declare_map_path,
        declare_use_sim_time,
        declare_params_path,
        declare_use_fast_lio,
        declare_use_rviz,
        declare_initial_pose_x,
        declare_initial_pose_y,
        declare_initial_pose_yaw,
        launch.actions.TimerAction(period=2.0, actions=[fast_lio_node]),
        launch.actions.TimerAction(
            period=8.0, actions=[pointcloud_to_laserscan_node]),
        launch.actions.TimerAction(
            period=10.0, actions=[navigation_group, rviz_node]),
    ])
