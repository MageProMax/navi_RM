import os
import yaml
from launch.substitution import Substitution

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


class ModeParameter(Substitution):
    """Preserve custom prior settings; override only the mapless planner."""
    def __init__(self, path, exploration_value, prior_default=None):
        super().__init__();self.path=path;self.exploration_value=exploration_value;self.prior_default=prior_default
    def perform(self, context):
        if LaunchConfiguration('navigation_mode').perform(context)=='explore':value=self.exploration_value
        else:
            with open(LaunchConfiguration('param_file').perform(context)) as file:value=yaml.safe_load(file)
            for key in self.path:
                if key not in value and self.prior_default is not None:
                    value=self.prior_default;break
                value=value[key]
        return str(value).lower() if isinstance(value,bool) else str(value)


def generate_launch_description():
    """Shared real/sim prior-map or mapless SLAM navigation core."""
    nav_path = get_package_share_directory('rm_bnrobot_nav')
    bringup_path = get_package_share_directory('real_bringup')

    default_map = os.path.join(nav_path, 'maps', 'room.yaml')
    # User-selected custom_room 2D/3D map pair in Gazebo world coordinates.
    default_pcd_map = os.path.join(nav_path, 'maps', 'pcd', 'simmap_normal.pcd')
    default_params = os.path.join(
        bringup_path, 'config', 'real_nav2_params_3d.yaml')
    nav_to_pose_bt = os.path.join(
        bringup_path, 'behavior_trees', 'navigate_to_pose_no_yaw.xml')
    nav_through_poses_bt = os.path.join(
        bringup_path, 'behavior_trees', 'navigate_through_poses_no_yaw.xml')
    rviz_config = os.path.join(bringup_path, 'rviz', 'nav_final.rviz')

    declared_arguments = [
        launch.actions.DeclareLaunchArgument('navigation_mode', default_value='prior', choices=['prior', 'explore']),
        launch.actions.DeclareLaunchArgument('localizer_backend', default_value='gicp', choices=['gicp', 'icp']),
        launch.actions.DeclareLaunchArgument('localizer_accumulation_seconds', default_value='0.5'),
        launch.actions.DeclareLaunchArgument('slam_params_file', default_value=os.path.join(bringup_path, 'config', 'exploration_slam.yaml')),
        launch.actions.DeclareLaunchArgument('pgo_params_file', default_value=os.path.join(bringup_path, 'config', 'exploration_pgo.yaml')),
        launch.actions.DeclareLaunchArgument('exploration_params_file', default_value=os.path.join(bringup_path, 'config', 'exploration.yaml')),
        launch.actions.DeclareLaunchArgument('use_ui', default_value='false'),
        launch.actions.DeclareLaunchArgument('ui_start_empty', default_value='false'),
        launch.actions.DeclareLaunchArgument('patrol_mode', default_value='formal', choices=['formal', 'test']),
        launch.actions.DeclareLaunchArgument('patrol_file', default_value=PathJoinSubstitution([get_package_share_directory('nav_plan'), 'config', 'patrol', PythonExpression(["'custom_room_test.yaml' if '", LaunchConfiguration('patrol_mode'), "' == 'test' else 'custom_room.yaml'"])])),
        launch.actions.DeclareLaunchArgument('use_nav_plan', default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'map', default_value=default_map,
            description='2D occupancy map aligned with the PCD map'),
        launch.actions.DeclareLaunchArgument(
            'pcd_map', default_value=default_pcd_map,
            description='3D PCD map used by coarse-to-fine ICP'),
        launch.actions.DeclareLaunchArgument('initial_pose_x', default_value='-1.96'),
        launch.actions.DeclareLaunchArgument('initial_pose_y', default_value='-1.85'),
        launch.actions.DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        launch.actions.DeclareLaunchArgument(
            'param_file', default_value=default_params,
            description='Separate Nav2 and 3D-localizer parameter file'),
        launch.actions.DeclareLaunchArgument(
            'lio_config', default_value=os.path.join(
                get_package_share_directory('fastlio2'), 'config', 'lio.yaml')),
        launch.actions.DeclareLaunchArgument(
            'use_fast_lio', default_value='true',
            description='Start FAST-LIO in this launch file'),
        launch.actions.DeclareLaunchArgument(
            'use_rviz', default_value='true'),
        launch.actions.DeclareLaunchArgument('rviz_config_file', default_value=rviz_config),
    ]

    prior = launch.conditions.IfCondition(PythonExpression(["'", LaunchConfiguration('navigation_mode'), "' == 'prior'"]))
    explore = launch.conditions.IfCondition(PythonExpression(["'", LaunchConfiguration('navigation_mode'), "' == 'explore'"]))
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path = LaunchConfiguration('map')
    pcd_map = LaunchConfiguration('pcd_map')
    param_file = LaunchConfiguration('param_file')
    use_fast_lio = LaunchConfiguration('use_fast_lio')
    use_rviz = LaunchConfiguration('use_rviz')

    configured_params = RewrittenYaml(
        source_file=param_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'planner_server.ros__parameters.GridBased.allow_unknown': ModeParameter(['planner_server','ros__parameters','GridBased','allow_unknown'],False),
            'planner_server.ros__parameters.GridBased.plugin': ModeParameter(['planner_server','ros__parameters','GridBased','plugin'],'nav2_smac_planner/SmacPlanner2D'),
            'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.expected_update_rate': ModeParameter(
                ['local_costmap','local_costmap','ros__parameters','obstacle_layer','scan','expected_update_rate'], .5, 0.),
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.expected_update_rate': ModeParameter(
                ['global_costmap','global_costmap','ros__parameters','obstacle_layer','scan','expected_update_rate'], .5, 0.),
            'default_nav_to_pose_bt_xml': nav_to_pose_bt,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt,
        },
        convert_types=True,
    )

    lio_config = LaunchConfiguration('lio_config')

    fast_lio = launch_ros.actions.Node(
        package='fastlio2',
        namespace='fastlio2',
        executable='lio_node',
        name='lio_node',
        output='screen',
        parameters=[{
            'config_path': lio_config,
            'use_sim_time': use_sim_time,
        }],
        condition=launch.conditions.IfCondition(use_fast_lio),
    )

    pointcloud_to_laserscan = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        condition=prior,
        output='screen',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.5,
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
    )

    mapping_scan = launch_ros.actions.Node(
        package='real_bringup', executable='accumulated_scan.py',
        name='pointcloud_to_mapping_scan', output='screen', condition=explore,
        parameters=[{'history_seconds':1.0, 'publish_period':.5,
            'min_height':.10, 'max_height':1.5, 'use_sim_time':use_sim_time}],
        remappings=[('scan','/scan_mapping')])

    exploration_obstacle_scan = launch_ros.actions.Node(
        package='real_bringup', executable='accumulated_scan.py',
        name='exploration_obstacle_scan', output='screen', condition=explore,
        parameters=[{'history_seconds':.6, 'publish_period':.1,
            'min_height':.10, 'max_height':1.5, 'use_sim_time':use_sim_time}],
        remappings=[('scan','/scan')])

    localizer = launch_ros.actions.Node(
        condition=prior,
        package='localizer',
        namespace='localizer',
        executable='localizer_node',
        name='localizer_node',
        output='screen',
        parameters=[configured_params, {
            'map_path': pcd_map,
            'registration_method': LaunchConfiguration('localizer_backend'),
            'accumulation_seconds': launch_ros.parameter_descriptions.ParameterValue(
                LaunchConfiguration('localizer_accumulation_seconds'), value_type=float),
            'initial_x': launch_ros.parameter_descriptions.ParameterValue(
                LaunchConfiguration('initial_pose_x'), value_type=float),
            'initial_y': launch_ros.parameter_descriptions.ParameterValue(
                LaunchConfiguration('initial_pose_y'), value_type=float),
            'initial_yaw': launch_ros.parameter_descriptions.ParameterValue(
                LaunchConfiguration('initial_pose_yaw'), value_type=float),
            'use_sim_time': use_sim_time,
        }],
    )

    navigation_group = launch.actions.GroupAction([
        SetRemap(src='/odom', dst='/navigation/odom'),
        launch_ros.actions.Node(package='pgo', executable='online_pgo_node',
            name='online_pgo', output='screen', condition=explore,
            parameters=[LaunchConfiguration('pgo_params_file'), {'use_sim_time': use_sim_time}]),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_path, 'launch', 'navigation_spin.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': configured_params,
                'autostart': 'true',
                # Nav2 Humble evaluates this inside a PythonExpression.
                'use_composition': 'False',
            }.items(),
        ),
        launch_ros.actions.Node(
            condition=prior,
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params, {
                'yaml_filename': map_path,
                'use_sim_time': use_sim_time,
            }],
        ),
        localizer,
        # Exactly one map -> odom authority: ICP in prior mode, SLAM in
        # exploration. FAST-LIO remains the odom -> base_link owner.
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            condition=prior,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
    ])

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz),
    )

    decision = launch_ros.actions.Node(
        package='nav_plan', executable='nav_plan_node', name='nav_plan_node',
        output='screen', parameters=[LaunchConfiguration('patrol_file'), {'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(PythonExpression(["'", LaunchConfiguration('use_nav_plan'), "'.lower() == 'true' and '", LaunchConfiguration('patrol_mode'), "' == 'formal' and '", LaunchConfiguration('navigation_mode'), "' == 'prior'"])))
    return launch.LaunchDescription([
        # Avoid blocking Fast DDS service replies behind synchronous writes.
        # Respect an explicit publication mode supplied by the operator.
        launch.actions.SetEnvironmentVariable('RMW_FASTRTPS_PUBLICATION_MODE',
            launch.substitutions.EnvironmentVariable('RMW_FASTRTPS_PUBLICATION_MODE',
                default_value='ASYNCHRONOUS')),
    ] + declared_arguments + [
        launch.actions.TimerAction(period=14.0, actions=[decision]),
        launch_ros.actions.Node(
            package='rm_bnrobot_function', executable='nav_client_ui.py', name='nav_client_ui',
            output='screen', parameters=[{'use_sim_time': use_sim_time,
                'patrol_file': LaunchConfiguration('patrol_file'),
                'ui_start_empty': launch_ros.parameter_descriptions.ParameterValue(
                    LaunchConfiguration('ui_start_empty'), value_type=bool),
                'navigation_mode': LaunchConfiguration('navigation_mode')}],
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_ui'))),
        launch_ros.actions.Node(
            package='rm_bnrobot_function', executable='nav_pose', name='nav_to_pose_client',
            condition=prior, output='screen', parameters=[LaunchConfiguration('patrol_file'), {'use_sim_time': use_sim_time,
                'patrol.mode': LaunchConfiguration('patrol_mode'), 'require_localization': True, 'require_motion_ready': True}],
            remappings=[('targetPose', 'goal_pose')]),
        launch_ros.actions.Node(
            package='rm_bnrobot_function', executable='nav_pose', name='nav_to_pose_client',
            condition=explore, output='screen', parameters=[{'use_sim_time': use_sim_time,
                'patrol.mode': 'explore', 'require_localization': True, 'require_motion_ready': True, 'require_mapping_ready': True}]),
        launch_ros.actions.Node(package='real_bringup', executable='frontier_explorer.py',
            name='frontier_explorer', output='screen', condition=explore,
            parameters=[LaunchConfiguration('exploration_params_file'), {'use_sim_time': use_sim_time, 'require_mapping_ready': True}]),
        launch_ros.actions.Node(
            package='real_bringup', executable='spin_navigation_adapter.py',
            name='spin_navigation_adapter', output='screen',
            parameters=[LaunchConfiguration('param_file'), {'use_sim_time': use_sim_time, 'require_mapping_ready': launch_ros.parameter_descriptions.ParameterValue(
                PythonExpression(["'", LaunchConfiguration('navigation_mode'), "' == 'explore'"]), value_type=bool), 'scan_guard': launch_ros.parameter_descriptions.ParameterValue(
                PythonExpression(["'", LaunchConfiguration('navigation_mode'), "' == 'explore'"]), value_type=bool)}]),
        launch.actions.TimerAction(period=2.0, actions=[fast_lio]),
        launch.actions.TimerAction(
            period=4.0, actions=[pointcloud_to_laserscan, mapping_scan, exploration_obstacle_scan]),
        launch.actions.TimerAction(
            period=6.0, actions=[navigation_group, rviz]),
    ])
