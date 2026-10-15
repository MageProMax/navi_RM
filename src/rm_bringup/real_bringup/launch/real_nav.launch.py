import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    nav_package_path = get_package_share_directory('rm_bnrobot_nav')
    real_nav_package_path = get_package_share_directory('real_bringup')
    bringup_path = get_package_share_directory('nav2_bringup')
    default_map_path = os.path.join(nav_package_path, 'maps', 'room.yaml')
    nav2_params_path = os.path.join(real_nav_package_path, 'config', 'real_nav2_params.yaml')
    nav_to_pose_bt_path = os.path.join(
        real_nav_package_path, 'behavior_trees', 'navigate_to_pose_no_yaw.xml')
    nav_through_poses_bt_path = os.path.join(
        real_nav_package_path, 'behavior_trees', 'navigate_through_poses_no_yaw.xml')
    
    rviz2_path = os.path.join(nav_package_path, 'rviz', 'rviz2.rviz')
    

    
    urdf_path = get_package_share_directory('rm_bnrobot_sim')
    model_path = urdf_path + '/urdf/bngu_sentinel/bnbot_real.xacro'

    mode_arg_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(model_path),
        description='URDF 的绝对路径'
    )

    declare_use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    declare_map_path = launch.actions.DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='地图文件路径'
    )
    declare_params_path = launch.actions.DeclareLaunchArgument(
        'param_file',
        default_value=nav2_params_path,  
        description='导航参数文件路径'
    )
    
    declare_use_fast_lio = launch.actions.DeclareLaunchArgument(
        'use_fast_lio',
        default_value='True',
        description='是否使用 Fast LIO 进行定位',
    )
    
    use_sim_arg = launch.substitutions.LaunchConfiguration('use_sim_time')
    map_arg = launch.substitutions.LaunchConfiguration('map')
    param_arg = launch.substitutions.LaunchConfiguration('param_file')
    use_fast_lio_arg = launch.substitutions.LaunchConfiguration('use_fast_lio')

    nav_params_no_yaw = RewrittenYaml(
        source_file=param_arg,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_path,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt_path,
        },
        convert_types=True,
    )

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': use_sim_arg}]
    )


    fastlio2_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )

    fast_lio_node = launch_ros.actions.Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[{"config_path": fastlio2_config_path.perform(launch.LaunchContext()),
                     'use_sim_time': use_sim_arg}],
        condition=launch.conditions.IfCondition(use_fast_lio_arg)
    )

    # =========================================================

    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',      
            'transform_tolerance': 0.5,            
            # base_link uses the ground as z=0.  Keep a 10 cm margin so floor
            # noise and small LIO roll/pitch errors do not become obstacles.
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
            'use_sim_time': use_sim_arg,
        }],
        remappings=[

            # 它发出的点云话题默认带前缀，所以改为接收 /fastlio2/cloud_registered
            ('/cloud_in', '/fastlio2/body_cloud'),
            ('/scan', '/scan')
        ],
        condition=launch.conditions.IfCondition(use_fast_lio_arg)
    )

    # 导航相关节点组合
    navi_group = launch.actions.GroupAction([
        SetRemap(src='/odom', dst='/fastlio2/lio_odom'),

        # 1. 导航控制模块
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_path, 'launch', 'navigation_launch.py')), 
            launch_arguments={
                'use_sim_time': use_sim_arg,
                'params_file': nav_params_no_yaw,
                'autostart': 'True',
                'use_composition': 'False',
            }.items(),
        ),
        
        # 2. 2D 地图服务器 
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_arg, 'use_sim_time': use_sim_arg}]
        ),

        # AMCL is the sole map -> odom authority. FAST-LIO continues to own
        # odom -> base_link.
        launch_ros.actions.Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[param_arg, {'use_sim_time': use_sim_arg}]
        ),
        
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_arg},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )
    ])

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("real_bringup"), "rviz", "nav_final.rviz"]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg.perform(launch.LaunchContext())],
        parameters=[{'use_sim_time': use_sim_arg}],
        output='screen'
    )
    ld = launch.LaunchDescription([
        declare_map_path,
        declare_use_sim_time,
        declare_params_path,
        declare_use_fast_lio,
        mode_arg_path,
    ])

    # 机器人静态 TF；map -> odom 由 AMCL 动态发布。
    ld.add_action(robot_state_publisher_node)  


    # Fast LIO
    if fast_lio_node:
        ld.add_action(fast_lio_node)

    # 延迟启动激光转扫描
    if pointcloud_to_laserscan_node:
        ld.add_action(launch.actions.TimerAction(
            period=4.0,
            actions=[pointcloud_to_laserscan_node]
        ))

    # 延迟启动导航和 RViz
    ld.add_action(launch.actions.TimerAction(
        period=6.0,
        actions=[navi_group, rviz_node]
    ))

    return ld
