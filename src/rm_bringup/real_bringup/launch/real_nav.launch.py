import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    nav_package_path = get_package_share_directory('rm_bnrobot_nav')
    real_nav_package_path = get_package_share_directory('real_bringup')
    bringup_path = get_package_share_directory('nav2_bringup')
    default_map_path = os.path.join(nav_package_path, 'maps', 'room.yaml')
    nav2_params_path = os.path.join(real_nav_package_path, 'config', 'real_nav2_params.yaml')
    rviz2_path = os.path.join(nav_package_path, 'rviz', 'rviz2.rviz')
    rviz2_path2 = os.path.join(real_nav_package_path, 'rviz', 'fastlio.rviz')
    
    
    # Fast_lio 相关路径
    fast_lio_path = get_package_share_directory('fast_lio')
    fast_lio_config_path = os.path.join(real_nav_package_path, 'config')

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
        description='是否使用 Fast LIO 进行定位'
    )
    
    use_sim_arg = launch.substitutions.LaunchConfiguration('use_sim_time')
    map_arg = launch.substitutions.LaunchConfiguration('map')
    param_arg = launch.substitutions.LaunchConfiguration('param_file')
    use_fast_lio_arg = launch.substitutions.LaunchConfiguration('use_fast_lio')


    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    


    # Fast_lio 节点
    fast_lio_node = None
    if fast_lio_path:
        fast_lio_node = launch_ros.actions.Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio',  
            output='screen',
            parameters=[
                PathJoinSubstitution([fast_lio_config_path, 'mid360_real.yaml']), 
                {
                    # 1. 时间配置
                    'use_sim_time': use_sim_arg,
                    'use_system_time': False,
                    
                    'publish_frame_id': 'map',         
                    'child_frame_id': 'base_link',       
                    'publish_tf': True,                
                    
                    # 3. 里程计发布
                    'publish_odom': True,               
                    'odom_topic': '/odom',               
                    'odom_frame_id': 'map',             
                    'base_link_frame_id': 'base_link',  
                    
                    
                    #'extrinsic_T': [0.0, 0.0, 0.0],     
                    #'extrinsic_R': [0.0, 0.0, 0.0, 1.0], #
             
                    
                    # 5. 其他辅助参数
                    'publish_rate': 50.0,                # TF/里程计发布频率
                }
            ],
            remappings=[
        
            ],
            condition=launch.conditions.IfCondition(use_fast_lio_arg)
        )
    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'mid360',      
            'transform_tolerance': 0.5,            
            'min_height': 0.0,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 50.0,
            'use_inf': False,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_arg,
        }],
        remappings=[
            ('cloud_in', '/cloud_registered'),
            ('scan', '/scan')
        ],
        condition=launch.conditions.IfCondition(use_fast_lio_arg)
    )

    # 导航相关节点组合
    navi_group = launch.actions.GroupAction([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_path, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_arg,
                'use_sim_time': use_sim_arg,
                'params_file': param_arg,
                'autostart': 'True',
                'use_composition': 'True',
            }.items(),
        )
    ])

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_path],
        parameters=[{'use_sim_time': use_sim_arg}],
        output='screen'
    )


    ld = launch.LaunchDescription([
        declare_map_path,
        declare_use_sim_time,
        declare_params_path,
        declare_use_fast_lio,
        mode_arg_path
    ])

    # 静态 TF
    ld.add_action(robot_state_publisher_node)  

    # Fast LIO
    if fast_lio_node:
        ld.add_action(fast_lio_node)
    
    # 延迟启动激光转扫描
    if pointcloud_to_laserscan_node:
        ld.add_action(launch.actions.TimerAction(
            period=2.0,
            actions=[pointcloud_to_laserscan_node]
        ))

    # 延迟启动导航和 RViz
    ld.add_action(launch.actions.TimerAction(
        period=5.0,
        actions=[navi_group, rviz_node]
    ))

    return ld