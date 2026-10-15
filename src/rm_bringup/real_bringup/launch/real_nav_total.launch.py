import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 立即启动 Livox 雷达 
    livox_dir = get_package_share_directory('livox_ros_driver2')
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_dir, 'launch_ROS2', 'msg_MID360_launch.py')
        )
    )

    # 2. 延迟启动 Nav2导航和重定位节点
    real_bringup_dir = get_package_share_directory('real_bringup')
    map_dir=get_package_share_directory('rm_bnrobot_nav')
    map_path=os.path.join(map_dir,'maps/qingzhen_bbb.yaml')
    real_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(real_bringup_dir, 'launch', 'real_nav.launch.py')
        ),
        launch_arguments={
            'map': map_path
        }.items()
    )
    navGroup_launch = TimerAction(
        period=3.0,
        actions=[real_nav_launch]
    )


    # 3. 下发节点和导航客户端 (延迟13秒)
    nav_pose_node = Node(
        package='rm_bnrobot_function',
        executable='nav_pose',
        name='nav_pose_node',
        output='screen'
    )
    
    can_twist_node = Node(
        package='can_twist',
        executable='can_twist_node',
        name='can_twist_node',
        output='screen'
    )
    
    nav_can_launch = TimerAction(
        period=25.0,
        actions=[can_twist_node] #nav_pose_node, 
    )

   
    # 4. 获取位姿节点 
    get_pose_node = Node(
        package='rm_bnrobot_function',
        executable='get_pose.py', 
        name='get_pose_node',
        output='screen'
    )
    
    getPose_launch = TimerAction(
        period=14.0,
        actions=[get_pose_node]
    )


    # 5. 决策节点rm_plan 
    # rm_plan_dir = get_package_share_directory('rm_plan')
    # rm_plan_launch = TimerAction(
    #     period=15.0,
    #     actions=[IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(rm_plan_dir, 'launch', 'sentry_test.launch.py')
    #     )
    # )]
    # )

    nav_plan_node = Node(
        package='nav_plan',
        executable='nav_plan_node',
        name='nav_plan_node',
        output='screen',
        emulate_tty=True,
    )
    navPlan_launch = TimerAction(
        period=14.0,
        actions=[nav_plan_node]
    )


    ld = LaunchDescription()

    ld.add_action(livox_launch)
    ld.add_action(navGroup_launch)
    ld.add_action(nav_can_launch)
    # ld.add_action(getPose_launch)         
    #ld.add_action(rm_plan_launch)
    ld.add_action(navPlan_launch)
    return ld