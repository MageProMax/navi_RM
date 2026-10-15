import os
import sys

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the Gazebo robot model and its joint-state controller."""
    robot_name = 'bngu_sentinel'
    package_path = get_package_share_directory('rm_bnrobot_sim')
    default_urdf_path = os.path.join(
        package_path, 'urdf', 'bngu_sentinel', 'bnbot.urdf.xacro')
    default_world_path = os.path.join(
        package_path, 'world', 'custom_room.world')
    gazebo_path = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch', 'gazebo.launch.py')

    model_path_arg = launch.actions.DeclareLaunchArgument(
        'model', default_value=default_urdf_path,
        description='Robot xacro path')
    world_path_arg = launch.actions.DeclareLaunchArgument(
        'world', default_value=default_world_path,
        description='Gazebo world path')
    gui_arg = launch.actions.DeclareLaunchArgument(
        'gui', default_value='false',
        description='Start gzclient; disabled by default to avoid GPU/OOM kills')
    initial_pose_x_arg = launch.actions.DeclareLaunchArgument(
        'initial_pose_x', default_value='-1.96')
    initial_pose_y_arg = launch.actions.DeclareLaunchArgument(
        'initial_pose_y', default_value='-1.85')
    initial_pose_yaw_arg = launch.actions.DeclareLaunchArgument(
        'initial_pose_yaw', default_value='0.0')
    robot_model = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', LaunchConfiguration('model'),
                                      ' cmd_vel_topic:=', LaunchConfiguration('cmd_vel_topic')]),
        value_type=str,
    )
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_model,
            'use_sim_time': True,
        }],
    )
    gazebo_node = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_path),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true',
        }.items(),
    )
    spawn_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # Use the launch interpreter, which already has working ROS bindings,
        # rather than the Conda interpreter chosen by the script's env shebang.
        prefix=[sys.executable],
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name,
                   '-x', LaunchConfiguration('initial_pose_x'),
                   '-y', LaunchConfiguration('initial_pose_y'),
                   '-z', '0',
                   '-Y', LaunchConfiguration('initial_pose_yaw')],
    )

    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'bnbot_joint_state_broadcaster'],
        output='screen',
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        model_path_arg,
        world_path_arg,
        gui_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
        robot_state_node,
        gazebo_node,
        spawn_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_node,
                on_exit=[load_joint_state_controller],
            ),
        ),
    ])
