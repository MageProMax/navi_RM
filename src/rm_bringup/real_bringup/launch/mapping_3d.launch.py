"""Remote-controlled real-robot mapping; optional simulation backend for tests."""
import os
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup = get_package_share_directory('real_bringup')
    sim = get_package_share_directory('rm_bnrobot_sim')
    nav = get_package_share_directory('rm_bnrobot_nav')
    backend = LaunchConfiguration('backend')
    simulated = PythonExpression(["'", backend, "' == 'sim'"])
    sim_time = ParameterValue(simulated, value_type=bool)
    defaults = {
        'gui': 'true', 'use_rviz': 'true', 'start_lidar': 'true',
        'world': os.path.join(sim, 'world', 'custom_room.world'),
        'model': PythonExpression([
            repr(os.path.join(sim, 'urdf', 'bngu_sentinel', 'bnbot.urdf.xacro')),
            " if '", backend, "' == 'sim' else ",
            repr(os.path.join(sim, 'urdf', 'bngu_sentinel', 'bnbot_real.xacro'))]),
        'lio_config': PythonExpression([
            repr(os.path.join(nav, 'config', 'lio_sim.yaml')),
            " if '", backend, "' == 'sim' else ",
            repr(os.path.join(get_package_share_directory('fastlio2'), 'config', 'lio.yaml'))]),
        'pgo_params_file': os.path.join(bringup, 'config', 'exploration_pgo.yaml'),
        'rviz_config_file': os.path.join(bringup, 'rviz', 'mapping_3d.rviz'),
        'initial_pose_x': '-1.96', 'initial_pose_y': '-1.85', 'initial_pose_yaw': '0.0',
        'cmd_vel_topic': '/mapping/cmd_vel',
    }
    declared = [DeclareLaunchArgument('backend', default_value='real', choices=['sim', 'real'])] + [
        DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'gazebo_sim.launch.py')),
        condition=IfCondition(simulated), launch_arguments={
            key: LaunchConfiguration(key) for key in
            ['world', 'model', 'gui', 'initial_pose_x', 'initial_pose_y', 'initial_pose_yaw', 'cmd_vel_topic']}.items())
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')),
        condition=IfCondition(PythonExpression([
            "'", backend, "' == 'real' and '", LaunchConfiguration('start_lidar'), "'.lower() == 'true'"])))
    robot = Node(package='robot_state_publisher', executable='robot_state_publisher',
        condition=UnlessCondition(simulated), parameters=[{
            'robot_description': ParameterValue(launch.substitutions.Command(['xacro ', LaunchConfiguration('model')]), value_type=str),
            'use_sim_time': sim_time}])
    lio = Node(package='fastlio2', executable='lio_node', namespace='fastlio2', name='lio_node',
        output='screen', parameters=[{'config_path': LaunchConfiguration('lio_config'), 'use_sim_time': sim_time}])
    mapper = Node(package='pgo', executable='online_pgo_node', name='online_pgo', output='screen',
        parameters=[LaunchConfiguration('pgo_params_file'), {'use_sim_time': sim_time}])
    rviz = Node(package='rviz2', executable='rviz2', name='mapping_rviz',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config_file')], parameters=[{'use_sim_time': sim_time}])
    return launch.LaunchDescription(declared + [gazebo, driver, robot, mapper,
        TimerAction(period=2.0, actions=[lio]), rviz])
