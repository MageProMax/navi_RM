import launch
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {'backend': 'real', 'use_sim_time': 'false', 'spin_speed': '1.0', 'spin_duration': '10.0'}
    declared = [launch.actions.DeclareLaunchArgument('backend', default_value='real', choices=['real', 'sim'])] + [
        launch.actions.DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items() if k != 'backend']
    simulated = PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"])
    parameters = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'dry_run': ParameterValue(simulated, value_type=bool),
        'allow_rotation': True, 'spin_enabled': True, 'require_spin_permission': True,
        'cmd_vel_timeout': .25, 'linear_limit': 1.0,
        'spin_speed': ParameterValue(LaunchConfiguration('spin_speed'), value_type=float),
        'spin_duration': ParameterValue(LaunchConfiguration('spin_duration'), value_type=float),
    }
    arbiter = Node(package='can_twist', executable='can_twist_node', name='can_twist_node',
                   output='screen', parameters=[parameters])
    bridge = Node(package='real_bringup', executable='sim_can_bridge.py', name='sim_can_bridge',
                  output='screen', condition=launch.conditions.IfCondition(simulated),
                  parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])
    return launch.LaunchDescription(declared + [arbiter, bridge])
