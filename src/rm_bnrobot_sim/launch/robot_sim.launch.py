import launch
import launch_ros
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #默认路径（功能包，urdf,rviz）
    packges_default_path=get_package_share_directory("rm_bnrobot_sim")
    urdf_default_path=os.path.join(packges_default_path,"urdf/bngu_sentinel/bnbot.urdf.xacro")
    rviz_default_path=os.path.join(packges_default_path,"config/default_ui.rviz")
    model_arg_path=launch.actions.DeclareLaunchArgument(
        name='model',default_value=urdf_default_path,description="默认机器人urdf路径")
    #读取xacro文件，加载机器人模型秒速
    robot_model=launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(["xacro ",launch.substitutions.LaunchConfiguration("model")])
    )                                    #逆天了这里必须加空格！不然命令沾一块识别不出来
    #发布节点（机器人状态，关节，rviz2）
    robot_state_node=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_model}]
    )
    robot_joint_node=Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
    Rviz2_node=Node(
        package='rviz2',
        executable='rviz2',
        arguments=('-d',rviz_default_path)
    )
    return launch.LaunchDescription([
        model_arg_path,
        robot_state_node,
        robot_joint_node,
        Rviz2_node
    ])
    