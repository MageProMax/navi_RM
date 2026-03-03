## author 牢🐴

# question
- 扫描高度
- 只能完整一次完整避障
- 上层规划


# 基于ro2 和 nav2 自动巡检
## 1.项目介绍
该项目是基于肉丝2和nav2的哨兵导航

## 2.功能包介绍
- rm_bnrobot_sim 机器人仿真
- rm_bnrobot_nav 机器人导航
- rm_bnrobot_function 导航相关功能包
- rm_interfaces  目标相关接口
- livox_ros_driver2 雷达驱动
- can_twist_node 指令发送节点
- can_control    新的下发指令节点
- rm_bringup 一键启动功能包（real可用）
## 3 安装依赖
- 系统版本： Ubunt22.04     
- ROS 版本：ROS 2 Humble
本项目建图使用 slam-toolbox，fast_lio，导航使用 Navigation 2 ,仿真采用 Gazebo，运动控制采用 ros2-control ，
依赖：（没写到的只有去问豆包了）

1. 安装基本依赖（slam_toolbox,nav2,ros2_control）

```
sudo apt install ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-slam-toolbox
sudo apt install -y ros-humble-gazebo-ros2-control
```

2. 安装仿真相关功能包

```
sudo apt install ros-$ROS_DISTRO-robot-state-publisher  ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-xacro
sudo apt install ros-$ROS_DISTRO-tf-transformations
sudo pip3 install transforms3d
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-pointcloud-to-laserscan
```

3. jetson 源码编译gazebo_ros_pkgs
https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros
编译pulginp时可能会出现.h文件和.hpp文件的冲突，主要是jetson的tf2文件夹下没有.hpp文件.按照报错提示vim将.hpp文件修改成.h文件即可
```
source /home/jetson/self_gazebo/install/setup.bash
source /home/jetson/navi_RM/install/setup.bash
```

## 2.2 接口
| Topic name | Type | Note |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Mid360 自定义消息类型 |
| `/livox/lidar/pointcloud` | `sensor_msgs/msg/PointCloud2` | ROS2 点云消息类型 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Gazebo 插件仿真 IMU |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 麦克纳姆轮小车运动控制接口 |

## 2.3 运行

### 1. 一键启动
##### 实例模式
```
- ros2 launch livox_ros_driver2 msg_MID360_launch.py    # 启动雷达
- ros2 launch real_bringup real_nav.launch.py           # 启动导航
- ros2 run can_control can_control                      # 启动下发节点
- ros2 run rm_bnrobot_function get_pose.py 
- ros2 launch rm_plan sentry_test.launch.py 
- ros2 run rm_bnrobot_function nav_pose 

```
1.1 使用map2.yaml参数
```
ros2 launch real_bringup real_nav.launch.py map:=/home/jetson/navi_RM/src/rm_bnrobot_nav/maps/maps2.yaml
ros2 launch real_bringup real_nav.launch.py map:=/home/jetson/navi_RM/src/rm_bnrobot_nav/maps/classroom.yaml
```
1.2 标点
```
ros2 topic echo /clicked_point
```

### 2.单独启动
1. rviz里显示机器人(非必要步骤，仅调试urdf时可方便查看)
- 参数：model
```
ros2 launch rm_bnrobot_sim robot_sim.launch.py
```
2. 在gazebo中启动仿真 
- 参数：model:模型路径 world：地图路径
```
ros2 launch rm_bnrobot_sim gazebo_sim.launch.py
```
3. 启动nav导航  
- 参数：map, use_sim_time , param_file
```
ros2 launch rm_bnrobot_nav nav_load.launch.py
```
4. 初始化坐标
```
ros2 run rm_bnrobot_function init_pose.py
```
5. 获取坐标 
```
ros2 run rm_bnrobot_function get_pose.py 
```
6. 导航到指定点 
- 参数：goal.x goal.y goal.z:目标点的x,y,z坐标
```
ros2 run rm_bnrobot_function nav_pose.py 
```

