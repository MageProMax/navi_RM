# 基于 ROS 2 和 Nav2 的哨兵自动巡检

## author 牢🐴

## question / 待验证

- 扫描高度：先验导航使用 z=0.10～0.50 m；探索导航使用 z=0.10～1.50 m，并进行多帧运动补偿。实机需结合实际障碍高度检查。
- 连续避障：需要验证多次遇障后的重规划、恢复行为与任务继续执行。
- 上层规划：需要继续验证裁判信息、决策目标与 Nav2 导航任务的衔接。

## 1. 项目介绍

该项目是基于 ROS 2（肉丝2）和 Nav2 的哨兵导航，支持实机和 Gazebo 仿真。

当前有两种地图链路，启动时通过 `navigation_mode` 选择：

```text
Mid-360 点云 + IMU → FAST-LIO：车体点云、odom → base_link
        ├── prior：配对 PCD / 二维地图 → GICP 重定位 → map → odom
        └── explore：三维关键帧 → GICP 回环＋PGO → 重建二维地图、map → odom
                         ↓
Nav2：全局规划 + MPPI 全向局部控制
                         ↓
/nav_cmd_vel → 自旋参考系适配 → /cmd_vel
                         ↓
共享 CAN 仲裁：受击自旋、急停、超时、0x120 编码
        ├── 实机：USB-CAN → MCU
        └── 仿真：同一载荷解码 /cmd_vel_sim → Gazebo
```

有先验地图时使用 **FAST-LIO + 三维 GICP 重定位 + MPPI**；没有地图时使用 **在线建图 + 自动探索**，也可以指定未知目的地逐段探索到达。探索默认使用 FAST-LIO＋三维 GICP 回环＋PGO 位姿图优化，并按优化后的历史关键帧重建二维地图。二维投影扫描不再承担回环匹配；回环修图期间取消旧路径并暂停运动，新地图就绪后重新规划。实现、参数与限制见 [在线三维回环建图](docs/online_pgo_mapping_20261015.md)。

保留了 AMCL 二维定位启动方式。仿真使用 Gazebo Classic，关节状态由 `ros2_control` 管理，底盘速度由平面运动插件执行。

### 1.1 定位与控制

| 项目 | 当前配置 |
|---|---|
| 激光惯性里程计 | FAST-LIO |
| 三维重定位 | 粗到细两阶段 GICP，0.5 秒运动补偿点云累积，可切回 ICP |
| 重定位更新频率 | 调度上限 5 Hz，实际频率受计算耗时限制 |
| 局部控制器 | `nav2_mppi_controller::MPPIController` |
| 运动模型 | `Omni`，全向运动 |
| 控制频率 | 20 Hz，`model_dt=0.05 s` |
| 先验导航障碍切片 | `base_link` 下 z=0.10～0.50 m |
| 探索建图／避障切片 | z=0.10～1.50 m；建图累积 1.0 s，避障累积 0.6 s |
| 探索全局规划器 | `SmacPlanner2D`，禁止直接穿越未知栅格 |
| 实机碰撞外形 | 平移按 0.633×0.526 m 矩形、每侧余量 0.02 m；旋转按 0.47 m 扫掠半径，近墙暂停自旋，见 [贴墙通行](docs/close_wall_navigation_20261022.md) |
| 局部膨胀衰减 | `cost_scaling_factor=30.0`，`inflation_radius=0.5 m` |

三维定位模式的 TF 分工：

```text
map ── localizer ──> odom ── FAST-LIO ──> base_link ── URDF ──> 雷达/IMU
```

先验模式由 localizer、探索模式由 online_pgo 独占发布 `map → odom`；两者互斥，不要额外启动旧 PGO 或 slam_toolbox 发布同一段 TF。三维导航不启动 AMCL。Gazebo 的 `/odom` 仅用于真值对照；三维定位使用
`/fastlio2/lio_odom`，Nav2 使用适配后的 `/navigation/odom` 和 `nav_base_link`。

先验定位默认使用 GICP，并以最新车体坐标累积最近 0.5 秒点云。实机／仿真一键入口支持 `localizer_backend:=gicp` 和 `localizer_accumulation_seconds:=0.5`；对照测试可设为 `icp` 和 `0.0`。查看 `ros2 topic echo /localizer/registration_status` 可检查匹配是否接受、内点比例和计算耗时。实现、回退与验证边界见 [GICP 定位升级说明](docs/gicp_localization_20261015.md)。该升级只作用于先验导航。

### 1.2 雷达安装参数

实机和仿真均以车体中心竖直轴的地面投影为 `base_link` 原点，x 向前、y 向左、z 向上。

| 项目 | 实机与仿真配置 |
|---|---|
| 雷达 FOV 原点到车体中心竖直轴的半径 | 221.57 mm |
| 雷达 FOV 原点离地高度 | 330.18 mm |
| FOV 原点位置 | `[0, 0.22157, 0.33018] m`，位于车体左侧，无前向偏移 |
| 雷达坐标轴方向 | x 轴朝车头正前方，z 轴朝左上方，y 轴朝左下方（右手系） |
| 安装倾角 | 沿用约 45°，`roll=-0.7854 rad, pitch=0, yaw=0` |

这里的位置指 **FOV 原点**，不是 STL 模型的视觉中心，也不是雷达内部 IMU 原点。
FAST-LIO 的车体到 IMU 外参与 IMU 到雷达外参组合后，与上述安装位置一致。
对应 IMU 原点为 `[0.011000000, 0.206840878, 0.282513959] m`；实机／仿真模型、在线建图射线起点、动态点过滤与离线地图重建均使用同一左侧安装位置。
软件配置已核对，实机机械尺寸以实际测量为准。

## 2. 功能包介绍

| 功能包 | 用途 |
|---|---|
| `rm_bnrobot_sim` | 机器人模型、Gazebo world、传感器与底盘仿真 |
| `rm_bnrobot_nav` | 仿真导航启动、地图与仿真 FAST-LIO 参数 |
| `rm_bnrobot_function` | 导航目标客户端、位姿读取、地图转换等工具 |
| `rm_interfaces` | 目标点等自定义接口 |
| `interface` | PCD 保存、重定位等服务接口 |
| `livox_ros_driver2` | 实机 Mid-360 雷达驱动 |
| `ros2_livox_simulation` | Livox 仿真插件；源码位于 `src/livox_laser_simulation_ros2` |
| `fastlio2` | FAST-LIO 激光惯性里程计 |
| `localizer` | PCD 地图三维 GICP 重定位 |
| `pgo` / `hba` | 三维回环、历史轨迹优化、地图重建与保存；HBA 离线优化 |
| `real_bringup` / `sim_bringup` | 实机与仿真启动；源码位于 `src/rm_bringup` |
| `can_twist` | 实机 CAN 速度下发，执行文件 `can_twist_node` |
| `can_control` | USB-CAN 下发，执行文件 `can_control` |
| `nav_plan` | 上层目标与裁判相关逻辑 |
| `rm_plan` | 保留的哨兵决策与测试启动 |

## 3. 环境与安装依赖

- 系统：Ubuntu 22.04。
- ROS：ROS 2 Humble。
- 本项目当前使用 Conda 环境：`ros2_env`。
- 以下命令均在项目根目录执行，每个新终端都需要加载环境。

### 3.1 基本依赖

```bash
sudo apt install ros-humble-nav2-bringup ros-humble-nav2-mppi-controller \
  ros-humble-nav2-smac-planner \
  ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan

sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro ros-humble-tf-transformations \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-robot-localization

sudo apt install build-essential cmake pkg-config python3-colcon-common-extensions \
  python3-rosdep libpcl-dev libeigen3-dev libyaml-cpp-dev libfmt-dev \
  libopencv-dev qtbase5-dev libusb-1.0-0-dev
```

FAST-LIO、回环与雷达驱动还需要 Sophus、GTSAM 和 Livox-SDK2。
工作区已包含部分依赖源码，详细说明见 [FASTLIO2 ROS2 README](src/rm_lio/FASTLIO2_ROS2/README.md)。

地图重建脚本使用 NumPy、SciPy、Pillow、PyYAML；`tools/build_prior_map.py`、`pcd2map.py`、`add_normal.py` 还使用 Open3D。

```bash
conda activate ros2_env
python -m pip install numpy scipy pillow pyyaml open3d opencv-python transforms3d PyQt5
```

### 3.2 编译与加载环境

```bash
conda activate ros2_env
source /opt/ros/humble/setup.bash

# 首次配置或新增依赖时使用
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

colcon build --parallel-workers 2
source install/setup.bash
```

以后每次打开终端：

```bash
conda activate ros2_env
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 4. 接口

| Topic name | Type | Note |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 当前驱动与仿真使用的雷达消息 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 实机或仿真 IMU |
| `/fastlio2/body_cloud` | `sensor_msgs/msg/PointCloud2` | 车体坐标下的点云，供重定位与切片使用 |
| `/fastlio2/world_cloud` | `sensor_msgs/msg/PointCloud2` | FAST-LIO 世界坐标下的点云 |
| `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | 真实车体里程计，供定位及自旋适配器使用 |
| `/navigation/odom` | `nav_msgs/msg/Odometry` | 仿真／实机共用的固定朝向导航里程计 |
| `/nav_cmd_vel` | `geometry_msgs/msg/Twist` | 三维导航参考系的平移速度 |
| `/spin_active` | `std_msgs/msg/Bool` | CAN 仲裁当前是否正在执行受击自旋 |
| `/emergency_stop` | `std_msgs/msg/Bool` | true 停止平移和自旋 |
| `/localizer/map_cloud` | `sensor_msgs/msg/PointCloud2` | 三维参考地图可视化 |
| `/pgo/status` | `std_msgs/msg/String` | 探索三维回环次数、候选验证状态和地图修正版本 |
| `/pgo/map_ready` | `std_msgs/msg/Bool` | 探索地图就绪许可；修图期间暂停平移和自旋 |
| `/pgo/optimized_path` | `nav_msgs/msg/Path` | 优化后的探索关键帧轨迹 |
| `/localizer/registration_status` | `std_msgs/msg/String` | 配准 JSON：后端、是否接受、累积帧数、有效匹配比例及耗时 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 点云高度切片生成的导航扫描 |
| `/scan_2d_raw` | `sensor_msgs/msg/LaserScan` | 仿真额外的二维雷达调试扫描 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 二维占据地图 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 底盘速度指令 |
| `/goal_pose` | `rm_interfaces/msg/Goal` | 三维完整启动中，正式模式接收的目标点 |
| `/targetPose` | `rm_interfaces/msg/Goal` | 单独运行 `nav_pose` 时的默认输入 |
| `/navigation/client/status` | `rm_interfaces/msg/NavigationStatus` | 客户端模式、目标、执行／等待状态 |
| `/navigation/exploration_target` | `rm_interfaces/msg/Goal` | 未知目的地的探索意图 |
| `/navigation/exploration_goal` | `rm_interfaces/msg/Goal` | 探索器选择的已知可达中间点 |
| `/exploration/status` | `std_msgs/msg/String` | 探索进度和最终目的地状态 |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | RViz 标点 |

当前实机驱动的 `xfer_format=1`，使用 `CustomMsg`；不要默认认为 `/livox/lidar` 是 `PointCloud2`。
实际接口可用 `ros2 topic list -t` 查看。

## 5. 运行

### 导航客户端界面

下面四套导航完整一键指令均设置 `use_ui:=true`，同时启动客户端界面；5.9 节的仅建图入口只打开 RViz，不启动导航客户端。已运行导航时，另开界面可执行：

```bash
ros2 launch rm_bnrobot_function navigation_ui.launch.py use_sim_time:=false
```

附加到仿真时改为 `use_sim_time:=true`。界面支持实时地图、车体位置、巡检文件编辑、开始／暂停／取消／急停。编辑目标后直接点击开始，会先应用当前配置；单独应用配置会暂停。探索模式中，双击已知区域切到手动导航，双击未知区域切到“探索到目标”，逐段建图前往。详见 [导航客户端界面](docs/navigation_ui.md)。

### 导航模式选择与完整参数

以下“一键启动”均在项目根目录执行，先按 3.2 节加载 `ros2_env`、ROS 和工作区环境。选择一条启动；切换前停止旧入口。

| 环境／任务 | 完整一键指令 |
|---|---|
| 仿真：无先验地图，在线建图并自动探索（推荐测试） | [仿真探索](#sim-explore-full) |
| 仿真：已有地图，GICP 定位并测试巡检 | [仿真先验巡检](#sim-prior-full) |
| 实机：已有地图，GICP 定位并正式决策 | [实机先验导航](#real-prior-full) |
| 实机：无先验地图，在线建图并自动探索 | [实机探索](#real-explore-full) |
| 实机：仅建图，使用底盘遥控采集 | [实机仅建图](#mapping-only-real) |
| 实机：新场地地图及巡检文件 | [7.2.8 完整启动](#real-site-full) |

完整命令列出对应模式使用的可配置参数；不适用该模式、父入口固定覆盖的内部参数和空命名空间不写入启动命令。下表保留这些参数的含义，便于查阅 `--show-args`。路径使用 `$PWD/src/...`，编辑这些 YAML 后重启即可加载；修改编译源码、模型依赖或新增依赖仍需重新编译。

| 参数 | 示例／默认 | 生效范围或说明 |
|---|---|---|
| `navigation_mode` | `prior` / `explore` | 先验定位／三维回环探索；界面不能在线切换此链路 |
| `world` | `custom_room.world` | 仅仿真，场景文件 |
| `model` | `bnbot.urdf.xacro` / `bnbot_real.xacro` | 仿真／实机模型文件 |
| `map` | `room.yaml` | 仅先验，二维地图 |
| `pcd_map` | `simmap_normal.pcd` | 仅先验，与二维地图配对的三维地图 |
| `initial_pose_x` | `-1.96` m | 仿真出生位置；先验模式同时用作定位初值 |
| `initial_pose_y` | `-1.85` m | 同上 |
| `initial_pose_yaw` | `0.0` rad | 仿真出生朝向／先验定位初始朝向 |
| `localizer_backend` | `gicp` / `icp` | 仅先验，地图定位后端；不控制探索回环 |
| `localizer_accumulation_seconds` | `0.5` s | 仅先验，点云累积窗口；`0.0` 为单帧 |
| `param_file` | `real_nav2_params_3d.yaml` | 实际用于导航与先验定位的原始配置文件，注意是单数 `param` |
| `lio_config` | `lio_sim.yaml` / `lio.yaml` | 仿真／实机 FAST-LIO 外参、噪声和前端配置 |
| `pgo_params_file` | `exploration_pgo.yaml` | 仅探索，三维回环、关键帧和地图重建配置 |
| `exploration_params_file` | `exploration.yaml` | 仅探索，前沿选择、净空、失败避让等配置 |
| `slam_params_file` | `exploration_slam.yaml` | 旧二维建图兼容参数，当前三维探索不读取 |
| `patrol_mode` | `formal` / `test` | 仅先验，正式决策／按列表巡检 |
| `patrol_file` | `custom_room.yaml` / `custom_room_test.yaml` | 仅先验，必须与 `patrol_mode` 及地图匹配；探索忽略 |
| `spin_speed` | `1.0` rad/s | 受击自旋速度 |
| `spin_duration` | `10.0` s | 单次受击自旋时长 |
| `gui` | 命令中 `true`，launch 默认 `false` | 仅仿真，Gazebo 窗口 |
| `server` | `true` | 仅仿真，Gazebo 服务端；完整仿真保持开启 |
| `use_rviz` | `true` | 打开 RViz |
| `use_ui` | 命令中 `true`，launch 默认 `false` | 打开导航客户端界面 |
| `log_level` | `info` | Nav2 日志等级 |
| `use_respawn` | `False` | Nav2 非组合节点崩溃后是否重启，不等于完整系统故障恢复 |
| `namespace` | 空字符串 | 当前单车链路使用根命名空间，不能只改此值就完成多机器人隔离 |
| `container_name` | `nav2_container` | 组合容器名称；当前固定非组合模式，此项不生效 |
| `use_sim_time` | 仿真 `true`，实机 `false` | **父入口固定**，按环境使用对应时钟 |
| `use_fast_lio` | `true` | **父入口固定**，完整链路包含 FAST-LIO |
| `use_nav_plan` | `true` | **父入口固定**；实际只有 `prior + formal` 启动正式决策 |
| `backend` | 仿真 `sim`，实机 `real` | **父入口固定**，选择 dry_run 仿真桥或真实 CAN |
| `cmd_vel_topic` | `/cmd_vel_sim` | **仿真父入口固定**，Gazebo 从共享 CAN 解码桥接收速度 |
| `autostart` | `true` | **共享核心固定**，自动激活 Nav2；不表示取消就绪检查 |
| `use_composition` | `False` | **共享核心固定**，使用可恢复生命周期管理的独立节点 |
| `params_file` | 内部生成的重写 YAML | **共享核心固定覆盖**，复数 `params`；实际传给 Nav2 的是按模式重写后的副本，无需在命令中设置；调整配置用 `param_file` |

探索命令省略先验地图、GICP 定位和巡检参数，不会加载旧地图或执行旧巡检点。实机探索的地图坐标以本次建图初始化为参考，无需设置先验初始坐标；仿真仍通过 `initial_pose_x/y/yaw` 设置出生位姿。当前探索由 online_pgo 独占发布 `/map` 和 `map → odom`，回环修图期间取消旧路径并暂停运动，就绪后重新规划。配置与保存指令见 [在线三维回环建图](docs/online_pgo_mapping_20261015.md)。目标探索详见 [未知目的地探索导航](docs/guided_exploration_20261015.md)。

### 5.1 仿真一键启动（完整参数）

<a id="sim-explore-full"></a>

#### 仿真探索建图（一键完整参数，推荐测试）

```bash
ros2 launch sim_bringup sim_nav_3d_total.launch.py \
  navigation_mode:=explore \
  pgo_params_file:="$PWD/src/rm_bringup/real_bringup/config/exploration_pgo.yaml" \
  exploration_params_file:="$PWD/src/rm_bringup/real_bringup/config/exploration.yaml" \
  use_ui:=true \
  spin_speed:=1.0 \
  spin_duration:=10.0 \
  world:="$PWD/src/rm_bnrobot_sim/world/custom_room.world" \
  model:="$PWD/src/rm_bnrobot_sim/urdf/bngu_sentinel/bnbot.urdf.xacro" \
  gui:=true \
  use_rviz:=true \
  initial_pose_x:=-1.96 \
  initial_pose_y:=-1.85 \
  initial_pose_yaw:=0.0 \
  server:=true \
  param_file:="$PWD/src/rm_bringup/real_bringup/config/real_nav2_params_3d.yaml" \
  lio_config:="$PWD/src/rm_bnrobot_nav/config/lio_sim.yaml" \
  use_respawn:=False \
  log_level:=info
```

启动 `custom_room.world`、机器人、FAST-LIO、三维 GICP 回环＋PGO、地图重建、Nav2、自动探索、客户端界面、RViz、Gazebo 和 CAN dry_run 桥。有效扫描到来后开始建图，地图与底盘就绪后自动探索；也可在界面标记未知目的地，点击开始逐段前往。

界面暂停移动不会停止建图。回环需要重新观测历史区域并通过质量及连续确认门限。地图以本次建图起点为参考，不强制与 Gazebo world 原点重合。

<a id="sim-prior-full"></a>

#### 仿真先验巡检（一键完整参数）

```bash
ros2 launch sim_bringup sim_nav_3d_total.launch.py \
  navigation_mode:=prior \
  localizer_backend:=gicp \
  localizer_accumulation_seconds:=0.5 \
  use_ui:=true \
  patrol_mode:=test \
  patrol_file:="$PWD/src/nav_referee/nav_plan/config/patrol/custom_room_test.yaml" \
  spin_speed:=1.0 \
  spin_duration:=10.0 \
  world:="$PWD/src/rm_bnrobot_sim/world/custom_room.world" \
  model:="$PWD/src/rm_bnrobot_sim/urdf/bngu_sentinel/bnbot.urdf.xacro" \
  map:="$PWD/src/rm_bnrobot_nav/maps/room.yaml" \
  pcd_map:="$PWD/src/rm_bnrobot_nav/maps/pcd/simmap_normal.pcd" \
  gui:=true \
  use_rviz:=true \
  initial_pose_x:=-1.96 \
  initial_pose_y:=-1.85 \
  initial_pose_yaw:=0.0 \
  server:=true \
  param_file:="$PWD/src/rm_bringup/real_bringup/config/real_nav2_params_3d.yaml" \
  lio_config:="$PWD/src/rm_bnrobot_nav/config/lio_sim.yaml" \
  use_respawn:=False \
  log_level:=info
```

先验模式在定位及底盘就绪后按测试文件自动巡检。正式模式需同时改为 `patrol_mode:=formal` 和 `patrol_file:="$PWD/src/nav_referee/nav_plan/config/patrol/custom_room.yaml"`。

两条命令都通过 CAN dry_run 桥向 Gazebo 的 `/cmd_vel_sim` 下发，保留受击自旋。无窗口运行时，在完整命令里将 `gui`、`use_rviz`、`use_ui` 都改为 `false`；更换出生位置时修改 `initial_pose_x/y/yaw`，先验定位初值随之保持一致。链路说明见 [仿真与实机一致性检查](docs/sim_real_parity_20261014.md)。

### 正式决策与测试巡检目标文件

仿真、实机三维入口均启动同一个 `nav_pose` 客户端。默认 `patrol_mode:=formal`，按比赛状态选择补给／占领点；`patrol_mode:=test` 按列表自动循环巡检，两种模式只启用一个目标来源。

使用 5.1 节完整先验命令中的 `patrol_mode` 和 `patrol_file` 选择测试／正式目标文件；新场地实机使用 7.2.8 节的完整命令。更换地图时一并更换配对 PCD、定位初值和巡检文件。

默认正式文件为 [custom_room.yaml](src/nav_referee/nav_plan/config/patrol/custom_room.yaml)，测试文件为 [custom_room_test.yaml](src/nav_referee/nav_plan/config/patrol/custom_room_test.yaml)。文件格式、单轮巡检、停留时间和验证结果见 [巡检配置说明](docs/patrol_navigation.md)。目标坐标须与选用地图的 map 坐标系一致。

### 5.2 实机一键启动（完整参数）

命名空间保持默认空值，命令中省略 `namespace`；不要传入 `namespace:=""`，Shell 会将它变成无值的 `namespace:=`，导致 ROS 2 拒绝启动。

下面两条实机导航入口都包含 Livox 雷达驱动。启动前停止单独运行的 `msg_MID360_launch.py`，避免重复启动驱动。自动探索会下发运动指令；使用遥控器手动采图时执行 [5.9 实机遥控建图](#mapping-only-real)。复制命令时，续行符 `\` 必须在行末且后面没有空格，最后一行不加续行符。

<a id="real-prior-full"></a>

#### 实机先验导航（一键完整参数）

下面完整展示当前默认 `custom_room` 配对地图及正式文件。新场地需先完成 7.2 节，并使用 [7.2.8 新场地完整命令](#real-site-full)；地图、定位初值和巡检文件必须对应实际场地。

```bash
ros2 launch real_bringup real_nav_3d_total.launch.py \
  navigation_mode:=prior \
  localizer_backend:=gicp \
  localizer_accumulation_seconds:=0.5 \
  use_ui:=true \
  patrol_mode:=formal \
  patrol_file:="$PWD/src/nav_referee/nav_plan/config/patrol/custom_room.yaml" \
  spin_speed:=1.0 \
  spin_duration:=10.0 \
  map:="$PWD/src/rm_bnrobot_nav/maps/room.yaml" \
  pcd_map:="$PWD/src/rm_bnrobot_nav/maps/pcd/simmap_normal.pcd" \
  use_rviz:=true \
  initial_pose_x:=-1.96 \
  initial_pose_y:=-1.85 \
  initial_pose_yaw:=0.0 \
  param_file:="$PWD/src/rm_bringup/real_bringup/config/real_nav2_params_3d.yaml" \
  lio_config:="$PWD/src/rm_lio/FASTLIO2_ROS2/fastlio2/config/lio.yaml" \
  model:="$PWD/src/rm_bnrobot_sim/urdf/bngu_sentinel/bnbot_real.xacro" \
  use_respawn:=False \
  log_level:=info
```

正式模式按比赛状态选择目标。测试巡检需同时改为 `patrol_mode:=test` 和 `patrol_file:="$PWD/src/nav_referee/nav_plan/config/patrol/custom_room_test.yaml"`，新场地替换为本场地测试文件。

<a id="real-explore-full"></a>

#### 实机探索建图（一键完整参数）

```bash
ros2 launch real_bringup real_nav_3d_total.launch.py \
  navigation_mode:=explore \
  pgo_params_file:="$PWD/src/rm_bringup/real_bringup/config/exploration_pgo.yaml" \
  exploration_params_file:="$PWD/src/rm_bringup/real_bringup/config/exploration.yaml" \
  use_ui:=true \
  spin_speed:=1.0 \
  spin_duration:=10.0 \
  use_rviz:=true \
  param_file:="$PWD/src/rm_bringup/real_bringup/config/real_nav2_params_3d.yaml" \
  lio_config:="$PWD/src/rm_lio/FASTLIO2_ROS2/fastlio2/config/lio.yaml" \
  model:="$PWD/src/rm_bnrobot_sim/urdf/bngu_sentinel/bnbot_real.xacro" \
  use_respawn:=False \
  log_level:=info
```

两条实机命令均包含雷达驱动、FAST-LIO、Nav2、客户端、RViz 和真实 CAN 仲裁；探索模式使用在线 PGO，不启动先验定位器和正式决策。无需额外启动 `can_twist_node` 或 `can_control`。仲裁默认在约 25 秒后启动，客户端此前等待底盘就绪。

实机雷达配置位于 `src/livox_ros_driver2/config/MID360_config.json`。当前主机 IP 为 `192.168.1.5`、雷达 IP 为 `192.168.1.154`，需与现场网络匹配。驱动外参保持零值，安装变换由 URDF 与 FAST-LIO 配置处理。其中 `extrinsic_parameter.x/y/z` 必须写整数 `0`，不能写浮点数 `0.0`，否则当前驱动的 `GetInt()` 会触发断言；`roll/pitch/yaw` 可写 `0.0`。

### 5.3 实机分步启动

每条命令在分别加载好环境的终端中运行：

```bash
# 1. 启动雷达
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 2. 启动三维定位与导航，不重复启动雷达
ros2 launch real_bringup real_nav_3d.launch.py

# 3. 启动与一键入口一致的底盘仲裁，保留受击自旋和超时规则
ros2 launch real_bringup motion_arbiter.launch.py backend:=real use_sim_time:=false

# 4. 获取位姿
ros2 run rm_bnrobot_function get_pose.py
```

### 5.4 使用自定义地图

三维重定位使用的 PCD 与二维地图必须共享相同的坐标原点、朝向和尺度。

```bash
# 将文件放在当前工作区 maps 目录后，根据实际文件名修改
ros2 launch real_bringup real_nav_3d.launch.py \
  map:="$PWD/src/rm_bnrobot_nav/maps/room.yaml" \
  pcd_map:="$PWD/src/rm_bnrobot_nav/maps/pcd/simmap_normal.pcd"
```

| 环境 | 默认二维地图 | 默认 PCD |
|---|---|---|
| 仿真三维定位 | `maps/room.yaml` | `maps/pcd/simmap_normal.pcd` |
| 实机三维定位（当前先使用仿真地图） | `maps/room.yaml` | `maps/pcd/simmap_normal.pcd` |

当前实机三维入口默认初始位置与 custom_room 仿真一致：`x=-1.96, y=-1.85, yaw=0`。
车的位置不同，可通过 `initial_pose_x`、`initial_pose_y`、`initial_pose_yaw` 启动参数覆盖，
或调用重定位服务。实机现场必须与参考地图的布局、坐标和尺度相符才能正确定位。
原 `map.yaml/test.pcd` 保留为历史地图，尚不建议直接部署。

### 5.5 保留的 AMCL 二维定位模式

```bash
# 仿真
ros2 launch sim_bringup sim_nav_total.launch.py gui:=true

# 实机：先另行启动雷达
ros2 launch real_bringup real_nav.launch.py
```

AMCL 模式使用二维地图完成定位；不要与三维定位模式同时启动。
`init_pose.py` 用于 AMCL 初始位姿，三维定位使用 `localizer` 的重定位接口。

### 5.6 单独调试

```bash
# 显示机器人模型
ros2 launch rm_bnrobot_sim robot_sim.launch.py

# 只启动 Gazebo 和机器人
ros2 launch rm_bnrobot_sim gazebo_sim.launch.py gui:=true

# 在已启动的 Gazebo 上启动三维导航
ros2 launch rm_bnrobot_nav nav_load_3d.launch.py

# RViz 标点
ros2 topic echo /clicked_point
```

### 5.7 发送导航目标

三维入口已经启动 `nav_pose`，不要再启动第二个客户端。优先在界面中选手动导航／测试巡检，编辑点后开始；在探索链路中标未知点则使用“探索到目标”。RViz 的 Publish Point 只发布 `/clicked_point`，用下面命令读取坐标，不会自动发送目标：

```bash
ros2 topic echo /clicked_point
```

正式模式且未运行上层决策时，可发布 `/goal_pose`（数值替换成当前 map 下的实测点）：

```bash
ros2 topic pub --once /goal_pose rm_interfaces/msg/Goal \
  "{goal_frame_id: 'map', goal_x: 0.0, goal_y: 0.0, goal_z: 0.0}"
```

手动／测试／探索模式会忽略正式目标来源。单独调试客户端时才执行 `ros2 run rm_bnrobot_function nav_pose`，其默认输入是 `/targetPose`；`nav_pose` 是 C++ 执行文件，不是 `nav_pose.py`。

### 5.8 GICP 定位参数与 ICP 对照

实机和仿真先验导航默认使用 **PCL GICP＋0.5 秒运动补偿点云累积**。每帧点云通过对应 LIO 位姿转换到最新车体坐标，再计算局部协方差进行配准。该后端借鉴 PB 的设计，使用的库为 PCL；不需要另外安装 small_gicp，也不要求已有 PCD 含法向量。

| 启动参数 | 默认值 | 用途 |
|---|---|---|
| `localizer_backend` | `gicp` | `gicp` 为协方差配准，`icp` 为点到点配准 |
| `localizer_accumulation_seconds` | `0.5` | 累积时间窗口；`0.0` 使用单帧 |

这两个启动参数覆盖定位器 YAML 中的 `registration_method`、`accumulation_seconds`。其他定位参数在 `src/rm_bringup/real_bringup/config/real_nav2_params_3d.yaml` 的 `/localizer/localizer_node` 下设置：最大累积 10 帧、每帧预降采样 0.08 m、协方差邻居数 20、精匹配最大对应距离 0.5 m、最小有效匹配比例 0.5。修改启动参数后需重启导航；修改源码配置文件后需重新编译安装对应包并加载环境。

仿真使用单帧 ICP 做对照时，在 5.1 节完整先验命令里同时设置 `localizer_backend:=icp` 和 `localizer_accumulation_seconds:=0.0`，其他参数保持同一场景配置。

实机使用同样参数，附加到第 7.2.8 节的新场地启动命令即可，保留该场地的地图、初始位姿和巡检文件参数。ICP 对照仍保留本次新增的匹配距离、有效匹配比例和异常修正保护。`navigation_mode:=explore` 不启动该定位器，这两个参数不改变探索定位链路。

本次 custom_room 仿真往返中，两种方案均完成 2/2 个目标，受击自旋期间能继续导航：

| 指标 | 单帧 ICP | GICP＋0.5 秒累积 |
|---|---:|---:|
| 两次到点时位置估计与真值快照之差 | 14.5 / 6.9 cm | 11.2 / 5.6 cm |
| 两次实际到点误差 | 32.1 / 21.5 cm | 28.7 / 20.1 cm |
| 配准耗时 P95 | 33.3 ms | 78.1 ms |

两种方案各测试一次，以上不是整段轨迹误差统计，也不能视为实机精度保证。GICP 不能补足弱特征区域缺失的几何约束，多帧累积也依赖 LIO 的正确性。完整实现、参数、测试证据与限制见 [GICP 定位升级说明](docs/gicp_localization_20261015.md)。

探索回环配置通过 `pgo_params_file` 指定，默认 `exploration_pgo.yaml`；旧的 `slam_params_file` 不控制新的探索后端。详细参数和保存指令见 [在线三维回环建图](docs/online_pgo_mapping_20261015.md)。

### 5.9 实机遥控建图，不启动导航（一键完整参数）

本模式用于在真实场地用遥控器开车采集地图。启动 Mid-360 驱动、FAST-LIO、三维回环 PGO、机器人安装 TF 和 RViz；不启动 Gazebo、Nav2、探索器、巡检客户端或本项目的 CAN 导航下发节点。底盘沿用原有遥控控制，电脑只负责接收雷达数据和建图。

先关闭旧导航／建图入口，在项目根目录按 3.2 节加载 `ros2_env` 环境。入口、静态地图过滤和 RViz 配置首次更新后执行：

```bash
colcon build --packages-select pgo real_bringup
source install/setup.bash
```

<a id="mapping-only-real"></a>

#### 实机遥控建图启动指令

以下列出实机建图使用的全部参数。入口默认就是 `backend:=real`，使用真实时钟。

```bash
ros2 launch real_bringup mapping_3d.launch.py \
  backend:=real \
  use_rviz:=true \
  start_lidar:=true \
  model:="$PWD/src/rm_bnrobot_sim/urdf/bngu_sentinel/bnbot_real.xacro" \
  lio_config:="$PWD/src/rm_lio/FASTLIO2_ROS2/fastlio2/config/lio.yaml" \
  pgo_params_file:="$PWD/src/rm_bringup/real_bringup/config/exploration_pgo.yaml" \
  rviz_config_file:="$PWD/src/rm_bringup/real_bringup/rviz/mapping_3d.rviz"
```

`model` 位于模型资源包中，用于发布实机安装 TF，不会启动仿真。`lio_config`、`pgo_params_file` 分别控制前端与回环，`rviz_config_file`、`use_rviz` 控制显示。已单独运行雷达驱动时设置 `start_lidar:=false`，避免重复启动；雷达网络地址按 7.2.1 节检查。

启动后先让车静止至少 5 秒，等待 FAST-LIO 完成 IMU 初始化，并在 RViz 中确认点云和地图正常更新，再用遥控器缓慢行驶。经过门口、转角时放慢速度，最后回到先前经过的区域，给回环提供重访观测。建图坐标由首次初始化建立，不需要填写导航初始坐标或巡检目标点。

#### 保存二维地图、PCD 和关键帧

先用遥控器停车，保持建图程序运行。人离开后继续扫描到其原位置及后方环境，等待过滤状态和 `/pgo/map_ready`，再另开已加载环境的终端保存：

```bash
export MAPPING_DIR="$PWD/maps_output/mapping_only_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$MAPPING_DIR/pgo"

# 一次服务同时保存同一快照的静态 PCD 和二维地图
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$MAPPING_DIR/pgo', save_patches: true}"
```

返回 `success: true` 后，`pgo/` 下包含配对的 `map.pcd`、`map.pgm`、`map.yaml` 和 `start_pose.env`；`patches/`、`poses.txt`、`frame_times.txt` 保留原始观测及优化位姿，便于重新过滤。`dynamic_filter.yaml` 保存本次过滤配置，`filter_metadata.yaml` 记录删点数量及是否触及资源上限。`map_saver_cli` 仍能单独保存当前 `/map`，部署先验导航时使用本服务一次生成的配对文件，避免两次保存跨越地图更新。

```bash
ros2 topic echo /pgo/static_filter_status --qos-durability transient_local
ros2 topic echo /pgo/map_ready --qos-durability transient_local
```

三维多帧一致性过滤默认开启：体素 0.10 m，至少 3 次独立空闲观测，空闲／占用证据比至少 3:1。射线只在真实返回之前的三维空间提供证据，端点前保护 0.20 m，按实际端点的三维位置保留邻近表面；不能通过抬高扫描高度或简单删稀疏点来代替。停车时每秒保存一次辅助观测，最近观测绑定到图优化关键帧，回环后同步重算。实时 `/fastlio2/body_cloud`、避障 `/scan` 和 `/scan_mapping` 不删行人。

未被重新看见、始终遮挡或一直站着不动的人仍可能留下，需要补扫；资源预算耗尽时保守保留全部点并报告 `*_budget_retained_all`，不能把此状态当作已经完成清理。详细参数与验证见 [静态地图动态点清理](docs/static_map_filter_20261015.md)。已有关键帧按 7.2.4 节重新生成时，也默认调用相同 C++ 过滤器；使用 HBA 新位姿时会先按新位姿重新判断，再同时生成 PCD 与二维图。

回环需要实际移动后重访已观测区域并通过确认条件，原地静止不能验证闭环效果。更多参数见 [在线三维回环建图](docs/online_pgo_mapping_20261015.md)。

### 5.10 无路径自转与占墙退让

仿真和实机的三维先验／探索导航使用相同恢复逻辑，默认启用，无需修改原启动指令。仅建图入口不启动这套自动动作。

- 有导航任务但连续 8 秒未前进至少 0.20 m：确认实时雷达的旋转净空后，以 0.35 rad/s 原地转约 180°，最长 12 秒，再重新规划原目标。既处理无路径，也处理有路径但控制器一直走不动。
- 局部代价地图认为车体轴心落在碰撞区，持续 1 秒：从各方向选择已观测、实时雷达有净空、能够减小地图碰撞深度的空地，以 0.08 m/s 退让。支持麦轮横移；离开碰撞区即停，单次最多 0.40 m／8 秒。方向由空地位置决定。
- 恢复时取消当前 Nav2 动作，结束后保留同一个目标重新发送；探索器暂停切换目标。两次恢复间隔至少 8 秒，无正常前进时最多尝试 3 次。
- 暂停、急停、地图回环修正、雷达／里程计／代价地图过期都会中止恢复。未知或雷达未观测的方向不作为退让通道；没有安全方向时停车。保留受击自旋功能，退让期间暂时禁止旋转，防止贴墙转动。

参数位于 `src/rm_bringup/real_bringup/config/real_nav2_params_3d.yaml` 的 `spin_navigation_adapter.ros__parameters`，通过原启动命令中的 `param_file` 加载，修改后重启导航：

| 参数（均带 `recovery.` 前缀） | 默认值 | 含义 |
|---|---|---|
| `enabled` | `true` | 导航入口启用自动恢复 |
| `no_path_timeout` | `8.0` | 无有效进展的等待时间，秒 |
| `progress_distance` | `0.20` | 重置无进展计时／重试次数所需平移，米 |
| `overlap_timeout` | `1.0` | 占墙确认时间，秒 |
| `spin_speed` | `0.35` | 恢复自转速度，rad/s；与受击自旋参数分开 |
| `spin_angle` | `3.141592653589793` | 恢复自转角度，rad，按真实 LIO 朝向累计 |
| `spin_timeout` | `12.0` | 单次自转最长时间，秒 |
| `retreat_speed` | `0.08` | 退让速度，m/s |
| `retreat_distance` | `0.40` | 单次退让上限，米 |
| `retreat_timeout` | `8.0` | 单次退让最长时间，秒 |
| `radius` | `0.47` | 实时雷达检查的车体包络半径，米 |
| `cooldown` | `8.0` | 两次恢复间隔，秒 |
| `max_attempts` | `3` | 无正常前进时的恢复次数上限 |

```bash
ros2 topic echo /navigation/recovery_status
ros2 topic echo /navigation/client/status
```

界面显示“自动脱困 / 重新规划”。`blocked` 表示没有安全退让方向，`rotation_blocked` 表示旋转范围内有障碍，`exhausted` 表示达到次数上限，不再自动恢复；人工处理后暂停并重新开始可重置次数。恢复不能修复错误地图或定位漂移。实现和仿真验证见 [导航自动恢复](docs/navigation_recovery_20261015.md)。

### 5.11 长走廊建图错位与前端诊断

实机与仿真 LIO 现在都保留 25 m 内的点，`lidar_filter_num: 1`，再按 0.05 m 体素降采样。原来的 10 m 截断与隔点抽样可能丢掉走廊远处端墙、转角等少量关键约束，往返时出现沿走廊方向的距离漂移。仅增加回环或修改 MPPI 不能补回这些观测。

对应配置为 `src/rm_lio/FASTLIO2_ROS2/fastlio2/config/lio.yaml`（实机）和 `src/rm_bnrobot_nav/config/lio_sim.yaml`（仿真），由启动参数 `lio_config` 指定。已同步编译安装，重启原建图／探索命令生效；自行传入其他 `lio_config` 时需同步检查这两个参数。该调整不直接改变局部避障的扫描范围。

同时修正了水平导航 TF 与点云的滚转／俯仰／高度转换，避免底盘颠簸时把未补偿的点云按水平位姿拼接；匹配失败时保留 IMU 预测及其协方差，且不向 FAST-LIO 内部匹配地图插入失败帧。匹配缓存按实际点数分配，并修正并行特征标记的数据竞争。

```bash
ros2 topic echo /fastlio2/tracking_status
ros2 topic echo /pgo/status
```

`effective_points` 为当前有效平面匹配点数；`translation_eigen_ratio` 为平移信息矩阵最小／最大特征值比，小于 0.01 时 `weak_translation=true`，提示至少一个平移方向约束较弱。`measurement_updated=false` 表示当前未成功完成激光更新。它是退化诊断，不是新增轮速融合；完全缺少端面、转角或立柱等约束时仍可能漂移。

左侧走廊实际 Gazebo 往返约 12.16 m 的对照测试中，原配置 LIO 最大位置误差约 4.44 m；调整范围与密度后，参数对照和完整修正版均降至约 0.2 m 量级。结果只代表此次仿真路线，不能作为实机精度保证。已经错位的地图需要重新采集，参数不会自动修复历史漂移。详见 [长走廊建图修正](docs/corridor_mapping_20261015.md)。

## 6. 重定位与状态检查

启动三维定位后，日志出现下面的信息表示初始重定位成功：

```text
Initial 3D relocalization succeeded; publishing map -> odom
```

手动设置 PCD 与初始定位位置，以仿真地图为例：

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '$PWD/src/rm_bnrobot_nav/maps/pcd/simmap_normal.pcd', x: -1.96, y: -1.85, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
```

启动与数据流检查：

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 param get /controller_server FollowPath.plugin
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
```

导航节点应为 `active`，控制器插件应为 `nav2_mppi_controller::MPPIController`。

先验模式查看定位后端和每次配准结果：

```bash
ros2 param get /localizer/localizer_node registration_method
ros2 param get /localizer/localizer_node accumulation_seconds
ros2 topic echo /localizer/registration_status
```

默认参数应为 `gicp` 和 `0.5`。状态中 `accepted` 表示本次匹配最终是否被接受；`frames` 是累积帧数；`inlier_ratio` 是有效对应比例；`fitness_m2` 是均方距离，单位 m²（`-1` 表示尚无最终残差）；`duration_ms` 是本次点云累积与配准耗时。该状态仅在处理新扫描时发布，不能代替完整的定位健康状态机；长期没有消息也需要检查雷达、LIO 和时间同步。


## 7. 地图生成

### 7.1 custom_room 仿真地图重建

当前仿真 PCD 和二维地图由同一个 `custom_room.world` 的碰撞几何生成，
直接使用 Gazebo 世界坐标。这是几何参考地图，不是机器人巡航采集的 SLAM 地图。

```bash
python tools/rebuild_custom_room_maps.py
colcon build --packages-select rm_bnrobot_nav
source install/setup.bash
```

输出为 `room.pgm`、`room.yaml`、`pcd/simmap_normal.pcd`。
二维地图分辨率为 0.05 m，投影高度范围是 0.10～1.50 m；先验导航实时 `/scan` 为 0.10～0.50 m，探索实时扫描为 0.10～1.50 m。几何参考地图与实际巡航建图的来源不同。

详细说明见 [custom_room 地图说明](tools/README_custom_room_maps.md)。

### 7.2 实机新场地先验地图全流程

目标是生成同一坐标系下的 `map.pcd + map.yaml + map.pgm`，再配场地自己的巡检点。推荐 **雷达采集 → FAST-LIO＋PGO → 保存关键帧 → 观测轨迹投影 → 重定位验证 → 实机导航**。

建图时用底盘自己的遥控／手动控制方式走场地，不启动自动导航和额外 CAN 下发。先停止旧导航、Gazebo、localizer 和在线探索，避免多个雷达驱动、FAST-LIO 或 `map → odom` 发布者。地图采集、离线处理完成后再切换成先验导航。

#### 7.2.1 创建场地目录和终端环境

在项目根目录执行。`field_a_20261015` 换成自己的场地／采集批次名，使用字母、数字或下划线；每次重新采集用新目录保留旧地图：

```bash
conda activate ros2_env
source /opt/ros/humble/setup.bash
source install/setup.bash

export NAV_WS="$PWD"
export SITE_NAME=field_a_20261015
export SITE_DIR="$NAV_WS/maps_output/$SITE_NAME"
mkdir -p "$SITE_DIR/pgo" "$SITE_DIR/prior" "$SITE_DIR/bags"

cat > "$SITE_DIR/site.env" <<EOF
export NAV_WS="$NAV_WS"
export SITE_NAME="$SITE_NAME"
export SITE_DIR="$SITE_DIR"
EOF
```

后面的每个新终端先在项目根目录执行：

```bash
conda activate ros2_env
source /opt/ros/humble/setup.bash
source install/setup.bash
source maps_output/field_a_20261015/site.env
```

雷达主机／设备 IP 修改 `src/livox_ros_driver2/config/MID360_config.json`，修改后先重新安装配置：

```bash
colcon build --packages-select livox_ros_driver2
source install/setup.bash
```

#### 7.2.2 启动雷达与建图

**终端 A：雷达。**

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**终端 B：FAST-LIO＋在线 PGO，包含 RViz 和实机安装 TF。** 使用与 5.9 节相同的仅建图入口；终端 A 已启动雷达，因此关闭重复驱动。此入口已包含 FAST-LIO，不能同时运行 `lio_launch.py` 或旧的 `pgo_launch.py`：

```bash
ros2 launch real_bringup mapping_3d.launch.py \
  backend:=real \
  start_lidar:=false \
  use_rviz:=true
```

**终端 C：检查数据并录包。** 先确认点云、IMU 持续更新，启动后让车静止至少 5 秒，等待 `IMU initialized`；在第一帧建图的位置和朝向做物理标记，后续放回这个位置用于初始定位：

```bash
ros2 topic hz /livox/lidar
# Ctrl+C 后检查下一项
ros2 topic hz /livox/imu
# Ctrl+C 后检查 LIO
ros2 topic echo /fastlio2/lio_odom --once --field pose.pose

ros2 bag record -o "$SITE_DIR/bags/mapping" \
  /livox/lidar /livox/imu /fastlio2/body_cloud /fastlio2/lio_odom \
  /tf /tf_static /rosout
```

慢速走遍需要导航的通道和房间，转向观测后方与遮挡区域。尽量形成闭环再回到起点。当前在线 PGO 默认要求旧帧与当前帧相隔超过 15 秒、轨迹间距至少 4 m、关键帧间隔至少 15 帧，搜索半径约 2.5 m，并需连续两次匹配确认；停车等待不等于回环完成。回访时低速经过多个关键帧，检查 `/pgo/status` 和 RViz 中是否有明显重影或墙体错位。

#### 7.2.3 保存原始三维地图和观测轨迹

**终端 D：车辆停稳，PGO 仍在运行时保存。**

```bash
ros2 service type /pgo/save_maps
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$SITE_DIR/pgo', save_patches: true}"

ls -lh "$SITE_DIR/pgo/map.pcd" "$SITE_DIR/pgo/poses.txt"
ls "$SITE_DIR/pgo/patches" | head
```

返回 `success: true`、`SAVE SUCCESS!` 后才停止终端 A/B 的驱动和建图，终端 C 用 Ctrl+C 结束录包。输出：

```text
pgo/map.pcd       PGO 合并后的三维地图
pgo/poses.txt     每行：patch文件名 tx ty tz qw qx qy qz
pgo/patches/      每个关键帧的 base_link 点云
bags/mapping/    原始采集记录
```

`save_patches: true` 是后续观测投影和 HBA 的必要条件；同目录再次调用保存会覆盖 `map.pcd`、`poses.txt` 和 `patches/`。

#### 7.2.4 生成可导航且对齐的 PCD／二维地图

使用当前工具，先按每帧最终位姿进行三维动态点一致性清理，再从同一清理结果生成 PCD 并投影实际观测射线。未知背景保留为灰色，不把 PCD 包围框里的所有无点区域都当自由空间：

```bash
python tools/build_prior_map.py \
  --maps-path "$SITE_DIR/pgo" \
  --save-dir "$SITE_DIR/prior" \
  --resolution 0.05 \
  --free-z-min -0.15 --z-min 0.10 --z-max 1.50 \
  --max-ray-range 10.0 --pcd-voxel 0.05

cat "$SITE_DIR/prior/metadata.yaml"
cat "$SITE_DIR/prior/start_pose.env"
xdg-open "$SITE_DIR/prior/map.pgm"
```

输出 `prior/map.pcd`（含法向量）、`map.pgm`、`map.yaml`、`metadata.yaml`、`start_pose.env`。黑色是障碍、白色是观测自由空间、灰色是未知。确认 `free_cells` 非零、通道与房间连通、地面没有误变成墙；未采集好的区域应补采。

默认地面在 map 的 z≈0；高度不符合时调整高度参数后重建。PCD 和二维图均使用相同最终关键帧位姿；不要将 YAML 的 `origin` 随意改为 `[0,0,0]`，也不要另拿独立 SLAM 地图直接配这个 PCD。

旧的 `pcd2map.py` 默认只有障碍＋未知，不能独自补足自由空间。它的 `--assume-free` 会将无点区域视为自由，是人为假设；新场地主流程使用上面的观测投影，不依赖这个假设。

#### 7.2.5 可选：HBA 离线优化后重新生成地图

没有重影且地图质量合格时可跳过。HBA 使用完整关键帧，默认窗口为 20 帧；短采集不适合直接使用默认优化设置。

HBA 更新后先执行 `colcon build --packages-select hba` 并重新加载 `install/setup.bash`。本项目默认 `hba/config/hba.yaml` 中 `planar: true`，与平面 PGO／二维导航保持一致，只优化 yaw／x／y；一般三维数据可设置 `planar: false`。新版会拒绝缺失点云、格式错误和不足窗口的数据，优化成功前不能保存位姿；加载失败不会覆盖上次成功结果。

**HBA 执行完成不等于地图质量改善。** 本次仿真流程已完成采集、优化、成对重建和 Nav2 规划，但这组走廊数据的 HBA 结果几何误差增大，建议选用 `prior`。保留原始数据和优化前地图，对照墙体／门框后再决定是否使用 `prior_hba`。结果与限制见 [HBA 仿真验证](docs/hba_sim_20261015.md)。

**终端 B（已停 PGO 后）：**

```bash
ros2 launch hba hba_launch.py
```

**终端 D：**

```bash
ros2 service call /hba/refine_map interface/srv/RefineMap \
  "{maps_path: '$SITE_DIR/pgo'}"
```

服务的 `load poses success!` 只说明数据加载成功；等终端 B 出现 `END OPTIMIZE`，再保存最终位姿：

```bash
ros2 service call /hba/save_poses interface/srv/SavePoses \
  "{file_path: '$SITE_DIR/refined_poses.txt'}"

python tools/build_prior_map.py \
  --maps-path "$SITE_DIR/pgo" \
  --poses "$SITE_DIR/refined_poses.txt" \
  --save-dir "$SITE_DIR/prior_hba" \
  --resolution 0.05 \
  --free-z-min -0.15 --z-min 0.10 --z-max 1.50 \
  --max-ray-range 10.0 --pcd-voxel 0.05
```

HBA 保存的是七列 `tx ty tz qw qx qy qz`，没有直接保存最终 PCD 的服务；工具按原关键帧顺序和优化位姿同时重建 PCD／二维图，不能把优化 PCD 与旧二维图混用。后续如果选 HBA 输出，统一把下方 `MAP_DIR` 改为 `$SITE_DIR/prior_hba`，重新检查巡检点。用完停止 HBA。

#### 7.2.6 先验证重定位，再接底盘

把车放回建图第一帧对应的物理位置和朝向。初始坐标取保存的第一帧车体位姿，不假设它一定是 `(0,0,0)`，也不沿用仿真的 `(-1.96,-1.85)`：

```bash
export MAP_DIR="$SITE_DIR/prior"
# 使用 HBA 结果时改为：export MAP_DIR="$SITE_DIR/prior_hba"
source "$MAP_DIR/start_pose.env"

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

另一个已加载场地环境的终端，重复设置 `MAP_DIR`、加载 `start_pose.env`，启动**不含雷达和 CAN**的导航入口进行定位检查：

```bash
export MAP_DIR="$SITE_DIR/prior"
source "$MAP_DIR/start_pose.env"

ros2 launch real_bringup real_nav_3d.launch.py \
  navigation_mode:=prior \
  localizer_backend:=gicp localizer_accumulation_seconds:=0.5 \
  map:="$MAP_DIR/map.yaml" pcd_map:="$MAP_DIR/map.pcd" \
  initial_pose_x:="$NAV_INITIAL_X" \
  initial_pose_y:="$NAV_INITIAL_Y" \
  initial_pose_yaw:="$NAV_INITIAL_YAW" \
  use_nav_plan:=false use_ui:=true
```

检查终端：

```bash
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
ros2 run tf2_ros tf2_echo map base_link
# Ctrl+C 后继续检查
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 param get /controller_server FollowPath.plugin
```

应看到初始化重定位成功、Nav2 节点 active、MPPI 插件正确。检查地图与实时点云墙体是否重合、车体是否位于实际位置；`relocalize_check` 只表示初始化状态，不代表持续定位质量。此时没有启动底盘仲裁，客户端显示等待底盘是预期状态。

车没有放回起点时，必须使用实测 map 坐标和 yaw 覆盖初值。也可在节点运行时手动重定位：

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '$MAP_DIR/map.pcd', x: $NAV_INITIAL_X, y: $NAV_INITIAL_Y, z: 0.0, yaw: $NAV_INITIAL_YAW, pitch: 0.0, roll: 0.0}"
```

#### 7.2.7 建立该场地自己的巡检文件

在界面或 RViz 的 map 坐标系下标点。RViz Publish Point 可用 `ros2 topic echo /clicked_point` 读取 x/y；点名任意且唯一，执行顺序由 `point_names` 决定。

先创建只含起点、执行一轮的测试模板，不直接给新场地套用旧坐标：

```bash
cat > "$SITE_DIR/patrol_test.yaml" <<EOF
nav_to_pose_client:
  ros__parameters:
    goal.frame_id: map
    patrol:
      loop: false
      wait_seconds: 1.0
      point_names: [start]
      points:
        start: [$NAV_INITIAL_X, $NAV_INITIAL_Y, 0.0]
EOF
```

用界面添加实测的 p1/p2 等点、排序、另存为该文件；或编辑 YAML。要循环巡检设 `loop: true`。仅在 `points` 下添加点但不加入 `point_names`，不会执行该点。

正式模式保留 recovery/occupy/attack/defend 四个角色。模板先填起点，部署前替换为本场地对应角色的位置：

```bash
cat > "$SITE_DIR/patrol_formal.yaml" <<EOF
nav_plan_node:
  ros__parameters:
    health_threshold: 0.25
    waypoints:
      recovery: [$NAV_INITIAL_X, $NAV_INITIAL_Y, 0.0]
      occupy: [$NAV_INITIAL_X, $NAV_INITIAL_Y, 0.0]
      attack: [$NAV_INITIAL_X, $NAV_INITIAL_Y, 0.0]
      defend: [$NAV_INITIAL_X, $NAV_INITIAL_Y, 0.0]
EOF
```

不要把地图原点／第一帧坐标当成全场统一零点，也不要直接复用 custom_room 的巡检坐标。

<a id="real-site-full"></a>

#### 7.2.8 一键启动新场地实机先验导航（完整参数）

先停止 7.2.6 的独立雷达和导航入口；下面的一键入口已经包含雷达、导航、客户端和 CAN 仲裁。

```bash
export MAP_DIR="$SITE_DIR/prior"
source "$MAP_DIR/start_pose.env"

# 测试巡检：等待定位及底盘就绪后自动执行测试文件
ros2 launch real_bringup real_nav_3d_total.launch.py \
  navigation_mode:=prior \
  localizer_backend:=gicp \
  localizer_accumulation_seconds:=0.5 \
  use_ui:=true \
  patrol_mode:=test \
  patrol_file:="$SITE_DIR/patrol_test.yaml" \
  spin_speed:=1.0 \
  spin_duration:=10.0 \
  map:="$MAP_DIR/map.yaml" \
  pcd_map:="$MAP_DIR/map.pcd" \
  use_rviz:=true \
  initial_pose_x:="$NAV_INITIAL_X" \
  initial_pose_y:="$NAV_INITIAL_Y" \
  initial_pose_yaw:="$NAV_INITIAL_YAW" \
  param_file:="$PWD/src/rm_bringup/real_bringup/config/real_nav2_params_3d.yaml" \
  lio_config:="$PWD/src/rm_lio/FASTLIO2_ROS2/fastlio2/config/lio.yaml" \
  model:="$PWD/src/rm_bnrobot_sim/urdf/bngu_sentinel/bnbot_real.xacro" \
  use_respawn:=False \
  log_level:=info
```

停止测试入口后，正式模式使用对应角色文件，并等待实际比赛状态／裁判信息选择目标：

```bash
ros2 launch real_bringup real_nav_3d_total.launch.py \
  navigation_mode:=prior \
  localizer_backend:=gicp \
  localizer_accumulation_seconds:=0.5 \
  use_ui:=true \
  patrol_mode:=formal \
  patrol_file:="$SITE_DIR/patrol_formal.yaml" \
  spin_speed:=1.0 \
  spin_duration:=10.0 \
  map:="$MAP_DIR/map.yaml" \
  pcd_map:="$MAP_DIR/map.pcd" \
  use_rviz:=true \
  initial_pose_x:="$NAV_INITIAL_X" \
  initial_pose_y:="$NAV_INITIAL_Y" \
  initial_pose_yaw:="$NAV_INITIAL_YAW" \
  param_file:="$PWD/src/rm_bringup/real_bringup/config/real_nav2_params_3d.yaml" \
  lio_config:="$PWD/src/rm_lio/FASTLIO2_ROS2/fastlio2/config/lio.yaml" \
  model:="$PWD/src/rm_bnrobot_sim/urdf/bngu_sentinel/bnbot_real.xacro" \
  use_respawn:=False \
  log_level:=info
```

这些文件使用绝对启动参数，不必复制进 install 或重新编译地图包。场地归档至少保留：原始 rosbag、PGO patches/poses、选用的配对 PCD/PGM/YAML、`start_pose.env` 和巡检文件。

### 7.3 受击自旋与导航同时运行

实机与仿真三维一键入口均保留受击自动自旋，默认 1 rad/s、持续 10 秒；MPPI 在固定朝向的
`nav_base_link` 中规划平移，再按车体朝向转换速度。修改速度/时间时，在 5.1／5.2 或 7.2.8 的完整命令中设置 `spin_speed:=1.0`、`spin_duration:=10.0`，停止旧入口后重启。

导航零速度停止平移，自旋可继续；完整停止使用 `/emergency_stop`。
实现、停止规则和验证结果见 [受击自旋导航说明](docs/spin_navigation_20261014.md)。
