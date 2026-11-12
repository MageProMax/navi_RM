# 当前实机先验导航的仿真复刻

默认对应根目录 `start_real_prior_nav.sh` 使用的
`maps_output/20261110_152945/obstacle_band`，不使用旧的 custom_room 场景。

```bash
conda activate ros2_env
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_livox_simulation rm_bnrobot_sim rm_bnrobot_nav sim_bringup
source install/setup.bash
ros2 run sim_bringup start_sim_prior_nav.sh gui:=true
```

也可以直接执行 `bash src/rm_bringup/sim_bringup/scripts/start_sim_prior_nav.sh`。
默认打开 RViz 和空目标列表的导航界面，与实机相同：点击开始后按表格顺序
执行全部目标一遍，默认不循环。选中行仅用于编辑；需要只去某一行时选择
“单点导航 · 仅执行选中行”。
`gui:=true` 打开 Gazebo；后台运行加 `use_rviz:=false use_ui:=false`。
所有尾部 `name:=value` 参数会转发给启动文件。

生成器把 PCD 按 5 cm 体素化，仅导出外露三角面作为 Gazebo 的可见和碰撞几何，
并添加 z=0 的水平地板。地图元数据中的拟合平面仅用于定位扫描的高度裁剪，
不能据此推断场地有真实坡面。PCD、PGM、地图原点、
高度元数据和起点文件保持原样；产物存于地图目录下的 `simulation/`。
`manifest.json` 记录输入与产物 SHA-256，文件变化后自动重建。
`NAV_MAP_DIR=/absolute/path/to/map` 可更换配套地图目录，需包含相同文件；
`NAV_SIM_DIR=/absolute/path/to/output` 可另选生成目录。

共用链路为：模拟 MID360 CustomMsg/IMU → FAST-LIO → 高度裁剪 GICP →
Nav2 / MPPI → 导航参考系转换、近墙保护与恢复 → CAN 仲裁与 0x120 编码 →
仿真桥解码 → Gazebo 底盘。Gazebo `/odom` 只用于验收真值，不作为导航定位输入。
仿真使用 `/clock`，CAN 为 dry-run，不启动实机雷达或 USB-CAN。
全局障碍层使用 `combination_method: 1`（Max），实时射线只能清除动态障碍层
自身的占用，不能清除先验地图中的障碍及其膨胀圈；实机使用同一策略。

仿真入口透传 `param_file`、`lio_config`、`rviz_config_file`、`use_nav_plan` 和
`ui_start_empty`，默认与实机使用同一份 `real_nav2_params_3d.yaml`。
LIO 参数只保留仿真静止接触振动的加速度离散阈值差异：实机 0.05 g、仿真 0.15 g。

仿真避障扫描读取 `/fastlio2/navigation_cloud`：保留点云的姿态水平化，
高度以当前车体为参考，不叠加 LIO 相对初始化时刻的高度变化。
定位仍读取原来的 `/fastlio2/body_cloud`，三维坐标与 TF 的对应关系保持原样。
`obstacle_cloud_topic` 参数可选择避障点云；实机入口默认仍使用原话题。
2026-11-12 的只读采样中，原始雷达地面接近 z=0，原 `body_cloud` 中地面
截距约为 0.14 m，导致受 0.08 m 偏移约束的地面拟合失败，并产生地面障碍。
这次修改不扩大地面过滤高度或改变障碍高度区间。

自动验收：

```bash
python src/rm_bringup/sim_bringup/test/real_prior_sim_smoke.py
```

脚本自行启动无界面仿真，默认隔离到 ROS 域 211 和 Gazebo 端口 11511；
可用 `ROS_DOMAIN_ID`、`GAZEBO_MASTER_URI` 环境变量指定其他空闲域与端口。
请先停止占用相同测试域/端口的进程。脚本结束会清理自己启动的进程组。
它通过界面共用的 `/navigation/client/configure` 和 `/navigation/client/start`
接口发送三段目标，检查：

- LiDAR、IMU、FAST-LIO、GICP 注册扫描、障碍扫描、清除点云和编码 CAN 帧均有数据。
- 关键 Nav2 生命周期节点 active，实际参数与实机配置一致，CAN dry-run。
- 启用同一实机 PCD、高度裁剪、GICP 后端及 0.5 秒累积。
- 每段客户端完成，真实车体到点误差小于 0.35 m，定位与真值误差小于 0.20 m。
- 第二段受击后自旋约 10 秒仿真时间，同时平移超过 0.3 m。
- 急停后真实编码帧速度为零，车体不继续移动。
- 原始地图等输入文件的校验值不变。

结果和节点日志在 `log/real_prior_sim/`，失败也保存 `result.json`。
`--output-dir` 可另选目录，重复 `--goal X Y` 可指定目标路线。
测试输出记录每段目标的定位位置、真值位置及误差；失败不会放宽门限。
快速几何与链路检查也注册到了 CTest：

```bash
colcon test --packages-select sim_bringup --ctest-args -R 'real_prior_sim_geometry|sim_real_navigation_contract' --output-on-failure
colcon test-result --test-result-base build/sim_bringup --verbose
```

这是根据已记录障碍高度带生成的近似场景。点云以外的环境、真实逐点采样、
轮滑、MCU 动力学和硬件延迟不在验收范围；仿真通过仍需实车对照。
地图起点临近墙壁，所以先平移离墙，再验收受击自旋的安全包络。

复刻期间还修正了 Livox 仿真插件的量程初始化：原来的自定义 `Init()`
未调用 Gazebo 基类，最小/最大量程保留为 0/1000，ODE 射线却从 0.1 m
开始，因此返回距离少算 0.1 m，无回波射线也可能被当成有效返回。
现在由基类初始化量程，再用 CSV 扫描模式替换规则射线网格。
命中机器人自身碰撞体的射线记为遮挡并输出无效距离，防止仿真接触造成的
微小车体倾斜让边缘越过水平自体过滤框而误触发近墙保护。外部碰撞体的距离和
软件保护门限保持原有处理。
