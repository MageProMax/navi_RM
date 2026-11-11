# 同步编辑后的配对地图

导航时成对使用本目录的 `map.yaml` 与 `map.pcd`；`map.pgm` 与用户的 `edited/map_edit.pgm` 完全相同。

依据原图与修改图的灰度变化生成擦除掩膜：占用概率降低、且新像素不再被判为障碍的区域参与删除，包括在原自由空间上重新涂白的区域。抗锯齿边缘仍被判为障碍的像素保留点云。未新增障碍。

按用户明确选择，删除擦除区域内所有高度的点，包括地面及天花板。原始 2,462,448 点，删除 838,069 点，剩余 1,624,379 点。保留点的所有 PCD 字段、原始数值及顺序不变。

`erase_mask.png` 中白色表示删除区域；`edit_comparison.png` 中红色显示删除区域；详细校验与源文件哈希见 `edit_metadata.json`。

原始 `pgo/` 和 `edited/` 文件均未覆盖。原始关键帧未修改，从原始关键帧重新导出地图会恢复被手动删除的点，需重新应用擦除掩膜。

## 实机先验导航

先停止旧 `mapping_3d.launch.py`、独立雷达驱动和单独地图服务，再在已激活 `ros2_env` 的终端执行：

```bash
cd /home/mage/navigation/nav_rm_final
bash maps_output/real_20261022_144424/edited_synced/start_prior.sh
```

脚本加载本目录的配对 `map.yaml/map.pcd`，使用当前实机模型和 FAST-LIO 外参；启动雷达、GICP 定位、Nav2、导航界面、RViz，以及约 25 秒后启动的 CAN 仲裁。`use_nav_plan:=false` 关闭旧场地的正式巡检决策；`ui_start_empty:=true` 让客户端以空列表和手动模式启动，不载入默认巡检点。目标由用户添加或双击地图给出，点击“开始”后才执行。多个目标可选择按列表巡检。RViz 使用先验导航配置 `nav_final.rviz`，显示 `/map` 和 `/localizer/map_cloud`。

脚本的默认初值来自本目录 `start_pose.env`，只适用于车辆放回本次建图起点与朝向：x=-0.0119637896562 m、y=-0.206480209949 m、yaw=-0.000845283377346 rad。换位置时可在脚本末尾追加 `initial_pose_x:=... initial_pose_y:=... initial_pose_yaw:=...` 覆盖，不要把任意停放位置当作建图起点。

用户确认当前仍在建图起点和原朝向，因此继续使用上述 map 初值。另一次 `/fastlio2/lio_odom` 读数 `x=-0.0018398425796400941, y=-0.21190038972503755, qz=0.655370281736005, qw=0.7553077477540326` 属于 odom 坐标，yaw 约 `1.4293455116601874 rad`；该值不是 map 位姿，不直接写入先验定位初值。

启动后静止至少 5 秒完成 IMU 初始化，检查 `/localizer/registration_status` 与 RViz 点云和地图的对齐情况。需要从其他位置重定位时使用 `/localizer/relocalize` 服务；当前定位器不订阅 RViz 的 `/initialpose`。

实时障碍使用真实回波射线清除：全局障碍层允许已观测自由格覆盖旧先验占用，射线起点为实际 `mid360` 原点。缺测不视为空地，当前回波仍标记为障碍。重启导航生效；RViz 看 `/global_costmap/costmap` 的实时结果，`/map` 和 `/localizer/map_cloud` 仍是原始先验参考。详见 `docs/prior_ray_clearing_20261022.md`。
