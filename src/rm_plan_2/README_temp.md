目的

本 temp 文档用于底盘与运动层同学做通信连通性测试。当前版本的决策逻辑为“临时 demo”，不追求策略正确性，只验证：

决策能否稳定接收 /chassis/packet

决策能否按规则切换目标点并发布 /motion/goal

运动层能否稳定收到 goal 并执行/打印

0) 环境与启动前准备
0.1 新终端 source 顺序
source /opt/ros/humble/setup.bash
source ~/sentry_decision_ws/install/setup.bash

0.2 编译

在工作空间根目录：

cd ~/sentry_decision_ws
colcon build --base-paths src

1) 实现了哪些内容（当前版本）
1.1 接口包

sentry_interfaces

SentryChassisPacket.msg：底盘 → 决策

Goal2D.msg：决策 → 运动层

1.2 调试节点（建议用来验收底盘输出）

sentry_decision_data/packet_debug

订阅 /chassis/packet

打印三种状态：waiting / IDLE / RUN

1.3 主决策节点（联调主入口）

sentry_decision/sentry_decision_node

输入：/chassis/packet

输出：/motion/goal

门控：game_progress == 4 才发布 valid=true

支持三种模式：

mode:=hold：目标=自身当前位置（测试“跟随本车点位输出”）

mode:=fixed：目标=固定点（fixed_x/fixed_y）

mode:=demo：目标=临时三态策略（repair/defend/attack）

防抖：状态切换才打印 STATE CHANGE；目标点变化小于 goal_eps 时不重复发布

2) 联调流程
Step A：底盘侧先验收能发包

终端 A：

ros2 run sentry_decision_data packet_debug


预期：

没收到包：waiting packet...

收到但未开始：IDLE game_progress=...

开始：RUN game_progress=4 ...

如果一直 waiting：

先 ros2 topic list | grep chassis

再 ros2 topic echo /chassis/packet 看是否有数据

Step B：运动层侧验收“我能收到 goal”

终端 B：

ros2 topic echo /motion/goal


如果什么都没有：

确认决策节点是否在跑（见 Step C）

确认运动层与决策处于同一 ROS_DOMAIN_ID（如有使用）

Step C：启动决策节点（推荐 demo 模式）

终端 C：

ros2 run sentry_decision sentry_decision_node --ros-args -p mode:=demo


运行中你会看到：

waiting packet... (publish invalid goal)（无包）

IDLE ... (publish invalid goal)（game_progress != 4）

STATE CHANGE: ... -> ...（状态切换时才出现一次）

3) 临时 demo 策略说明（仅用于联调）

当 mode:=demo 时，决策按如下规则选目标点：

自身血量 < hp_low → 维修点（REPAIR）

弹量 < bullet_low → 防守点（DEFEND）

其他情况 → 进攻点（ATTACK）

对应的目标点坐标全部参数化，可运行时修改。

4) 常用参数
4.1 通用参数

goal_topic（默认 /motion/goal）

packet_topic（默认 /chassis/packet）

goal_eps：目标点防抖阈值（默认 0.05m）

示例：

ros2 run sentry_decision sentry_decision_node --ros-args -p mode:=demo -p goal_eps:=0.05

4.2 fixed 模式参数

fixed_x、fixed_y（单位 m）

示例：

ros2 run sentry_decision sentry_decision_node --ros-args -p mode:=fixed -p fixed_x:=4.0 -p fixed_y:=2.0

4.3 demo 模式参数（建议底盘/运动层联调时使用）

hp_low（默认 80）

bullet_low（默认 30）

pt_repair_x / pt_repair_y

pt_defend_x / pt_defend_y

pt_attack_x / pt_attack_y

示例

ros2 run sentry_decision sentry_decision_node --ros-args \
  -p mode:=demo \
  -p hp_low:=80 \
  -p bullet_low:=30 \
  -p pt_repair_x:=1.0 -p pt_repair_y:=1.0 \
  -p pt_defend_x:=2.0 -p pt_defend_y:=2.0 \
  -p pt_attack_x:=3.0 -p pt_attack_y:=3.0

5) 不接底盘时，如何用命令行“模拟底盘发包”（决策自测）

终端 D（模拟底盘，循环发布 5Hz）：

ros2 topic pub -r 5 /chassis/packet sentry_interfaces/msg/SentryChassisPacket "{
  header: {frame_id: map},
  game_progress: 4,
  remain_time: 300,
  robot_id: 3,
  team_color: 1,
  bullet_remaining: 200,
  ally_hp: [100, 100, 300],
  enemy_hp: [200, 200, 200],
  ally_pos_x: [1.0, 2.0, 3.0],
  ally_pos_y: [1.5, 2.5, 3.5],
  enemy_pos_x: [6.0, 7.0, 8.0],
  enemy_pos_y: [6.5, 7.5, 8.5]
}"

快速制造三种状态：

进攻（ATTACK）：ally_hp[2] 高，bullet_remaining 高

维修（REPAIR）：把哨兵血量 ally_hp[2] 改低（如 50）

防守（DEFEND）：把 bullet_remaining 改低（如 10）

6) 备注

目前决策只输出“目标点”，不输出速度/轨迹

高地/障碍/路径规划尚未接入；现阶段目标仅用于验证通信闭环
