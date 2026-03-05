本项目为哨兵导航的决策层 仅负责经规划决策后输出最优移动目标位置 不负责速度/角速度/姿态控制、轨迹跟踪、转身等

0).其他说明
    新终端source顺序
        source /opt/ros/humble/setup.bash
        source ~/sentry_decision_ws/install/setup.bash
1).输入
1.1 话题与类型
    Topic：/chassis/packet
    Msg Type：sentry_interfaces/msg/SentryChassisPacket
    坐标系约定：
    原点：本方地图左下角
     +x：向右
     +y：向前
    单位：位置坐标单位为 mm
1.2 消息字段
    SentryChassisPacket.msg 当前字段如下：
        uint8 game_progress
    	决策门控：game_progress == 4 才开始发布有效目标
        uint16 remain_time（单位：秒）
    	uint8 robot_id
	        编号约定：1=英雄, 3=步兵, 7=哨兵
	        决策通过 robot_id-1 取自身索引（范围 0..2）
    	uint8 team_color
	        1=RED, 2=BLUE
	    uint16 bullet_remaining
	    uint16[3] ally_hp
	    uint16[3] enemy_hp
	    float32[3] ally_pos_x（m）
	    float32[3] ally_pos_y（m）
	    float32[3] enemy_pos_x（m）
	    float32[3] enemy_pos_y（m）
        std_msgs/Header header
        以上数据发布频率暂定为10hz
1.3 数组索引映射
    数组索引 i=0,1,2 对应机器人：

i=0 → 英雄
i=1 → 步兵
i=2 → 哨兵

⚠ 决策层 不再通过 robot_id-1 自动计算索引。

当前采用参数：

self_index（0/1/2）

由 launch 或命令行显式指定。

示例（哨兵）：

self_index = 2
self_hp = ally_hp[self_index]
self_pos = (ally_pos_x[self_index], ally_pos_y[self_index])

这样可避免 robot_id 编号变化带来的索引错误。
1.4 决策侧验收方法
    底盘发布完成后运行：
    ros2 run sentry_decision_data packet_debug
    预期
        没数据：waiting packet...
        game_progress!=4：IDLE ...
        game_progress==4：RUN ...
2.)输出
2.1 话题与类型
    Topic：/motion/goal
    Msg Type：sentry_interfaces/msg/Goal2D
2.2 消息字段与语义
    Goal2D.msg 字段如下：
        std_msgs/Header header（stamp + frame_id）
        float32 x（m）
        float32 y（m）
        bool valid
        uint8 goal_type（预留）
    valid 的关键语义
        valid = false：无有效目标（比赛未开始 / 无数据 / 门控未满足），运动层应不更新目标
        valid = true：目标有效，运动层应追踪 (x,y)
    goal_type（预留）
        未来可能会用到用来自定义决策方向(如进攻防守等等..)
2.3 决策当前输出行为
    主节点 sentry_decision_node 当前逻辑：
        如果没收到 /chassis/packet：发布 valid=false
        如果 game_progress != 4：发布 valid=false
        如果 game_progress == 4：发布 valid=true，目标点由 mode 决定：
        mode=fixed：目标为参数 fixed_x,fixed_y
        mode=hold：目标为自身当前位置（从 ally_pos 数组取）
    快速联调：
        ros2 run sentry_decision sentry_decision_node --ros-args -p mode:=fixed -p fixed_x:=4.0 -p fixed_y:=2.0

2.4 运动层验收方法
    ros2 topic echo /motion/goal
