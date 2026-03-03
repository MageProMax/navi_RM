sentry_decision

本工作包包含三个主要节点，用于测试哨兵决策层在没有真实裁判系统/导航系统时的完整数据链路。
只需运行假裁判系统节点即可模拟比赛环境，验证整个决策流程是否连通。

1. fake_referee
模拟裁判系统，持续发布比赛阶段、剩余时间、哨兵血量等信息，使得决策层无需真实裁判系统也能测试。

运行:   ros2 run sentry_decision fake_referee

2. sentry_info_receiver
用于查看是否成功收到假裁判系统 & get_pose.py 的数据。
只监控，不参与决策。

每秒打印：
game_progress
remain_time
sentry_hp
当前坐标 goalPose（若存在）

运行:     ros2 run sentry_decision sentry_info_receiver

3. decision_node（决策算法核心）
订阅裁判系统信息 + 当前坐标，根据规则生成目标点 targetPose。
当前为测试残血状态
只有当目标点改变时才发布，避免重复发送
发布 targetPose（供导航执行节点使用）

运行:  ros2 run sentry_decision decision_node


接口说明:
fake_referee 发布：
Topic名称               类型                          描述
/game_progress     std_msgs/UInt8        比赛阶段，4 表示比赛开始
/remain_time	   std_msgs/UInt16	     当前阶段剩余时间（秒）
/sentry_hp	       std_msgs/UInt16	     哨兵血量（假数据）

get_pose.py发布：
Topic 名称	         类型             	描述
goalPose	rm_interfaces/Goal	当前坐标 (x, y, z)

decision_node 发布：
Topic 名称	            类型	           描述
targetPose	rm_interfaces/Goal	决策后的目标坐标 (x, y, z)

启动方式:
ros2 launch sentry_decision sentry_test.launch.py

#构建工作空间(略)
启动假裁判系统(ros2 run sentry_decision fake_referee)
启动决策节点(ros2 run sentry_decision decision_node)
查看调试信息(ros2 run sentry_decision sentry_info_receiver)