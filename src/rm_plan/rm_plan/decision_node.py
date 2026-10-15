import rclpy
from rclpy.node import Node

from rm_interfaces.msg import Goal
from std_msgs.msg import UInt8, UInt16


class DecisionNode(Node):
    """
    Simple decision node based on:
    - game_progress (==4 means game started)
    - remain_time
    - sentry_hp
    """

    def __init__(self):
        super().__init__('decision_node')

        # cached data
        self.current_x = None
        self.current_y = None
        self.current_z = None

        self.game_progress = None
        self.remain_time = None
        self.hp = None

        self.last_target = (None, None, None)

        # subscriptions
        self.create_subscription(Goal, 'goalPose', self.pose_callback, 10)
        self.create_subscription(UInt8, '/game_progress', self.progress_callback, 10)
        self.create_subscription(UInt16, '/remain_time', self.remain_time_callback, 10)
        self.create_subscription(UInt16, '/sentry_hp', self.hp_callback, 10)

        # publisher
        self.target_pub = self.create_publisher(Goal, 'targetPose', 10)

        # timer
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("DecisionNode started.")

    def pose_callback(self, msg: Goal):
        self.current_x = msg.goal_x
        self.current_y = msg.goal_y
        self.current_z = msg.goal_z

    def progress_callback(self, msg: UInt8):
        self.game_progress = msg.data

    def remain_time_callback(self, msg: UInt16):
        self.remain_time = msg.data

    def hp_callback(self, msg: UInt16):
        self.hp = msg.data

    def timer_callback(self):
        # 完整的初始值判断，避免NoneType比较错误
        if self.game_progress is None:
            self.get_logger().info("[WAITING] No game progress data.")
            return
        
        # check if game started
        game_started = (self.game_progress == 4)

        if not game_started:
            self.get_logger().info("[WAITING] Game not started (progress={}).".format(self.game_progress))
            return

        if self.remain_time is None or self.hp is None:
            self.get_logger().info("[WAITING] Missing remain_time or hp data.")
            return

        # ========== 修复缩进：这部分代码要属于timer_callback函数 ==========
        # decision rules
        # if self.remain_time > 300:
        if self.remain_time > 540 and self.remain_time < 580:
            target = (-0.86, 7.65, 0.0)
        elif self.remain_time>500:
            target = (-3.06, 2.79, 0.0)
        else:
            target = (-0.00, 0.00, 0.0)
        # else:
        #    target = (0.0, 0.0, 0.0)

        # only publish when changed
        # if target != self.last_target:
        msg = Goal()
        msg.goal_x, msg.goal_y, msg.goal_z = target
        self.target_pub.publish(msg)
        self.last_target = target
        self.get_logger().info(f"[DECISION] New target={target}")
        # else:
        #     self.get_logger().info(f"[KEEP] Current target={target}")


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
