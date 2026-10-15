import rclpy
from rclpy.node import Node

from rm_interfaces.msg import Goal
from std_msgs.msg import UInt8, UInt16


class SentryInfoReceiver(Node):
    """Only receive and print status (no decision)."""

    def __init__(self):
        super().__init__('sentry_info_receiver')

        self.current_x = None
        self.current_y = None
        self.current_z = None

        self.game_progress = None
        self.remain_time = None
        self.hp = None

        # Subscriptions
        self.create_subscription(Goal, 'goalPose', self.pose_callback, 10)
        self.create_subscription(UInt8, '/game_progress', self.game_progress_callback, 10)
        self.create_subscription(UInt16, '/remain_time', self.remain_time_callback, 10)
        self.create_subscription(UInt16, '/sentry_hp', self.hp_callback, 10)

        # Timer for printing
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("SentryInfoReceiver started.")

    def pose_callback(self, msg: Goal):
        self.current_x = msg.goal_x
        self.current_y = msg.goal_y
        self.current_z = msg.goal_z

    def game_progress_callback(self, msg: UInt8):
        self.game_progress = msg.data

    def remain_time_callback(self, msg: UInt16):
        self.remain_time = msg.data

    def hp_callback(self, msg: UInt16):
        self.hp = msg.data

    def timer_callback(self):
        pose_str = (
            f"({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f})"
            if self.current_x is not None else "None"
        )

        self.get_logger().info(
            f"[INFO] game_progress={self.game_progress}, "
            f"remain_time={self.remain_time}, hp={self.hp}, pose={pose_str}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SentryInfoReceiver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
