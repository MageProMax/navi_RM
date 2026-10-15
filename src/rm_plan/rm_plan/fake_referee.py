import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, UInt16


class FakeReferee(Node):
    """
    Minimal fake referee:
    - Publishes game_progress = 4 ("game started")
    - Publishes remain_time and sentry_hp at 1 Hz
    """

    def __init__(self):
        super().__init__('fake_referee')

        self.game_progress = 4      # 4 = GAME STARTED
        self.remain_time = 600      # fake match time
        self.hp = 200               # fake HP

        self.game_progress_pub = self.create_publisher(UInt8, '/game_progress', 10)
        self.remain_time_pub = self.create_publisher(UInt16, '/remain_time', 10)
        self.hp_pub = self.create_publisher(UInt16, '/sentry_hp', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("FakeReferee started. Publishing fake referee data.")

    def timer_callback(self):
        if self.remain_time > 0:
            self.remain_time -= 1

        # game_progress
        msg_pg = UInt8()
        msg_pg.data = self.game_progress
        self.game_progress_pub.publish(msg_pg)

        # remain_time
        msg_rt = UInt16()
        msg_rt.data = self.remain_time
        self.remain_time_pub.publish(msg_rt)

        # sentry_hp
        msg_hp = UInt16()
        msg_hp.data = self.hp
        self.hp_pub.publish(msg_hp)

        self.get_logger().info(
            f"[FakeReferee] game_progress=4, remain_time={self.remain_time}, hp={self.hp}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FakeReferee()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
