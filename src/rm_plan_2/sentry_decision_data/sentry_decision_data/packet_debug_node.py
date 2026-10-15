import rclpy
from rclpy.node import Node

from sentry_interfaces.msg import SentryChassisPacket


class PacketDebugNode(Node):
    def __init__(self):
        super().__init__("packet_debug_node")

        self.msg_latest = None

        self.sub = self.create_subscription(
            SentryChassisPacket,
            "/chassis/packet",
            self.cb,
            10
        )

        # 每 0.5 秒检查一次有没有消息
        self.timer = self.create_timer(0.5, self.loop)
        self.get_logger().info("PacketDebugNode started, subscribing /chassis/packet")

    def cb(self, msg: SentryChassisPacket):
        self.msg_latest = msg

    def loop(self):
        if self.msg_latest is None:
            self.get_logger().info("waiting packet...")
            return

        gp = int(self.msg_latest.game_progress)
        rt = int(self.msg_latest.remain_time)
        rid = int(self.msg_latest.robot_id)

        if gp != 4:
            self.get_logger().info(f"IDLE game_progress={gp} remain_time={rt}s robot_id={rid}")
        else:
            self.get_logger().info(f"RUN  game_progress=4 remain_time={rt}s robot_id={rid}")


def main():
    rclpy.init()
    node = PacketDebugNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
