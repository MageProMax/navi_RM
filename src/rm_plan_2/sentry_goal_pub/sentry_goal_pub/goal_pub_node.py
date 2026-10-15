import rclpy
from rclpy.node import Node

from sentry_interfaces.msg import Goal2D


class GoalPubNode(Node):
    def __init__(self):
        super().__init__("goal_pub_node")

        self.timer = self.create_timer(1.0, self.loop)

        self.declare_parameter("x", 4000.0)
        self.declare_parameter("y", 6000.0)

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)


        self.pub = self.create_publisher(Goal2D, "/motion/goal", 10)

        self.get_logger().info("GoalPubNode started, publishing to /motion/goal")

    def loop(self):
        msg = Goal2D()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.x = self.x
        msg.y = self.y
        msg.valid = True
        msg.goal_type = 1
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GoalPubNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
