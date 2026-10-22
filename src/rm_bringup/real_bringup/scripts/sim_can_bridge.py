#!/usr/bin/env python3
"""Decode the same 0x120 payload used by real CAN into Gazebo ROS velocities."""
import struct
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray


class SimCanBridge(Node):
    def __init__(self):
        super().__init__('sim_can_bridge')
        self.sim_time = bool(self.get_parameter('use_sim_time').value)
        self.command = Twist()
        self.receipt = -float('inf')
        self.pub = self.create_publisher(Twist, '/cmd_vel_sim', 1)
        self.create_subscription(UInt8MultiArray, '/can_twist/command_frame', self.receive, 1)
        self.create_timer(0.01, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9 if self.sim_time else time.monotonic()

    def receive(self, msg):
        if len(msg.data) != 8:
            self.command = Twist()
            self.receipt = -float('inf')
            return
        vx_mcu, vy_mcu, wz, _power = struct.unpack('<hhhh', bytes(msg.data))
        if max(abs(vx_mcu), abs(vy_mcu), abs(wz)) > 500:
            self.command = Twist()
            self.receipt = -float('inf')
            return
        command = Twist()
        # Inverse of the hardware convention: MCU vx=-ROS vy, MCU vy=ROS vx.
        command.linear.x, command.linear.y = vy_mcu/500., -vx_mcu/500.
        command.angular.z = wz/500.
        self.command = command
        self.receipt = self.now()

    def tick(self):
        age = self.now() - self.receipt
        self.pub.publish(self.command if 0 <= age < .25 else Twist())


def main():
    rclpy.init();node = SimCanBridge()
    try:rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':main()
