#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from rm_interfaces.msg import Goal




class TFListener(Node):

    def __init__(self):
        super().__init__('tf2_listener')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(1, self.get_transform)
        self.goalPose_publisher_ = self.create_publisher(
            Goal, 'goalPose', 10)
        self.timer = self.create_timer(1, self.get_transform)

    def get_transform(self):
        try:
            tf = self.buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
            transform = tf.transform

            msg = Goal()
            msg.goal_x=transform.translation.x
            msg.goal_y=transform.translation.y
            msg.goal_z=transform.translation.z


            self.get_logger().info(f'发布:{str(msg)}')
            self.goalPose_publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'无法够获取坐标，原因: {str(e)}')


def main():
    rclpy.init()
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
