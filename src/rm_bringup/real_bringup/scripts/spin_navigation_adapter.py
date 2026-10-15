#!/usr/bin/env python3
"""Keep MPPI's translation frame non-rotating while the physical base spins."""
import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class SpinNavigationAdapter(Node):
    def __init__(self):
        super().__init__('spin_navigation_adapter')
        self.sim_time = bool(self.get_parameter('use_sim_time').value)
        self.require_mapping_ready=bool(self.declare_parameter('require_mapping_ready',False).value)
        self.mapping_ready=False;self.mapping_ready_since=math.inf
        if self.require_mapping_ready:
            self.create_subscription(Bool,'/pgo/map_ready',self.mapping_cb,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        self.scan_guard = bool(self.declare_parameter('scan_guard', False).value)
        self.scan_points = []
        self.exploration_yaw=0.;self.exploration_yaw_receipt=-math.inf;self.hit_spinning=False
        self.scan_stamp = -math.inf
        if self.scan_guard:
            self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
            self.create_subscription(Float64,'/navigation/exploration_yaw_rate',self.exploration_yaw_cb,1)
            self.create_subscription(Bool,'/spin_active',lambda m:setattr(self,'hit_spinning',m.data),1)
        self.declare_parameter('odom_timeout', 0.25)
        self.declare_parameter('command_timeout', 0.25)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        if not all(math.isfinite(x) and x > 0 for x in (self.odom_timeout, self.command_timeout)):
            raise ValueError('Timeouts must be finite and positive')
        self.odom_pub = self.create_publisher(Odometry, '/navigation/odom', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.permission_pub = self.create_publisher(Bool, '/spin_motion_permitted', 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/fastlio2/lio_odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, '/nav_cmd_vel', self.command_cb, 1)
        self.create_subscription(Bool, '/emergency_stop', self.stop_cb, 1)
        self.last_odom = None
        self.odom_receipt = -math.inf
        self.command_receipt = -math.inf
        self.command = Twist()
        self.have_command = False
        self.initial_idle = True  # A hit before the first goal can still spin after LIO starts.
        self.stopped = False
        self.yaw = self.yaw_rate = 0.0
        self.stamp_seconds = 0.0
        self.create_timer(0.01, self.tick)

    def mapping_cb(self,msg):
        if msg.data and not self.mapping_ready:
            self.mapping_ready_since=self.elapsed_clock()
            self.initial_idle=True  # Map is usable again; an idle robot may resume hit-spin.
        self.mapping_ready=msg.data
        if not msg.data:
            self.command=Twist();self.have_command=False;self.initial_idle=False
            self.cmd_pub.publish(Twist());self.permission_pub.publish(Bool(data=False))

    def elapsed_clock(self):
        return self.get_clock().now().nanoseconds*1e-9 if self.sim_time else time.monotonic()

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        values = (q.x, q.y, q.z, q.w, p.x, p.y, p.z, stamp)
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link' or not all(
                math.isfinite(v) for v in values) or norm < 1e-6:
            self.odom_receipt = -math.inf
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        virtual = copy.deepcopy(msg)
        virtual.child_frame_id = 'nav_base_link'
        virtual.pose.pose.orientation.x = virtual.pose.pose.orientation.y = 0.0
        virtual.pose.pose.orientation.z = 0.0
        virtual.pose.pose.orientation.w = 1.0
        virtual.twist.twist = Twist()
        self.yaw_rate = 0.0
        if self.last_odom is not None:
            old, old_stamp, old_yaw = self.last_odom
            dt = stamp - old_stamp
            if dt < 0:
                self.have_command = False
                self.initial_idle = False
                self.command = Twist()
            if 0.001 <= dt <= self.odom_timeout:
                # Differentiate the axis-centre pose, avoiding the IMU lever arm.
                virtual.twist.twist.linear.x = (p.x-old.pose.pose.position.x)/dt
                virtual.twist.twist.linear.y = (p.y-old.pose.pose.position.y)/dt
                self.yaw_rate = math.atan2(math.sin(yaw-old_yaw), math.cos(yaw-old_yaw))/dt
        self.last_odom = (copy.deepcopy(msg), stamp, yaw)
        self.yaw, self.stamp_seconds = yaw, stamp
        self.odom_receipt = self.elapsed_clock()
        # odom->base_link is LIO-owned. This child cancels only its planar yaw.
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'nav_base_link'
        transform.transform.rotation.z = math.sin(-yaw/2)
        transform.transform.rotation.w = math.cos(-yaw/2)
        self.tf.sendTransform(transform)
        self.odom_pub.publish(virtual)

    def command_cb(self, msg):
        if self.stopped:
            return
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
            self.have_command = False
            self.initial_idle = False
            return
        self.initial_idle = False
        self.command = copy.deepcopy(msg)
        self.command_receipt = self.elapsed_clock()
        self.have_command = True

    def exploration_yaw_cb(self,msg):
        self.exploration_yaw=max(-.4,min(.4,msg.data)) if math.isfinite(msg.data) else 0.
        self.exploration_yaw_receipt=self.elapsed_clock()

    def scan_cb(self, msg):
        if msg.header.frame_id != 'base_link':
            self.scan_stamp = -math.inf
            return
        self.scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.scan_points = [(r*math.cos(msg.angle_min+i*msg.angle_increment),
                             r*math.sin(msg.angle_min+i*msg.angle_increment))
                            for i,r in enumerate(msg.ranges)
                            if math.isfinite(r) and msg.range_min <= r < msg.range_max]

    @staticmethod
    def guarded_velocity(vx, vy, points, radius=.47):
        # Swept circular envelope, with 0.4 s reaction allowance. This guard
        # uses body-frame returns, independent of map alignment and MPPI costs.
        speed=math.hypot(vx,vy)
        if speed < 1e-9:return vx,vy
        ux,uy=vx/speed,vy/speed
        clearance=math.inf
        for x,y in points:
            along=x*ux+y*uy;across=abs(x*uy-y*ux)
            if along<=0 or across>=radius:continue
            clearance=min(clearance,along-math.sqrt(radius*radius-across*across))
        allowed=min(speed,max(0.,clearance)/.4,math.sqrt(2.*max(0.,clearance)))
        return vx*allowed/speed,vy*allowed/speed

    def stop_cb(self, msg):
        self.stopped = msg.data
        self.initial_idle = False
        self.have_command = False
        self.command = Twist()
        self.cmd_pub.publish(Twist())
        self.permission_pub.publish(Bool(data=False))

    def tick(self):
        now = self.elapsed_clock()
        stamp_age = self.get_clock().now().nanoseconds*1e-9 - self.stamp_seconds
        odom_valid = (0 <= now-self.odom_receipt < self.odom_timeout and
                      -0.05 <= stamp_age < self.odom_timeout)
        idle = self.have_command and abs(self.command.linear.x) < 1e-9 and abs(
            self.command.linear.y) < 1e-9 and abs(self.command.angular.z) < 1e-9
        # An explicit idle command remains idle, so a hit at a waypoint can spin.
        # A stale nonzero navigation command must never be replayed or keep spin enabled.
        permitted = not self.stopped and odom_valid and (
            self.initial_idle or idle or (self.have_command and
                                         0 <= now-self.command_receipt < self.command_timeout))
        if self.require_mapping_ready:
            permitted=permitted and self.mapping_ready and now-self.mapping_ready_since>=1.0
        if self.scan_guard:
            scan_age=self.get_clock().now().nanoseconds*1e-9-self.scan_stamp
            permitted = permitted and -.05 <= scan_age < .5 and bool(self.scan_points)
        effective = Twist()
        if permitted:
            yaw = self.yaw + self.yaw_rate*max(0.0, stamp_age)
            c, s = math.cos(yaw), math.sin(yaw)
            effective.linear.x = c*self.command.linear.x + s*self.command.linear.y
            effective.linear.y = -s*self.command.linear.x + c*self.command.linear.y
            effective.angular.z = self.command.angular.z
            if self.scan_guard:
                if not self.hit_spinning and 0<=now-self.exploration_yaw_receipt<1.5:
                    effective.angular.z=self.exploration_yaw
                effective.linear.x,effective.linear.y=self.guarded_velocity(
                    effective.linear.x,effective.linear.y,self.scan_points)
                # Preserve hit-spin whenever its swept footprint is clear.
                if any(math.hypot(x,y)<.45 for x,y in self.scan_points):
                    # CAN permission inhibits hit-spin. Keep the separately
                    # checked translation so the robot can retreat from a wall.
                    permitted=False
                    effective.angular.z=0.
        self.permission_pub.publish(Bool(data=permitted))
        self.cmd_pub.publish(effective)


def main():
    rclpy.init()
    node = SpinNavigationAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
