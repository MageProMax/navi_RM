#!/usr/bin/env python3
"""Keep MPPI's translation frame non-rotating while the physical base spins."""
import copy
import math
import time
import json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Polygon, Point32
from nav_msgs.msg import Odometry, Path
from nav2_msgs.msg import Costmap
from rm_interfaces.msg import NavigationStatus
from navigation_recovery import Recovery, RecoveryConfig, Grid, Decision
from robot_geometry import RobotGeometry
from std_msgs.msg import Bool, Float64, String
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.time import Time


class SpinNavigationAdapter(Node):
    def __init__(self):
        super().__init__('spin_navigation_adapter')
        self.sim_time = bool(self.get_parameter('use_sim_time').value)
        self.close_wall_mode=bool(self.declare_parameter('close_wall_mode',False).value)
        self.geometry=RobotGeometry()
        self.footprint_state=None;self.footprint_time=-math.inf
        if self.close_wall_mode:
            self.footprint_pubs=[self.create_publisher(Polygon,topic,1) for topic in
                                 ('/local_costmap/footprint_request','/global_costmap/footprint_request')]
        self.require_mapping_ready=bool(self.declare_parameter('require_mapping_ready',False).value)
        self.mapping_ready=False;self.mapping_ready_since=math.inf
        if self.require_mapping_ready:
            self.create_subscription(Bool,'/pgo/map_ready',self.mapping_cb,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        self.scan_guard = bool(self.declare_parameter('scan_guard', False).value)
        self.recovery_enabled=bool(self.declare_parameter('recovery.enabled',False).value)
        self.recovery_decision=Decision();self.recovery_announced=False;self.recovery_tick_time=-math.inf
        if self.recovery_enabled:
            defaults=RecoveryConfig();values={}
            for key,value in defaults.__dict__.items():values[key]=self.declare_parameter('recovery.'+key,value).value
            self.recovery=Recovery(RecoveryConfig(**values))
            self.recovery_grid=None;self.grid_receipt=-math.inf;self.grid_stamp=-math.inf
            self.client_status=None;self.client_receipt=-math.inf;self.explorer_state='';self.explorer_receipt=-math.inf
            self.path=[];self.path_receipt=-math.inf;self.path_stamp=-math.inf;self.task_key=None
            self.recovery_buffer=Buffer();self.recovery_listener=TransformListener(self.recovery_buffer,self)
            self.recovery_active_pub=self.create_publisher(Bool,'/navigation/recovery_active',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
            self.recovery_status_pub=self.create_publisher(String,'/navigation/recovery_status',10)
            self.recovery_active_pub.publish(Bool(data=False));self.last_recovery_state=''
            self.create_subscription(Costmap,'/local_costmap/costmap_raw',self.costmap_cb,1)
            self.create_subscription(NavigationStatus,'/navigation/client/status',self.client_cb,10)
            self.create_subscription(String,'/exploration/status',self.explorer_cb,10)
            self.create_subscription(Path,'/plan',self.path_cb,1)
        self.scan_points = []
        self.exploration_yaw=0.;self.exploration_yaw_receipt=-math.inf;self.hit_spinning=False
        self.scan_stamp = -math.inf
        if self.scan_guard or self.recovery_enabled or self.close_wall_mode:
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

    def client_cb(self,msg):
        if msg.paused or msg.emergency_stop or msg.state in ('stopped','complete'):
            self.recovery.cancel();self.recovery.goal=None;self.recovery.attempts=0
        self.recovery_tick_time=-math.inf
        key=(msg.mode,msg.point_index,msg.target.goal_frame_id,round(msg.target.goal_x,4),round(msg.target.goal_y,4))
        if key!=self.task_key:self.path_receipt=-math.inf;self.task_key=key
        self.client_status=msg;self.client_receipt=self.elapsed_clock()

    def explorer_cb(self,msg):
        try:self.explorer_state=json.loads(msg.data).get('state','');self.explorer_receipt=self.elapsed_clock()
        except (ValueError,TypeError):self.explorer_receipt=-math.inf

    def costmap_cb(self,msg):
        self.grid_receipt=-math.inf
        if msg.header.frame_id!='odom':return
        meta=msg.metadata;q=meta.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        try:self.recovery_grid=Grid(np.asarray(msg.data,dtype=np.uint8).reshape(meta.size_y,meta.size_x),meta.resolution,(meta.origin.position.x,meta.origin.position.y,yaw))
        except (ValueError,OverflowError):return
        self.grid_receipt=self.elapsed_clock();self.grid_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def path_cb(self,msg):
        self.path=[];self.path_receipt=-math.inf
        if not msg.poses:return
        try:
            if msg.header.frame_id=='odom':c,s,x,y=1.,0.,0.,0.
            else:
                transform=self.recovery_buffer.lookup_transform('odom',msg.header.frame_id,Time())
                q=transform.transform.rotation;t=transform.transform.translation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));c,s,x,y=math.cos(yaw),math.sin(yaw),t.x,t.y
            self.path=[(x+c*p.pose.position.x-s*p.pose.position.y,y+s*p.pose.position.x+c*p.pose.position.y) for p in msg.poses]
            if not all(math.isfinite(v) for p in self.path for v in p):self.path=[];return
            self.path_receipt=self.elapsed_clock();self.path_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        except Exception:return

    def recovery_update(self,now,odom_valid):
        if not self.recovery_enabled:return Decision()
        if 0<=now-self.recovery_tick_time<.1:return self.recovery_decision
        self.recovery_tick_time=now;client=self.client_status
        pose=(self.last_odom[0].pose.pose.position.x,self.last_odom[0].pose.pose.position.y,self.yaw) if self.last_odom else (0.,0.,0.)
        task=client is not None and 0<=now-self.client_receipt<2. and not client.paused and not client.emergency_stop
        if task:
            exploration=client.mode in ('explore','explore_goal') and 0<=now-self.explorer_receipt<2. and self.explorer_state=='no_reachable_frontiers'
            task=(client.state in ('sending','executing','failed','rejected','recovering') or
                  (client.state=='waiting' and client.point_count>0) or exploration)
            # Nav2 can report zero distance when planning has produced no path.
            # Only a geometrically nearby goal suppresses stalled-goal spin;
            # a costmap overlap still needs recovery even beside the goal.
            if client.state=='executing' and self.recovery_grid is not None and self.recovery_grid.sample(pose[0],pose[1])[0]<253:
                try:
                    gx,gy=client.target.goal_x,client.target.goal_y
                    if client.target.goal_frame_id!='odom':
                        tf=self.recovery_buffer.lookup_transform('odom',client.target.goal_frame_id,Time())
                        q=tf.transform.rotation;t=tf.transform.translation
                        angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                        gx,gy=t.x+math.cos(angle)*gx-math.sin(angle)*gy,t.y+math.sin(angle)*gx+math.cos(angle)*gy
                    if math.hypot(gx-pose[0],gy-pose[1])<.25:task=False
                except Exception:pass
        ready=not self.stopped and odom_valid and (not self.require_mapping_ready or (self.mapping_ready and now-self.mapping_ready_since>=1.))
        ros_now=self.get_clock().now().nanoseconds*1e-9
        fresh=(0<=now-self.grid_receipt<2. and -.05<=ros_now-self.grid_stamp<2. and -.05<=ros_now-self.scan_stamp<.5)
        valid_path=0<=now-self.path_receipt<2.5 and -.05<=ros_now-self.path_stamp<2.5 and bool(self.path)
        if valid_path:
            valid_path=all(self.recovery_grid.sample(x,y)[0]<253 for x,y in self.path if math.hypot(x-pose[0],y-pose[1])<1.2) if self.recovery_grid else False
        decision=self.recovery.tick(now,bool(task and ready),self.task_key,pose,self.recovery_grid if fresh else None,self.scan_points if fresh else [],valid_path)
        if task and ready and not fresh:decision=Decision('waiting_sensors',override=True,reason='Recovery requires fresh costmap and laser scan')
        if decision.active!=self.recovery_announced:
            self.recovery_active_pub.publish(Bool(data=decision.active));self.recovery_announced=decision.active
            # Neither start nor end of recovery may replay the old Nav2 velocity.
            self.command=Twist();self.have_command=False;self.initial_idle=True
        if decision.state!=self.last_recovery_state:
            self.get_logger().info('Recovery: '+decision.state+' '+decision.reason);self.last_recovery_state=decision.state
        self.recovery_status_pub.publish(String(data=json.dumps({'state':decision.state,'active':decision.active,'reason':decision.reason,'attempts':self.recovery.attempts})))
        self.recovery_decision=decision;return decision

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
        if msg.header.frame_id != 'base_link' or not all(math.isfinite(v) for v in (msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max)) or msg.angle_increment<=0 or not 0<=msg.range_min<msg.range_max:
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

    def physical_spin(self, angular_command):
        return self.hit_spinning or abs(self.yaw_rate)>.05 or abs(angular_command)>1e-6

    def publish_footprint(self, now, odom_valid, angular_command):
        if not self.close_wall_mode or not odom_valid:
            return
        spinning=self.physical_spin(angular_command)
        state=(spinning,self.yaw)
        if self.footprint_state is not None:
            old_spin,old_yaw=self.footprint_state
            delta=abs(math.atan2(math.sin(self.yaw-old_yaw),math.cos(self.yaw-old_yaw)))
            if old_spin==spinning and (spinning or delta<.005) and 0<=now-self.footprint_time<1.:
                return
        polygon=Polygon()
        polygon.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in
                        self.geometry.footprint(self.yaw,spinning)]
        for publisher in self.footprint_pubs:
            publisher.publish(polygon)
        self.footprint_state=state;self.footprint_time=now

    def tick(self):
        now = self.elapsed_clock()
        stamp_age = self.get_clock().now().nanoseconds*1e-9 - self.stamp_seconds
        odom_valid = (0 <= now-self.odom_receipt < self.odom_timeout and
                      -0.05 <= stamp_age < self.odom_timeout)
        recovery=self.recovery_update(now,odom_valid)
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
                if not self.close_wall_mode:
                    effective.linear.x,effective.linear.y=self.guarded_velocity(
                        effective.linear.x,effective.linear.y,self.scan_points)
                # Preserve hit-spin whenever its swept footprint is clear.
                if any(math.hypot(x,y)<.45 for x,y in self.scan_points):
                    # CAN permission inhibits hit-spin. Keep the separately
                    # checked translation so the robot can retreat from a wall.
                    permitted=False
                    effective.angular.z=0.
        if recovery.override:
            effective=Twist();permitted=False
            if recovery.active and not self.stopped and odom_valid and (not self.require_mapping_ready or (self.mapping_ready and now-self.mapping_ready_since>=1.)):
                scan_age=self.get_clock().now().nanoseconds*1e-9-self.scan_stamp
                if -.05<=scan_age<.5 and self.scan_points:
                    if self.close_wall_mode:
                        effective.linear.x,effective.linear.y=recovery.vx,recovery.vy
                    else:
                        effective.linear.x,effective.linear.y=self.guarded_velocity(recovery.vx,recovery.vy,self.scan_points,self.recovery.config.radius)
                    if recovery.state=='spin' and not any(math.hypot(x,y)<self.recovery.config.radius for x,y in self.scan_points):
                        effective.angular.z=recovery.wz;permitted=True
        if self.close_wall_mode:
            scan_age=self.get_clock().now().nanoseconds*1e-9-self.scan_stamp
            spin_clear=self.geometry.spin_clear(self.scan_points,scan_age)
            # A wall can forbid rotation while leaving translation along it valid.
            # Do not feed this permission back into the translation-ready flag above.
            if not spin_clear:
                permitted=False
                effective.angular.z=0.
            if -.05<=scan_age<.5 and self.scan_points:
                if self.physical_spin(effective.angular.z):
                    effective.linear.x,effective.linear.y=self.guarded_velocity(
                        effective.linear.x,effective.linear.y,self.scan_points,self.geometry.spin_radius)
                else:
                    effective.linear.x,effective.linear.y=self.geometry.guarded_velocity(
                        effective.linear.x,effective.linear.y,self.scan_points)
            else:
                effective=Twist();permitted=False
        self.publish_footprint(now,odom_valid,effective.angular.z)
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
