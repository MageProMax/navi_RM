#!/usr/bin/env python3
"""Project actual returns after compensating scan history into the latest base pose.

A missing angular bin is NaN, never a fabricated clear ray. Accumulation is in
odom so movement does not smear past points in the current robot frame.
"""
from collections import deque
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def project(points, increment, low, high, minimum=.3, maximum=10.):
    ranges = np.full(int(math.ceil(2*math.pi/increment)), np.nan)
    radius = np.hypot(points[:, 0], points[:, 1])
    valid = np.isfinite(points).all(axis=1) & (points[:, 2]>=low) & (points[:, 2]<=high) & (radius>=minimum) & (radius<maximum)
    selected = points[valid]
    bins = np.floor((np.arctan2(selected[:, 1], selected[:, 0])+math.pi)/increment).astype(int)
    bins = np.clip(bins, 0, len(ranges)-1)
    temporary = np.full(len(ranges), np.inf)
    np.minimum.at(temporary, bins, radius[valid])
    ranges[np.isfinite(temporary)] = temporary[np.isfinite(temporary)]
    return ranges


def matrix(t):
    q=t.transform.rotation;p=t.transform.translation
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]),np.array([p.x,p.y,p.z])


class AccumulatedScan(Node):
    def __init__(self):
        super().__init__('accumulated_scan')
        self.history=float(self.declare_parameter('history_seconds',1.).value)
        self.period=float(self.declare_parameter('publish_period',.2).value)
        self.low=float(self.declare_parameter('min_height',.10).value)
        self.high=float(self.declare_parameter('max_height',.7).value)
        self.increment=float(self.declare_parameter('angle_increment',math.pi/180).value)
        if not (0<self.history<=2 and 0<self.period<=self.history and self.low<self.high and 0<self.increment<.1):
            raise ValueError('Invalid accumulated scan parameters')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.clouds=deque();self.pending=deque(maxlen=10);self.last_stamp=None;self.published=-math.inf
        self.pub=self.create_publisher(LaserScan,'scan',qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/fastlio2/body_cloud',self.pending.append,qos_profile_sensor_data)
        self.create_timer(.02,self.tick)

    def tick(self):
        # TF and cloud travel independently. Retry queued scans rather than
        # extrapolating using the newest transform or silently shifting timestamps.
        while self.pending:
            msg=self.pending[0];stamp=Time.from_msg(msg.header.stamp);seconds=stamp.nanoseconds*1e-9
            now=self.get_clock().now().nanoseconds*1e-9
            if now-seconds>.5 or seconds-now>.05:
                self.pending.popleft();continue
            try:r,t=matrix(self.buffer.lookup_transform('odom',msg.header.frame_id,stamp))
            except Exception:return
            self.pending.popleft()
            if self.last_stamp is not None and seconds<=self.last_stamp:
                self.clouds.clear();self.published=-math.inf
            self.last_stamp=seconds
            raw=point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)
            if isinstance(raw,np.ndarray) and raw.dtype.names:
                points=np.column_stack([raw[name].ravel() for name in ('x','y','z')]).astype(float)
            else:points=np.asarray(list(raw),dtype=float).reshape(-1,3)
            self.clouds.append((seconds,points@r.T+t))
            while self.clouds and seconds-self.clouds[0][0]>self.history:self.clouds.popleft()
            if seconds-self.published<self.period or not len(points):continue
            # Wait for a complete history before initializing scan matching.
            if seconds-self.clouds[0][0]<self.history-.15:continue
            joined=np.concatenate([p for _,p in self.clouds]);body=(joined-t)@r
            ranges=project(body,self.increment,self.low,self.high)
            if np.count_nonzero(np.isfinite(ranges))<20:continue
            scan=LaserScan();scan.header=msg.header;scan.angle_min=-math.pi
            scan.angle_increment=self.increment;scan.angle_max=-math.pi+(len(ranges)-1)*self.increment
            scan.range_min=.3;scan.range_max=10.;scan.scan_time=self.period
            scan.ranges=ranges.astype(np.float32).tolist();self.pub.publish(scan);self.published=seconds


def main():
    rclpy.init();node=AccumulatedScan()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
