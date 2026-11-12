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
from robot_geometry import RobotGeometry
from ground_filter import estimate_ground, relative_height


def project(points, increment, low, high, minimum=.05, maximum=10.):
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


def clearing_endpoints(points, origin, plane, increment, low, high,
                       maximum=10., height_margin=.035, stop_margin=.10):
    """Measured ray prefixes inside the obstacle band, never missing-bin rays.

    Ground/ceiling returns are clipped where the beam leaves the height band.
    Current obstacles cap clearing in their angular bin and adjacent bins.
    Returned points retain the actual beam direction and are clearing-only.
    """
    if plane is None or not len(points):
        return np.empty((0,3))
    points=points[np.isfinite(points).all(axis=1)]
    origin=np.asarray(origin,dtype=float)
    h0=origin[2]-origin[:2]@plane[:2]-plane[2]
    lower,upper=low+height_margin,high-height_margin
    if not lower<h0<upper:
        return np.empty((0,3))
    delta=points-origin
    radius=np.hypot(delta[:,0],delta[:,1])
    valid=radius>.05
    points,delta,radius=points[valid],delta[valid],radius[valid]
    if not len(points):
        return np.empty((0,3))
    height=points[:,2]-points[:,:2]@plane[:2]-plane[2]
    count=int(math.ceil(2*math.pi/increment))
    bins=np.clip(np.floor((np.arctan2(delta[:,1],delta[:,0])+math.pi)/increment).astype(int),0,count-1)
    blockers=np.full(count,np.inf)
    obstacle=(height>=low)&(height<=high)
    np.minimum.at(blockers,bins[obstacle],radius[obstacle])
    blockers=np.minimum(np.minimum(blockers,np.roll(blockers,1)),np.roll(blockers,-1))
    fraction=np.ones(len(points))
    below=height<lower;above=height>upper
    fraction[below]=(h0-lower)/(h0-height[below])
    fraction[above]=(upper-h0)/(height[above]-h0)
    length=np.minimum(np.minimum(radius*fraction,maximum),blockers[bins]-stop_margin)
    valid=length>.05
    delta,radius,bins,length=delta[valid],radius[valid],bins[valid],length[valid]
    if not len(length):
        return np.empty((0,3))
    # Farthest supported ray per bin; do not synthesize angular-bin centres.
    order=np.argsort(length,kind='stable')[::-1]
    _,indices=np.unique(bins[order],return_index=True)
    indices=order[indices]
    return origin+delta[indices]*(length[indices]/radius[indices])[:,None]


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
        self.cloud_topic=str(self.declare_parameter('cloud_topic','/fastlio2/body_cloud').value)
        self.period=float(self.declare_parameter('publish_period',.2).value)
        self.low=float(self.declare_parameter('min_height',.10).value)
        self.high=float(self.declare_parameter('max_height',.7).value)
        self.increment=float(self.declare_parameter('angle_increment',math.pi/180).value)
        self.minimum_bins=int(self.declare_parameter('minimum_valid_bins',20).value)
        self.ground_enabled=bool(self.declare_parameter('ground_filter_enabled',True).value)
        self.ground_radius=float(self.declare_parameter('ground_fit_radius',4.).value)
        self.ground_slope=float(self.declare_parameter('ground_max_slope_deg',12.).value)
        self.ground_offset=float(self.declare_parameter('ground_max_offset',.08).value)
        self.ground_threshold=float(self.declare_parameter('ground_distance_threshold',.035).value)
        self.ground_minimum=int(self.declare_parameter('ground_min_points',30).value)
        self.ground_max_age=float(self.declare_parameter('ground_model_max_age',.5).value)
        self.ground_model=None
        self.projection_ground=None
        self.clearing_enabled=bool(self.declare_parameter('clearing_enabled',False).value)
        self.clearing_margin=float(self.declare_parameter('clearing_height_margin',.035).value)
        self.clearing_stop=float(self.declare_parameter('clearing_stop_margin',.10).value)
        self.clearing_origin=np.array([
            float(self.declare_parameter('clearing_sensor_x',0.).value),
            float(self.declare_parameter('clearing_sensor_y',.22157).value),
            float(self.declare_parameter('clearing_sensor_z',.33018).value)])
        self.geometry=RobotGeometry(
            length=float(self.declare_parameter('body_length',.633).value),
            width=float(self.declare_parameter('body_width',.526).value),
            height=float(self.declare_parameter('body_height',.42).value))
        if not (0<=self.history<=2 and 0<self.period<=max(self.history,.1) and self.low<self.high and 0<self.increment<.1 and self.minimum_bins>0):
            raise ValueError('Invalid accumulated scan parameters')
        if self.ground_enabled and not (0<self.ground_radius<=10 and 0<self.ground_slope<=15 and
                0<self.ground_offset<=.1 and 0<self.ground_threshold<.1 and
                self.ground_minimum>=10 and 0<=self.ground_max_age<=1):
            raise ValueError('Invalid ground filter parameters')
        if self.clearing_enabled and not (np.isfinite(self.clearing_stop) and
                self.clearing_margin>0 and self.clearing_stop>0 and
                2*self.clearing_margin<self.high-self.low and np.isfinite(self.clearing_origin).all()):
            raise ValueError('Invalid clearing ray parameters')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.clouds=deque();self.pending=deque(maxlen=10);self.last_stamp=None;self.published=-math.inf
        self.pub=self.create_publisher(LaserScan,'scan',qos_profile_sensor_data)
        self.clearing_pub=(self.create_publisher(PointCloud2,'clearing_cloud',qos_profile_sensor_data)
                           if self.clearing_enabled else None)
        self.create_subscription(PointCloud2,self.cloud_topic,self.pending.append,qos_profile_sensor_data)
        self.create_timer(.02,self.tick)

    def tick(self):
        # TF and cloud travel independently. Retry queued scans rather than
        # extrapolating using the newest transform or silently shifting timestamps.
        while self.pending:
            msg=self.pending[0];stamp=Time.from_msg(msg.header.stamp);seconds=stamp.nanoseconds*1e-9
            if msg.header.frame_id != 'base_link':
                self.pending.popleft();continue
            now=self.get_clock().now().nanoseconds*1e-9
            if now-seconds>.5 or seconds-now>.05:
                self.pending.popleft();continue
            try:r,t=matrix(self.buffer.lookup_transform('odom',msg.header.frame_id,stamp))
            except Exception:return
            self.pending.popleft()
            if self.last_stamp is not None and seconds<=self.last_stamp:
                self.clouds.clear();self.published=-math.inf
                self.ground_model=None
            self.last_stamp=seconds
            raw=point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)
            if isinstance(raw,np.ndarray) and raw.dtype.names:
                points=np.column_stack([raw[name].ravel() for name in ('x','y','z')]).astype(float)
            else:points=np.asarray(list(raw),dtype=float).reshape(-1,3)
            # Filter in each acquisition's body frame BEFORE adding history.
            # Otherwise self returns turn into stationary trails after motion.
            points=self.geometry.outside_body(points)
            self.clouds.append((seconds,points@r.T+t))
            while self.clouds and seconds-self.clouds[0][0]>self.history:self.clouds.popleft()
            if seconds-self.published<self.period:continue
            # Wait for a complete history before initializing scan matching.
            if seconds-self.clouds[0][0]<self.history-.15:continue
            joined=np.concatenate([p for _,p in self.clouds]);body=(joined-t)@r
            projection=self.ground_relative(body,points,r,t,seconds)
            if self.clearing_enabled:
                # Fresh returns only: history may still contain an old moving
                # object. Empty clouds replace stale clearing observations.
                ends=clearing_endpoints(points,self.clearing_origin,self.projection_ground,
                    self.increment,self.low,self.high,height_margin=self.clearing_margin,
                    stop_margin=self.clearing_stop)
                self.clearing_pub.publish(point_cloud2.create_cloud_xyz32(msg.header,ends.tolist()))
            ranges=project(projection,self.increment,self.low,self.high)
            valid_bins=np.count_nonzero(np.isfinite(ranges))
            # Clearing needs a fresh marking observation even for an empty or
            # sparse frame; otherwise Nav2 keeps re-marking its last obstacle.
            if valid_bins<self.minimum_bins and not self.clearing_enabled:continue
            scan=LaserScan();scan.header=msg.header;scan.angle_min=-math.pi
            scan.angle_increment=self.increment;scan.angle_max=-math.pi+(len(ranges)-1)*self.increment
            scan.range_min=.05;scan.range_max=10.;scan.scan_time=self.period
            scan.ranges=ranges.astype(np.float32).tolist();self.pub.publish(scan);self.published=seconds

    def ground_relative(self,body,current,r,t,seconds):
        self.projection_ground=None
        if not self.ground_enabled:
            return body
        plane=estimate_ground(current,self.ground_radius,self.ground_slope,
                              self.ground_offset,self.ground_threshold,self.ground_minimum)
        if plane is not None:
            # Cache in odom, so a briefly sparse scan can reuse the floor after
            # translation or yaw without leaving a floor attached to the robot.
            normal=r@np.array([-plane[0],-plane[1],1.])
            self.ground_model=(seconds,normal,-plane[2]-normal@t)
        if self.ground_model is None:
            return body
        stamp,normal,offset=self.ground_model
        if not 0<=seconds-stamp<=self.ground_max_age:
            return body
        local=r.T@normal
        plane=np.array([-local[0],-local[1],-offset-normal@t])/local[2]
        if abs(plane[2])>self.ground_offset:
            return body
        self.projection_ground=plane
        return relative_height(body,plane)


def main():
    rclpy.init();node=AccumulatedScan()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
