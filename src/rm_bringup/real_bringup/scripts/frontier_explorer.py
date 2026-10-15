#!/usr/bin/env python3
"""Automatic frontier targets for the existing Nav2 client, using only live SLAM maps."""
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String,UInt8MultiArray,Float64,Bool
from nav2_msgs.msg import Costmap
from std_srvs.srv import Trigger
from rm_interfaces.msg import Goal,NavigationStatus
from tf2_ros import Buffer,TransformListener
from frontier_planner import select_frontier
from scipy.ndimage import distance_transform_edt,label


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.require_mapping_ready=bool(self.declare_parameter('require_mapping_ready',False).value)
        self.mapping_ready=False;self.mapping_ready_since=0.
        if self.require_mapping_ready:self.create_subscription(Bool,'/pgo/map_ready',self.mapping_cb,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        self.clearance=float(self.declare_parameter('clearance',.48).value)
        self.min_cells=int(self.declare_parameter('min_frontier_cells',8).value)
        self.min_distance=float(self.declare_parameter('min_goal_distance',.8).value)
        self.timeout=float(self.declare_parameter('goal_timeout',90.).value)
        self.blacklist_duration=float(self.declare_parameter('blacklist_duration',180.).value)
        if not all(math.isfinite(v) and v>0 for v in [self.clearance,self.min_distance,self.timeout,self.blacklist_duration]) or self.min_cells<1:raise ValueError('Invalid exploration parameters')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.map=None;self.map_receipt=0.;self.map_started=None;self.costmap=None;self.costmap_receipt=0.;self.motion_receipt=-float("inf");self.invalid_ticks=0;self.client=None;self.client_receipt=0.
        self.need_clear=False;self.clear_future=None
        self.destination=None;self.destination_done=False;self.selected_is_destination=False
        self.clear_client=self.create_client(Trigger,"/navigation/client/clear_exploration_goal")
        self.selected=None;self.selected_time=0.;self.blacklist=[];self.sent=0;self.completed=0;self.failed=0;self.last_clock=0.
        self.yaw_pub=self.create_publisher(Float64,'/navigation/exploration_yaw_rate',1)
        self.goal_pub=self.create_publisher(Goal,'/navigation/exploration_goal',10)
        self.status_pub=self.create_publisher(String,'/exploration/status',10)
        self.create_subscription(OccupancyGrid,'/map',self.receive_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Goal,'/navigation/exploration_target',self.receive_destination,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(NavigationStatus,'/navigation/client/status',self.receive_status,10)
        self.create_subscription(Costmap,'/global_costmap/costmap_raw',self.receive_costmap,1)
        self.create_subscription(UInt8MultiArray,'/can_twist/command_frame',lambda m:setattr(self,'motion_receipt',self.now_seconds()) if len(m.data)==8 else None,1)
        self.create_timer(1.,self.tick)
    def mapping_cb(self,msg):
        if msg.data and not self.mapping_ready:self.mapping_ready_since=self.now_seconds()
        if not msg.data and self.mapping_ready:
            self.selected=None;self.blacklist=[];self.need_clear=True
        self.mapping_ready=msg.data

    def now_seconds(self):return self.get_clock().now().nanoseconds*1e-9
    def receive_map(self,msg):
        if msg.header.frame_id!='map' or not msg.info.width or msg.info.resolution<=0:return
        self.map=msg;self.map_receipt=self.now_seconds()
        if self.map_started is None:self.map_started=self.map_receipt
    def receive_costmap(self,msg):
        if msg.header.frame_id=="map":self.costmap=msg;self.costmap_receipt=self.now_seconds()
    def receive_destination(self,msg):
        self.destination=[msg.goal_x,msg.goal_y] if msg.goal_frame_id=='map' and all(math.isfinite(v) for v in (msg.goal_x,msg.goal_y)) else None
        self.destination_done=False;self.selected=None;self.blacklist=[];self.need_clear=False;self.invalid_ticks=0
    def receive_status(self,msg):self.client=msg;self.client_receipt=self.now_seconds()
    def status(self,state,frontiers=0):
        m=String();m.data=json.dumps({'state':state,'frontiers':frontiers,'goals_sent':self.sent,
            'completed':self.completed,'failed':self.failed,'goal':self.selected,'destination':self.destination},ensure_ascii=False);self.status_pub.publish(m)
    def remember(self,radius):
        self.blacklist.append((self.selected,self.now_seconds()+self.blacklist_duration,radius))
        self.selected=None
    def tick(self):
        now=self.now_seconds()
        self.yaw_pub.publish(Float64(data=0.))
        if now<self.last_clock:self.selected=None;self.blacklist=[]
        self.last_clock=now
        if self.client is None or not 0<=now-self.client_receipt<2.:self.status('waiting_client');return
        if self.client.mode not in ('explore','explore_goal'):self.selected=None;self.status('inactive');return
        if self.client.paused or self.client.emergency_stop:self.status('paused');return
        if self.destination_done:self.status('destination_reached');return
        if self.client.mode=='explore_goal' and self.destination is None:self.status('waiting_destination');return
        if self.need_clear:
            if self.clear_future is not None and self.clear_future.done():
                response=self.clear_future.result();self.clear_future=None
                if response and response.success:self.need_clear=False
            elif self.clear_future is None and self.clear_client.service_is_ready():self.clear_future=self.clear_client.call_async(Trigger.Request())
            self.status('replanning');return
        if self.require_mapping_ready and (not self.mapping_ready or now-self.mapping_ready_since<1.):self.status('waiting_mapping');return
        if self.map is None or not 0<=now-self.map_receipt<5. or now-self.map_started<5.:self.status('waiting_map');return
        if not 0<=now-self.motion_receipt<.5:self.status('waiting_motion');return
        if self.costmap is None or not 0<=now-self.costmap_receipt<2.:self.status('waiting_costmap');return
        try:
            t=self.buffer.lookup_transform('map','base_link',Time())
            age=now-t.header.stamp.sec-t.header.stamp.nanosec*1e-9
            if not -.05<=age<.5:raise ValueError('Stale pose')
            x,y=t.transform.translation.x,t.transform.translation.y
            if self.client.mode=='explore_goal' and self.destination is not None:
                q=t.transform.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                look_at=self.selected if self.selected is not None else self.destination
                desired=math.atan2(look_at[1]-y,look_at[0]-x)
                error=math.atan2(math.sin(desired-yaw),math.cos(desired-yaw))
                self.yaw_pub.publish(Float64(data=max(-.4,min(.4,error))))
        except Exception:self.status('waiting_pose');return
        msg=self.map;info=msg.info;q=info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));c,s=math.cos(yaw),math.sin(yaw)
        def to_rc(px,py):
            dx,dy=px-info.origin.position.x,py-info.origin.position.y
            return ((-s*dx+c*dy)/info.resolution,(c*dx+s*dy)/info.resolution)
        grid=np.asarray(msg.data,dtype=np.int16).reshape(info.height,info.width)
        cm=self.costmap;meta=cm.metadata;cg=np.asarray(cm.data,dtype=np.uint8).reshape(meta.size_y,meta.size_x)
        rr,cc=np.indices(grid.shape);mx=info.origin.position.x+c*(cc+.5)*info.resolution-s*(rr+.5)*info.resolution
        my=info.origin.position.y+s*(cc+.5)*info.resolution+c*(rr+.5)*info.resolution
        cr=np.floor((my-meta.origin.position.y)/meta.resolution).astype(int)
        co=np.floor((mx-meta.origin.position.x)/meta.resolution).astype(int)
        inside=(cr>=0)&(cr<meta.size_y)&(co>=0)&(co<meta.size_x)
        allowed=np.zeros(grid.shape,dtype=bool);allowed[inside]=cg[cr[inside],co[inside]]<253
        if self.selected:
            sr,sc=to_rc(*self.selected);sr,sc=int(sr),int(sc)
            valid=0<=sr<info.height and 0<=sc<info.width and allowed[sr,sc] and 0<=grid[sr,sc]<=25
            self.invalid_ticks=0 if valid else self.invalid_ticks+1
            target=self.client.target
            matched=math.hypot(target.goal_x-self.selected[0],target.goal_y-self.selected[1])<.001
            if matched and self.client.state=='complete':
                self.completed+=1;self.remember(.6)
                if self.selected_is_destination:self.destination_done=True;self.status('destination_reached');return
            elif matched and self.client.state in ('failed','rejected'):self.failed+=1;self.remember(1.)
            elif self.invalid_ticks>=3 or now-self.selected_time>self.timeout:
                self.failed+=1;self.remember(1.);self.need_clear=True;self.status('replanning');return
            else:self.status('navigating');return
        self.blacklist=[entry for entry in self.blacklist if entry[1]>now]
        excluded=[(to_rc(*xy),radius) for xy,expiry,radius in self.blacklist]
        robot=to_rc(x,y)
        if not 0<=robot[0]<info.height or not 0<=robot[1]<info.width:self.status('pose_out_of_map');return
        destination_rc=to_rc(*self.destination) if self.destination is not None else None
        rc=None;count=0;self.selected_is_destination=False
        if destination_rc is not None:
            dr,dc=int(math.floor(destination_rc[0])),int(math.floor(destination_rc[1]))
            if 0<=dr<info.height and 0<=dc<info.width:
                if grid[dr,dc]>25:self.status('destination_blocked');return
                free=(grid>=0)&(grid<=25)&allowed
                components,_=label(free,np.ones((3,3)))
                rr0,cc0=int(robot[0]),int(robot[1])
                clearance=distance_transform_edt(np.pad((grid>=0)&(grid<=25),1))[1:-1,1:-1]*info.resolution
                if free[dr,dc] and components[rr0,cc0]!=0 and components[rr0,cc0]==components[dr,dc] and clearance[dr,dc]>=self.clearance and not any(np.linalg.norm(np.asarray((dr,dc))-np.asarray(p))*info.resolution<radius for p,radius in excluded if radius>=1.):
                    rc=(dr,dc);self.selected_is_destination=True
        if rc is None:rc,count=select_frontier(grid,robot,info.resolution,self.clearance,self.min_cells,self.min_distance,excluded,allowed_cells=allowed,destination_rc=destination_rc)
        if rc is None:self.status('no_reachable_frontiers',count);return
        local_x=(rc[1]+.5)*info.resolution;local_y=(rc[0]+.5)*info.resolution
        gx=info.origin.position.x+c*local_x-s*local_y;gy=info.origin.position.y+s*local_x+c*local_y
        if self.selected_is_destination:gx,gy=self.destination
        self.selected=[gx,gy];self.selected_time=now;self.invalid_ticks=0;self.sent+=1
        goal=Goal();goal.goal_frame_id='map';goal.goal_x=gx;goal.goal_y=gy;goal.goal_z=0.
        self.goal_pub.publish(goal);self.status('navigating',count)
        self.get_logger().info(f'Exploration goal {self.sent}: ({gx:.2f}, {gy:.2f}), frontier groups={count}')


def main():
    rclpy.init();node=FrontierExplorer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown() if rclpy.ok() else None

if __name__=='__main__':main()
