#!/usr/bin/env python3
"""Bounded recovery policy; all poses/grid coordinates are in odom, outputs in body.

A map overlap is evidence of a localization/costmap problem, not permission to
cross a physical wall. Every escape also needs fresh, observed live clearance.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass
class RecoveryConfig:
    no_path_timeout: float = 8.
    overlap_timeout: float = 1.
    spin_speed: float = .35
    spin_angle: float = math.pi
    spin_timeout: float = 12.
    retreat_speed: float = .08
    retreat_distance: float = .40
    retreat_timeout: float = 8.
    cooldown: float = 8.
    max_attempts: int = 3
    radius: float = .47
    progress_distance: float = .20

    def validate(self):
        if any(not math.isfinite(v) or v<=0 for v in self.__dict__.values()):raise ValueError('Recovery limits must be finite and positive')
        if not .45<=self.radius<=.8 or self.spin_speed>.6 or self.retreat_speed>.15 or self.retreat_distance>.6 or self.max_attempts>10 or self.max_attempts!=int(self.max_attempts):raise ValueError('Unsafe recovery bounds')


class Grid:
    def __init__(self,data,resolution,origin=(0.,0.,0.)):
        self.data=np.asarray(data,dtype=np.uint8)
        if self.data.ndim!=2 or not self.data.size or not math.isfinite(resolution) or resolution<=0 or not all(math.isfinite(x) for x in origin):raise ValueError('Invalid recovery costmap')
        self.resolution=resolution;self.origin=origin
        self.c,self.s=math.cos(origin[2]),math.sin(origin[2])
        # Unknown is blocked, never an invented escape destination.
        blocked=self.data>=253
        self.depth=distance_transform_edt(np.pad(blocked,1,constant_values=True))[1:-1,1:-1]*resolution if (~blocked).any() else np.full(self.data.shape,np.inf)

    def sample(self,x,y):
        dx,dy=x-self.origin[0],y-self.origin[1]
        col=math.floor((self.c*dx+self.s*dy)/self.resolution);row=math.floor((-self.s*dx+self.c*dy)/self.resolution)
        if row<0 or col<0 or row>=self.data.shape[0] or col>=self.data.shape[1]:return 255,math.inf
        return int(self.data[row,col]),float(self.depth[row,col])


def travel_clearance(direction,points,radius):
    ux,uy=direction;clearance=math.inf
    for x,y in points:
        along=x*ux+y*uy;across=abs(x*uy-y*ux)
        if along<=0 or across>=radius:continue
        clearance=min(clearance,along-math.sqrt(radius*radius-across*across))
    return max(0.,clearance)


def observed_corridor(direction,points,distance,radius):
    """Each of three angular sectors must contain an actual distant return."""
    angle=math.atan2(direction[1],direction[0]);half=math.atan2(radius,distance+radius)
    sectors=set()
    for x,y in points:
        delta=math.atan2(math.sin(math.atan2(y,x)-angle),math.cos(math.atan2(y,x)-angle))
        if abs(delta)<=half and math.hypot(x,y)>=distance+radius+.05:
            sectors.add(min(2,int((delta+half)/(2*half)*3)))
    return len(sectors)==3


def escape_direction(grid,pose,points,cfg):
    x,y,yaw=pose;_,initial_depth=grid.sample(x,y)
    if not math.isfinite(initial_depth):return None
    best=None
    for angle in np.linspace(-math.pi,math.pi,32,endpoint=False):
        body=(math.cos(angle),math.sin(angle));world=(math.cos(angle+yaw),math.sin(angle+yaw))
        clearance=travel_clearance(body,points,cfg.radius)
        for distance in [cfg.retreat_distance*.5,cfg.retreat_distance*.75,cfg.retreat_distance]:
            if clearance<distance+.03 or not observed_corridor(body,points,distance,cfg.radius):continue
            previous=initial_depth;valid=True
            for d in np.linspace(0,distance,max(3,math.ceil(distance/(grid.resolution*.5)))):
                cost,depth=grid.sample(x+world[0]*d,y+world[1]*d)
                if cost==255 or depth>initial_depth+grid.resolution*.1 or depth>previous+grid.resolution*.1:valid=False;break
                previous=depth
            end_cost,_=grid.sample(x+world[0]*distance,y+world[1]*distance)
            if not valid or end_cost>=200:continue
            score=-distance-.002*end_cost+.03*min(clearance,2.)-.01*body[0]
            if best is None or score>best[0]:best=(score,world,distance)
    return None if best is None else (best[1],best[2])


@dataclass
class Decision:
    state: str = 'idle'
    active: bool = False
    override: bool = False
    vx: float = 0.
    vy: float = 0.
    wz: float = 0.
    reason: str = ''


class Recovery:
    def __init__(self,config):
        config.validate();self.config=config;self.goal=None;self.state='idle';self.attempts=0
        self.bad_since=None;self.overlap_since=None;self.progress_pose=None;self.until=0.;self.started=0.
        self.start_pose=None;self.direction=None;self.distance=0.;self.angle=0.;self.last_yaw=None;self.last_now=-math.inf

    def cancel(self):
        self.state='idle';self.bad_since=None;self.overlap_since=None;self.last_yaw=None

    def finish(self,now,pose,reason):
        self.state='cooldown';self.until=now+self.config.cooldown;self.progress_pose=pose[:2]
        self.bad_since=None;self.overlap_since=None
        return Decision('replanning',override=True,reason=reason)

    def tick(self,now,enabled,goal,pose,grid,points,path_valid):
        cfg=self.config
        if now<self.last_now:self.cancel();self.until=now+cfg.cooldown
        self.last_now=now
        if not enabled or grid is None or not points:
            active=self.state in ('retreat','spin','blocked');self.cancel()
            return Decision('waiting_sensors' if enabled else 'inactive',override=active)
        if goal!=self.goal:
            self.cancel();self.goal=goal;self.attempts=0;self.progress_pose=pose[:2];self.until=now
        cost,_=grid.sample(pose[0],pose[1]);overlap=253<=cost<=254
        if cost==255:
            self.cancel();return Decision('unknown_robot_cell',override=True,reason='Robot cell is unknown or outside local costmap')
        if self.state=='spin':
            if overlap:return self.finish(now,pose,'Map overlap interrupted spin')
            if any(math.hypot(x,y)<cfg.radius for x,y in points):return self.finish(now,pose,'Obstacle interrupted spin')
            self.angle+=math.atan2(math.sin(pose[2]-self.last_yaw),math.cos(pose[2]-self.last_yaw));self.last_yaw=pose[2]
            if self.angle>=cfg.spin_angle or now-self.started>=cfg.spin_timeout:return self.finish(now,pose,'Spin complete; replan original goal')
            return Decision('spin',True,True,wz=cfg.spin_speed,reason='No usable path or progress')
        if self.state=='retreat':
            travelled=math.hypot(pose[0]-self.start_pose[0],pose[1]-self.start_pose[1])
            if (not overlap and travelled>=.06) or travelled>=self.distance or now-self.started>=cfg.retreat_timeout:return self.finish(now,pose,'Escape finished; replan original goal')
            c,s=math.cos(pose[2]),math.sin(pose[2]);ux=c*self.direction[0]+s*self.direction[1];uy=-s*self.direction[0]+c*self.direction[1]
            if travel_clearance((ux,uy),points,cfg.radius)<.06 or not observed_corridor((ux,uy),points,min(.15,max(.06,self.distance-travelled)),cfg.radius):return self.finish(now,pose,'New obstacle or unobserved escape corridor')
            # Re-evaluate the remaining MAP segment after every map update.
            _,depth=grid.sample(pose[0],pose[1]);previous=depth
            for d in np.linspace(0,min(.12,max(0.,self.distance-travelled)),5):
                sample,new_depth=grid.sample(pose[0]+self.direction[0]*d,pose[1]+self.direction[1]*d)
                if sample==255 or new_depth>previous+grid.resolution*.1:return self.finish(now,pose,'Map no longer supports escape')
                previous=new_depth
            return Decision('retreat',True,True,ux*cfg.retreat_speed,uy*cfg.retreat_speed,reason='Move toward observed free area')
        if self.state=='blocked':
            if not overlap:return self.finish(now,pose,'Overlap cleared')
            if now-self.started>=2.:return self.finish(now,pose,'No observed escape corridor')
            return Decision('blocked',True,True,reason='No safe observed direction; holding')
        if self.progress_pose is None:self.progress_pose=pose[:2]
        progress=math.hypot(pose[0]-self.progress_pose[0],pose[1]-self.progress_pose[1])
        if progress>=cfg.progress_distance:
            self.bad_since=None;self.attempts=0;self.progress_pose=pose[:2]
        if overlap:self.overlap_since=now if self.overlap_since is None else self.overlap_since
        else:self.overlap_since=None
        # A fresh path with no physical progress can still be unusable to MPPI.
        self.bad_since=now if self.bad_since is None else self.bad_since
        if now<self.until:return Decision('cooldown',override=overlap)
        if self.attempts>=cfg.max_attempts:return Decision('exhausted',override=overlap,reason='Recovery attempt limit reached')
        if self.overlap_since is not None and now-self.overlap_since>=cfg.overlap_timeout:
            escape=escape_direction(grid,pose,points,cfg);self.attempts+=1;self.started=now;self.start_pose=pose
            if escape is None:self.state='blocked';return Decision('blocked',True,True,reason='No safe observed escape direction')
            self.direction,self.distance=escape;self.state='retreat'
            return self.tick(now,enabled,goal,pose,grid,points,path_valid)
        if overlap:return Decision('confirming_overlap',override=True)
        if now-self.bad_since>=cfg.no_path_timeout:
            if any(math.hypot(x,y)<cfg.radius for x,y in points):return Decision('rotation_blocked',override=not path_valid,reason='Live obstacle inside swept radius; keep following a valid path')
            self.attempts+=1;self.started=now;self.last_yaw=pose[2];self.angle=0.;self.state='spin'
            return Decision('spin',True,True,wz=cfg.spin_speed,reason='No progress despite path' if path_valid else 'No valid path')
        return Decision('monitoring')
