"""Pure occupancy-grid frontier selection; candidates stay inside observed free space."""
import math
import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, label
from scipy.spatial import cKDTree


def select_frontier(grid, robot_rc, resolution, clearance=.48, min_cells=8,
                    min_distance=.8, blacklist=(), allowed_cells=None, destination_rc=None):
    free=(grid>=0)&(grid<=25)
    safe=free&(distance_transform_edt(np.pad(free,1,constant_values=False))[1:-1,1:-1]*resolution>=clearance)
    if allowed_cells is not None:safe &= allowed_cells
    candidates=np.argwhere(safe)
    if not len(candidates):return None,0
    robot=np.asarray(robot_rc,dtype=float)
    tree=cKDTree(candidates)
    distance,index=tree.query(robot)
    if distance*resolution>clearance+.3:return None,0
    seed=tuple(candidates[index]);components,_=label(safe,np.ones((3,3)))
    reachable=safe&(components==components[seed])
    candidates=np.argwhere(reachable);tree=cKDTree(candidates)
    frontier=free&binary_dilation(grid<0,structure=np.ones((3,3)))
    # The padded array treats cells beyond the map as unknown too.
    frontier[0,:]|=free[0,:];frontier[-1,:]|=free[-1,:]
    frontier[:,0]|=free[:,0];frontier[:,-1]|=free[:,-1]
    clusters,count=label(frontier,np.ones((3,3)))
    best=None;best_score=math.inf;valid_clusters=0
    for cluster in range(1,count+1):
        anchors=np.argwhere(clusters==cluster)
        if len(anchors)<min_cells:continue
        valid_clusters+=1
        distances=np.linalg.norm(anchors-robot,axis=1)
        # Sample both nearby and distant anchors; a circular frontier's centroid
        # can lie at the robot and must not be used as its navigation target.
        ordered=anchors[np.argsort(distances)]
        sampled=ordered[np.unique(np.linspace(0,len(ordered)-1,min(32,len(ordered))).astype(int))]
        _,indices=tree.query(sampled)
        for anchor,idx in zip(sampled,indices):
            goal=candidates[idx]
            travel=float(np.linalg.norm(goal-robot))*resolution
            if travel<min_distance or np.linalg.norm(goal-anchor)*resolution>1.5:continue
            if any(np.linalg.norm(goal-np.asarray(p))*resolution<radius for p,radius in blacklist):continue
            score=travel-.15*math.log1p(len(anchors))
            if destination_rc is not None:score+=2.*float(np.linalg.norm(goal-np.asarray(destination_rc)))*resolution
            if score<best_score:best_score=score;best=tuple(int(v) for v in goal)
    return best,valid_clusters
