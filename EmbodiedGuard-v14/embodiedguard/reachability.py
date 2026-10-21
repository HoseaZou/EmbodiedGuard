"""Optimistic free-space cover: disconnection certifies circular-footprint failure.

Each fine tile is blocked only when ALL its points violate the unchanged disk
margin, using a distance Lipschitz upper bound. Eight-neighbor adjacency includes
corner contact, so the cover overapproximates continuous free-space connectivity.
Connected or budget-limited covers never prove a forward Ackermann path exists.
"""
from collections import deque
import hashlib
import math
import time
import numpy as np

_CACHE={}

def optimistic_cover(grid,scale,radius,factor=4):
    if factor<2 or scale<=0 or radius<=0:
        raise ValueError('Invalid reachability discretization')
    key=(hashlib.sha256(grid.occupied.tobytes()).hexdigest(),grid.width,grid.height,scale,radius,factor)
    if key in _CACHE:
        return _CACHE[key]
    fine=scale/factor
    original=np.repeat(np.repeat(grid.occupied,factor,axis=0),factor,axis=1)
    blocked=original.copy();height,width=blocked.shape
    reduced=radius-fine/math.sqrt(2)
    if reduced>0:
        n=math.ceil(reduced/fine+.5)
        for dy in range(-n,n+1):
            for dx in range(-n,n+1):
                if (max(abs(dx)-.5,0)*fine)**2+(max(abs(dy)-.5,0)*fine)**2 > reduced**2+1e-12:
                    continue
                blocked[max(0,dy):min(height,height+dy),max(0,dx):min(width,width+dx)] |= original[
                    max(0,-dy):min(height,height-dy),max(0,-dx):min(width,width-dx)]
        xs=(np.arange(width)+.5)*fine;ys=(np.arange(height)+.5)*fine
        blocked |= ((xs<=reduced)|(width*fine-xs<=reduced))[None,:]
        blocked |= ((ys<=reduced)|(height*fine-ys<=reduced))[:,None]
    if len(_CACHE)>=6:
        _CACHE.pop(next(iter(_CACHE)))
    _CACHE[key]=blocked
    return blocked

def certify_disconnection(grid,scale,radius,start,goal,*,max_cells=200000,deadline=None):
    blocked=optimistic_cover(grid,scale,radius)
    fine=scale/4;h,w=blocked.shape
    def cell(point):
        return int(math.floor(point[0]/fine)),h-1-int(math.floor(point[1]/fine))
    sx,sy=cell(start);gx,gy=cell(goal)
    result={'method':'optimistic_continuous_disk_free_space_cover','tile_m':fine,
            'radius_with_safety_margin_m':radius,'global_infeasibility_proven':False,
            'connected':None,'visited_tiles':0,'ackermann_feasibility_proven':False}
    if not all(0<=x<w and 0<=y<h and not blocked[y,x] for x,y in ((sx,sy),(gx,gy))):
        result['status']='ENDPOINT_NOT_IN_OPTIMISTIC_COVER'
        return result
    visited=np.zeros_like(blocked);visited[sy,sx]=True
    queue=deque([(sx,sy)])
    neighbors=((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1))
    while queue:
        x,y=queue.popleft();result['visited_tiles']+=1
        if (x,y)==(gx,gy):
            result.update(connected=True,status='CONNECTED_OVERAPPROXIMATION')
            return result
        if result['visited_tiles']>=max_cells or (result['visited_tiles']%1024==0 and deadline is not None and time.monotonic()>deadline):
            result['status']='COVER_BUDGET_EXHAUSTED'
            return result
        for dx,dy in neighbors:
            xx,yy=x+dx,y+dy
            if 0<=xx<w and 0<=yy<h and not blocked[yy,xx] and not visited[yy,xx]:
                visited[yy,xx]=True;queue.append((xx,yy))
    result.update(connected=False,status='PROVEN_DISCONNECTED_FREE_SPACE_COVER',global_infeasibility_proven=True)
    return result
