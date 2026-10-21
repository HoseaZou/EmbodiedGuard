"""Bounded forward hybrid-state search on a known static map.

Original implementation of the continuous (x,y,yaw) search design described by
Dolgov et al., Practical Search Techniques in Path Planning for Autonomous
Driving (2008): https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf
No claim of completeness, minimum cost, differential-drive, or dynamics safety.
Neither evaluator pose nor a model-provided map is an accepted planner input.
"""
from dataclasses import dataclass, field
import heapq
import math
import time

import numpy as np

from .sim import PlantConfig, PoseState, finite_number
from .reachability import certify_disconnection

@dataclass(frozen=True)
class PlannerConfig:
    max_expansions: int = 30000
    max_wall_seconds: float = 5.
    primitive_length_m: float = .8
    sample_step_m: float = .1
    xy_resolution_m: float = .4
    heading_bins: int = 72
    maximum_curvature_per_m: float = .27
    tracking_reserve_m: float = .35
    heuristic_weight: float = 1.4

    def __post_init__(self):
        if type(self.max_expansions) is not int or self.max_expansions < 1:
            raise ValueError('Invalid expansion bound')
        if type(self.heading_bins) is not int or self.heading_bins < 24:
            raise ValueError('Invalid heading resolution')
        for name in ('max_wall_seconds', 'primitive_length_m', 'sample_step_m',
                     'xy_resolution_m', 'maximum_curvature_per_m', 'heuristic_weight'):
            if finite_number(getattr(self, name), name) <= 0:
                raise ValueError('Planner bounds must be positive')
        if finite_number(self.tracking_reserve_m, 'tracking_reserve_m') < 0:
            raise ValueError('Tracking reserve must not be negative')
        if self.sample_step_m > .1 or self.primitive_length_m > 2.:
            raise ValueError('Planner spatial sampling is too coarse')

@dataclass
class PlanResult:
    ok: bool
    reason: str
    points: list = field(default_factory=list)
    diagnostics: dict = field(default_factory=dict)

class GlobalPlanner:
    def __init__(self, grid, scale, plant: PlantConfig, config=None):
        self.grid, self.scale, self.plant = grid, finite_number(scale, 'map scale'), plant
        if self.scale <= 0:
            raise ValueError('Map scale must be positive')
        self.config = config or PlannerConfig()
        self.curvature = min(self.config.maximum_curvature_per_m,
                             math.tan(plant.max_steer_rad * .5) / plant.wheelbase_m)
        self.radius = plant.radius_m + plant.safety_margin_m
        self.width, self.height = grid.width * self.scale, grid.height * self.scale
        self._distance_cache = None
        self._primitive = {}
        step_count = math.ceil(self.config.primitive_length_m / self.config.sample_step_m)
        distances = np.linspace(0., self.config.primitive_length_m, step_count + 1)[1:]
        for curvature in (-self.curvature, -self.curvature/2, 0., self.curvature/2, self.curvature):
            if curvature == 0:
                x, y = distances, np.zeros_like(distances)
            else:
                x, y = np.sin(curvature*distances)/curvature, (1-np.cos(curvature*distances))/curvature
            self._primitive[curvature] = (x, y, curvature*distances)

    def _cell(self, x, y):
        return (int(math.floor(x/self.scale)), self.grid.height-1-int(math.floor(y/self.scale)))

    def _inflated(self):
        blocked = self.grid.occupied.copy()
        # Holonomic center-grid diagnostic at the UNCHANGED physical margin.
        n = math.ceil(self.radius / self.scale + .5)
        for dy in range(-n, n+1):
            for dx in range(-n, n+1):
                if (max(abs(dx)-.5,0)*self.scale)**2+(max(abs(dy)-.5,0)*self.scale)**2 > self.radius**2 + 1e-9:
                    continue
                src = self.grid.occupied[max(0,-dy):min(self.grid.height,self.grid.height-dy),
                                         max(0,-dx):min(self.grid.width,self.grid.width-dx)]
                blocked[max(0,dy):min(self.grid.height,self.grid.height+dy),
                        max(0,dx):min(self.grid.width,self.grid.width+dx)] |= src
        xs=(np.arange(self.grid.width)+.5)*self.scale
        ys=(np.arange(self.grid.height)+.5)*self.scale
        blocked |= ((xs <= self.radius) | (self.width-xs <= self.radius))[None,:]
        blocked |= ((ys <= self.radius) | (self.height-ys <= self.radius))[:,None]
        return blocked

    def _distances(self, goal, deadline):
        cell = self._cell(*goal)
        if self._distance_cache is not None and self._distance_cache[0] == cell:
            return self._distance_cache[1:]
        blocked = self._inflated()
        distances = np.full(blocked.shape, np.inf)
        gx,gy=cell
        if not (0<=gx<self.grid.width and 0<=gy<self.grid.height) or blocked[gy,gx]:
            return distances, False
        distances[gy,gx]=0.
        queue=[(0.,gx,gy)]
        neighbors=((1,0,1.),(-1,0,1.),(0,1,1.),(0,-1,1.),
                   (1,1,math.sqrt(2)),(-1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(-1,-1,math.sqrt(2)))
        complete=True; expanded=0
        while queue:
            distance,x,y=heapq.heappop(queue)
            if distance > distances[y,x]+1e-9:
                continue
            expanded+=1
            if expanded%1024==0 and time.monotonic()>deadline:
                complete=False
                break
            for dx,dy,cost in neighbors:
                xx,yy=x+dx,y+dy
                if not (0<=xx<self.grid.width and 0<=yy<self.grid.height) or blocked[yy,xx]:
                    continue
                if dx and dy and (blocked[y,xx] or blocked[yy,x]):
                    continue
                new=distance+cost*self.scale
                if new+1e-9 < distances[yy,xx]:
                    distances[yy,xx]=new
                    heapq.heappush(queue,(new,xx,yy))
        if complete:
            self._distance_cache=(cell,distances,complete)
        return distances,complete

    def _clear(self, points, start, goal):
        # Sampled arc chords have sagitta <0.0004m at the configured resolution;
        # 0.005m extra covers this. The executor's independent continuous guard
        # remains mandatory, regardless of the planner's verdict.
        reserve=self.config.tracking_reserve_m
        for x,y in points:
            if min(x,y,self.width-x,self.height-y) <= self.radius+.005:
                return False
            extra=min(reserve, .10*math.hypot(x-start[0],y-start[1]), .10*math.hypot(x-goal[0],y-goal[1]))
            required=self.radius+math.sqrt(2)*self.scale/2+extra+.005
            col,row=self._cell(x,y)
            n=math.ceil(required/self.scale)+1
            x0,x1=max(0,col-n),min(self.grid.width,col+n+1)
            y0,y1=max(0,row-n),min(self.grid.height,row+n+1)
            rr,cc=np.nonzero(self.grid.occupied[y0:y1,x0:x1])
            if len(rr):
                ox=(cc+x0+.5)*self.scale
                oy=(self.grid.height-(rr+y0)-.5)*self.scale
                if np.any((ox-x)**2+(oy-y)**2 <= required**2):
                    return False
        return True

    def _connector(self, state, goal, start):
        x,y,yaw=state
        distance=math.hypot(goal[0]-x,goal[1]-y)
        if distance < 1e-6:
            return None
        bearing=math.atan2(goal[1]-y,goal[0]-x)
        delta=abs(math.atan2(math.sin(bearing-yaw),math.cos(bearing-yaw)))
        if delta > 1.5:
            return None
        count=math.ceil(distance/self.config.sample_step_m)
        u=np.linspace(0.,1.,max(3,count+1))
        p0,p1=np.array((x,y)),np.array(goal)
        t0=distance*np.array((math.cos(yaw),math.sin(yaw)))
        t1=distance*np.array((math.cos(bearing),math.sin(bearing)))
        p=(2*u**3-3*u**2+1)[:,None]*p0+(u**3-2*u**2+u)[:,None]*t0+(-2*u**3+3*u**2)[:,None]*p1+(u**3-u**2)[:,None]*t1
        d=(6*u**2-6*u)[:,None]*p0+(3*u**2-4*u+1)[:,None]*t0+(-6*u**2+6*u)[:,None]*p1+(3*u**2-2*u)[:,None]*t1
        dd=(12*u-6)[:,None]*p0+(6*u-4)[:,None]*t0+(-12*u+6)[:,None]*p1+(6*u-2)[:,None]*t1
        norms=np.linalg.norm(d,axis=1)
        if np.min(norms) < 1e-6:
            return None
        k=np.abs(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/norms**3
        if np.max(k) > self.curvature*.98:
            return None
        points=[tuple(a) for a in p]
        return (points,float(np.max(k))) if self._clear(points,start,goal) else None

    def plan(self, start: PoseState, goal):
        now=time.monotonic(); deadline=now+self.config.max_wall_seconds
        goal=tuple(finite_number(v,'goal') for v in goal)
        if len(goal)!=2:
            raise ValueError('Goal must contain two coordinates')
        for name in ('x','y','yaw_rad','speed_mps'):
            finite_number(getattr(start,name),name)
        diagnostics={'algorithm':'bounded_forward_hybrid_astar', 'global_infeasibility_proven':False,
                     'maximum_curvature_per_m':self.curvature, 'reverse_supported':False,
                     'map_source':'supervisor_known_static_grid', 'holonomic_grid_connected':None,
                     'holonomic_connectivity_is_not_ackermann_proof':True, 'expanded_nodes':0}
        origin=(start.x,start.y)
        def result(ok,reason,points=None):
            diagnostics['planning_wall_sec']=time.monotonic()-now
            return PlanResult(ok,reason,points or [],dict(diagnostics))
        if start.speed_mps>.05:
            return result(False,'NOT_STATIONARY')
        if not self._clear([origin,goal],origin,goal):
            return result(False,'PLANNING_ENDPOINT_CLEARANCE_REJECTED')
        connector=self._connector((start.x,start.y,start.yaw_rad),goal,origin)
        if connector:
            return result(True,'PLANNED',connector[0])
        distances,complete=self._distances(goal,deadline)
        sx,sy=self._cell(*origin)
        connected=bool(0<=sx<self.grid.width and 0<=sy<self.grid.height and math.isfinite(distances[sy,sx]))
        diagnostics['holonomic_grid_connected']=connected if complete else None
        if complete and not connected:
            proof=certify_disconnection(self.grid,self.scale,self.radius,origin,goal,deadline=deadline)
            diagnostics['continuous_free_space_cover']=proof
            diagnostics['global_infeasibility_proven']=proof['global_infeasibility_proven']
            if proof['global_infeasibility_proven']:
                return result(False,'PHYSICAL_FREE_SPACE_DISCONNECTED')
            return result(False,'NO_ROUTE_IN_INFLATED_GRID')
        resolution=min(self.config.xy_resolution_m,self.scale)
        def key(state):
            x,y,theta=state
            return (round(x/resolution),round(y/resolution),round((theta%(2*math.pi))*self.config.heading_bins/(2*math.pi))%self.config.heading_bins)
        def heuristic(state):
            x,y,theta=state;cx,cy=self._cell(x,y)
            distance=distances[cy,cx] if 0<=cx<self.grid.width and 0<=cy<self.grid.height else math.inf
            return max(math.hypot(x-goal[0],y-goal[1]),float(distance)) if math.isfinite(distance) else math.hypot(x-goal[0],y-goal[1])+10.
        # Nodes are immutable. Reopening a quantized key cannot overwrite a
        # parent used by an older child and introduce discontinuities.
        nodes=[((start.x,start.y,start.yaw_rad),0.,None,[],0.)]
        best={key(nodes[0][0]):0.}
        queue=[(self.config.heuristic_weight*heuristic(nodes[0][0]),0.,0)]
        chosen=None; tail=[]
        while queue:
            priority,cost,index=heapq.heappop(queue)
            state,g,parent,edge,previous_curvature=nodes[index]
            if g > best.get(key(state),math.inf)+1e-9:
                continue
            diagnostics['expanded_nodes']+=1
            if diagnostics['expanded_nodes']>self.config.max_expansions or time.monotonic()>deadline:
                return result(False,'SEARCH_BUDGET_EXHAUSTED')
            if diagnostics['expanded_nodes']%8==1 or math.hypot(state[0]-goal[0],state[1]-goal[1]) < 3.:
                connector=self._connector(state,goal,origin)
                if connector:
                    chosen=index;tail=connector[0][1:];break
            cosine,sine=math.cos(state[2]),math.sin(state[2])
            for curvature,(px,py,pyaw) in self._primitive.items():
                xx=state[0]+cosine*px-sine*py
                yy=state[1]+sine*px+cosine*py
                points=list(zip(xx.tolist(),yy.tolist()))
                if not self._clear(points,origin,goal):
                    continue
                target=(float(xx[-1]),float(yy[-1]),math.atan2(math.sin(state[2]+pyaw[-1]),math.cos(state[2]+pyaw[-1])))
                score=g+self.config.primitive_length_m*(1+.1*abs(curvature)/self.curvature)+.08*abs(curvature-previous_curvature)/self.curvature
                marker=key(target)
                if score+1e-9 >= best.get(marker,math.inf):
                    continue
                best[marker]=score
                new=len(nodes);nodes.append((target,score,index,points,curvature))
                heapq.heappush(queue,(score+self.config.heuristic_weight*heuristic(target),score,new))
        if chosen is None:
            return result(False,'NO_ROUTE_IN_SEARCH_RESOLUTION')
        edges=[]
        while nodes[chosen][2] is not None:
            edges.append(nodes[chosen][3]);chosen=nodes[chosen][2]
        points=[origin]
        for edge in reversed(edges):
            points.extend(edge)
        points.extend(tail)
        diagnostics['length_m']=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
        return result(True,'PLANNED',points)
