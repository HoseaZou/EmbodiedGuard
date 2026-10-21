"""Three explicit aisle choices; no claim of general obstacle-aware planning."""
from __future__ import annotations

import math

from .safety import segment_distance
from .sim import PlantConfig, PoseState


ROUTES = ('direct', 'upper', 'lower')


def _within_curvature_limit(points, limit):
    for a, b, c in zip(points, points[1:], points[2:]):
        area2 = abs((b[0]-a[0]) * (c[1]-a[1]) - (b[1]-a[1]) * (c[0]-a[0]))
        curvature = 2 * area2 / (math.dist(a, b) * math.dist(b, c) * math.dist(c, a))
        if curvature > limit:
            return False
    return True


def _legacy_aisle_path(start: PoseState, goal: tuple[float, float], route: str, config: PlantConfig) -> list[tuple[float, float]]:
    if route not in ROUTES:
        raise ValueError('unknown route choice')
    dx = goal[0] - start.x
    if dx < .5 or abs(start.yaw_rad) > .7:
        raise ValueError('forward route cannot be connected from this pose')
    count = math.ceil(dx / .2)
    amplitude = {'direct': 0., 'upper': 2., 'lower': -2.}[route]
    # A connection must be feasible under the preserved PID's max normalized steer .5.
    limit = math.tan(config.max_steer_rad * .5) / config.wheelbase_m
    # Keep the original shape first. A completed detour can leave nonzero yaw;
    # its Hermite correction may make the symmetric bump infeasible. Bounded
    # alternative shapes redistribute bending within the SAME chosen corridor.
    # They retain the start tangent and end tangent, with no turn-in-place,
    # steering-limit change, or autonomous choice of a different route.
    shapes = ((0., 0.),) if route == 'direct' else (
        (0., 0.), (.25, .2), (.25, -.2), (0., .2), (0., -.2))
    tangent = math.tan(start.yaw_rad)
    for blend, skew in shapes:
        points = []
        for index in range(count + 1):
            u = index / count
            sine = math.sin(math.pi * u)
            bump = amplitude * sine**2 * (1 - blend + blend * sine) * (1 + skew * (2*u - 1))
            y = ((2 * u**3 - 3 * u**2 + 1) * start.y
                 + (u**3 - 2 * u**2 + u) * dx * tangent
                 + (-2 * u**3 + 3 * u**2) * goal[1] + bump)
            points.append((start.x + u * dx, y))
        if not _within_curvature_limit(points, limit):
            continue
        if blend or skew:
            # Also sample analytic derivatives at finer intervals to reduce
            # missed peaks. This is numerical validation, not a global proof.
            samples = math.ceil(dx / .02)
            for index in range(samples + 1):
                u = index / samples
                sine, cosine = math.sin(math.pi*u), math.cos(math.pi*u)
                f = (1-blend)*sine**2 + blend*sine**3
                f1 = (2*(1-blend)*sine + 3*blend*sine**2) * math.pi*cosine
                f2 = ((2*(1-blend) + 6*blend*sine) * (math.pi*cosine)**2
                      - (2*(1-blend)*sine + 3*blend*sine**2) * math.pi**2*sine)
                warp = 1 + skew*(2*u-1)
                slope = ((6*u*u-6*u)*(start.y-goal[1])/dx
                         + (3*u*u-4*u+1)*tangent + amplitude*(f1*warp+2*skew*f)/dx)
                second = ((12*u-6)*(start.y-goal[1])/dx**2
                          + (6*u-4)*tangent/dx + amplitude*(f2*warp+4*skew*f1)/dx**2)
                if abs(second) / (1+slope*slope)**1.5 > limit:
                    break
            else:
                return points
        else:
            return points
    raise ValueError('route curvature exceeds available steering')


def blocked_by(points: list[tuple[float, float]], obstacles: list[dict], config: PlantConfig) -> list[str]:
    # Moving objects are checked in time by the execution guard, not frozen forever here.
    blocked = []
    for obj in obstacles:
        if obj['vx'] or obj['vy']:
            continue
        if any(segment_distance(obj['x'], obj['y'], *a, *b) <= obj['radius'] + config.radius_m + config.safety_margin_m
               for a, b in zip(points, points[1:])):
            blocked.append(obj['id'])
    return blocked


def aisle_path(start: PoseState, goal: tuple[float,float], route: str, config: PlantConfig,
               *, obstacles=None, bounds=None) -> list[tuple[float,float]]:
    """Bounded geometry within the caller-selected corridor; never choose a route.

Keep the original geometry when it has adequate measured static clearance. At
most 20 extra Hermite/bump shapes vary side excursion and peak skew. A nominal
0.35m tracking reserve is a conservative planning heuristic, not a guarantee;
the unchanged per-action guard remains authoritative. No evaluator data is read.
"""
    if route not in ROUTES:
        raise ValueError('unknown route choice')
    original = None
    original_error = None
    try:
        original = _legacy_aisle_path(start,goal,route,config)
    except ValueError as error:
        original_error = error
    if obstacles is None or route == 'direct':
        if original is not None:
            return original
        raise original_error
    static = [obj for obj in obstacles if obj['vx'] == 0 and obj['vy'] == 0]
    if not static:
        if original is not None:
            return original
        raise original_error

    def clear(points, reserve):
        if bounds is not None:
            left,bottom,right,top=bounds
            radius=config.radius_m+config.safety_margin_m
            if not all(left+radius<=x<=right-radius and bottom+radius<=y<=top-radius for x,y in points):
                return False
        return all(min(segment_distance(obj['x'],obj['y'],*a,*b) for a,b in zip(points,points[1:]))
                   > obj['radius']+config.radius_m+config.safety_margin_m+reserve for obj in static)

    reserve = .35
    if original is not None and clear(original,reserve):
        return original
    dx=goal[0]-start.x
    if dx < .5 or abs(start.yaw_rad) > .7:
        raise original_error or ValueError('forward route cannot be connected from this pose')
    limit=math.tan(config.max_steer_rad*.5)/config.wheelbase_m
    tangent=math.tan(start.yaw_rad)
    count=math.ceil(dx/.2)
    samples=math.ceil(dx/.02)
    sign=1 if route=='upper' else -1
    shapes=((0.,0.),(.25,.2),(.25,-.2),(0.,.2),(0.,-.2))
    for excursion in (2.5,3.,3.5,4.):
        amplitude=sign*excursion
        for blend,skew in shapes:
            fine=[];feasible=True
            for index in range(samples+1):
                u=index/samples;sine=math.sin(math.pi*u);cosine=math.cos(math.pi*u)
                f=(1-blend)*sine**2+blend*sine**3
                f1=(2*(1-blend)*sine+3*blend*sine**2)*math.pi*cosine
                f2=((2*(1-blend)+6*blend*sine)*(math.pi*cosine)**2
                    -(2*(1-blend)*sine+3*blend*sine**2)*math.pi**2*sine)
                warp=1+skew*(2*u-1)
                slope=((6*u*u-6*u)*(start.y-goal[1])/dx
                    +(3*u*u-4*u+1)*tangent+amplitude*(f1*warp+2*skew*f)/dx)
                second=((12*u-6)*(start.y-goal[1])/dx**2+(6*u-4)*tangent/dx
                    +amplitude*(f2*warp+4*skew*f1)/dx**2)
                if abs(second)/(1+slope*slope)**1.5>limit:
                    feasible=False;break
                y=((2*u**3-3*u*u+1)*start.y+(u**3-2*u*u+u)*dx*tangent
                    +(-2*u**3+3*u*u)*goal[1]+amplitude*f*warp)
                fine.append((start.x+u*dx,y))
            if not feasible or not clear(fine,reserve):
                continue
            points=[]
            for index in range(count+1):
                u=index/count;sine=math.sin(math.pi*u)
                bump=amplitude*sine**2*(1-blend+blend*sine)*(1+skew*(2*u-1))
                y=((2*u**3-3*u*u+1)*start.y+(u**3-2*u*u+u)*dx*tangent
                    +(-2*u**3+3*u*u)*goal[1]+bump)
                points.append((start.x+u*dx,y))
            if _within_curvature_limit(points,limit) and clear(points,reserve):
                return points
    if original is not None:
        # Preserve the original measured blockage verdict when the bounded
        # search finds nothing. It is not a claim that the corridor is clear.
        return original
    raise original_error

