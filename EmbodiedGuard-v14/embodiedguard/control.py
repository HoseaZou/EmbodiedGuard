"""Small tracking interface over isolated, frozen legacy route/PID algorithms.

World yaw and steering are counterclockwise/left positive. The legacy route
interface uses reflected world y and yaw degrees; its PID emits right-positive
steering. This module performs both conversions. It proposes commands only:
obstacle, freshness, footprint and speed authorization belong to the executor.
"""
from __future__ import annotations

import math
from bisect import bisect_right

from .legacy.backend.carla_pid_controller import CarlaPIDConfig, CarlaTrajectoryPIDController, PIDAxisConfig
from .legacy.backend.route_control_adapter import RouteControlAdapter, RouteControlAdapterConfig
from .legacy.pipeline.global_route_runtime_adapter import GlobalRouteRuntimeAdapter
from .legacy.route.route_manager import RouteManager, RouteManagerConfig
from .legacy.route.route_types import RoutePlan, RoutePoint
from .legacy.route.waypoint_queue import WaypointQueueConfig
from .sim import PlantConfig


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return value


class LegacyTracker:
    """Forward-only Ackermann path tracking; reset starts a new frame timeline.

    The caller supplies an already feasible, ordered path with >= 2 distinct
    points. Commands require increasing non-negative frames and monotonic time.
    ``cte_m`` is unsigned route projection distance, not a safety certificate.
    Parameters target the MVP's 1 m wheelbase / 0.6 rad steering-scale vehicle;
    they are not a differential-drive controller or physical validation.
    """

    def __init__(self, points: list[tuple[float, float]], cruise_speed_mps: float = 1.2,
                 *, global_curve_profile: bool = False, plant_config: PlantConfig | None = None):
        if type(global_curve_profile) is not bool:
            raise ValueError('global_curve_profile must be a supervisor-selected bool')
        self._global_curve_profile = global_curve_profile
        self._plant_config = plant_config or PlantConfig()
        if not isinstance(self._plant_config,PlantConfig):
            raise ValueError('plant_config must be a validated PlantConfig')
        if not isinstance(points, list) or len(points) < 2:
            raise ValueError('points must contain at least two positions')
        self._points = []
        for point in points:
            if not isinstance(point, (tuple, list)) or len(point) != 2:
                raise ValueError('each point must contain x and y')
            self._points.append((_number(point[0], 'point.x'), _number(point[1], 'point.y')))
        if any(math.dist(a, b) <= 1e-6 for a, b in zip(self._points, self._points[1:])):
            raise ValueError('consecutive route points must be distinct')
        self._cruise = _number(cruise_speed_mps, 'cruise_speed_mps')
        if self._cruise <= 0:
            raise ValueError('cruise_speed_mps must be positive')
        self.reset()

    def reset(self) -> None:
        distance = 0.0
        self._arc_lengths = []
        route_points = []
        for index, (x, y) in enumerate(self._points):
            if index:
                distance += math.dist(self._points[index - 1], (x, y))
            self._arc_lengths.append(distance)
            a, b = ((x, y), self._points[index + 1]) if index + 1 < len(self._points) else (self._points[index - 1], (x, y))
            yaw = -math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            route_points.append(RoutePoint(index, x, -y, yaw_deg=yaw, cumulative_distance_m=distance))
        plan = RoutePlan('embodiedguard-route', 'synthetic-2d',
                         dict(x=route_points[0].x, y=route_points[0].y),
                         dict(x=route_points[-1].x, y=route_points[-1].y),
                         min(math.dist(a, b) for a, b in zip(self._points, self._points[1:])),
                         tuple(route_points), distance)
        # Configure the legacy interfaces for the smaller vehicle and corridors.
        config = RouteManagerConfig(
            waypoint_queue=WaypointQueueConfig(waypoint_reached_radius_m=.15,
                lookahead_base_m=.5, lookahead_speed_gain=.6,
                lookahead_min_m=.8, lookahead_max_m=2.),
            preview_spacing_m=.4, preview_count=6,
            tangent_lookahead_m=.4 if self._global_curve_profile else 2.,
            completion_distance_m=.25, completion_speed_mps=.05,
            terminal_destination_activation_m=.1)
        self._route = GlobalRouteRuntimeAdapter(RouteManager(plan, config))
        self._adapter = RouteControlAdapter(RouteControlAdapterConfig(
            goal_stop_distance_m=.25, goal_slowdown_distance_m=3.,
            goal_deceleration_mps2=.6, route_tangent_lookahead_m=2.))
        self._pid = CarlaTrajectoryPIDController(CarlaPIDConfig(
            lateral=PIDAxisConfig(.6, 0., .03, 2.) if self._global_curve_profile else PIDAxisConfig(.35, .01, .03, 2.),
            yaw=PIDAxisConfig(1.4, 0., .03, 1.) if self._global_curve_profile else PIDAxisConfig(1.4, .02, .03, 1.)))
        if self._global_curve_profile:
            # A preflight may use the existing full mission horizon. This is
            # an estimate cap, never extra execution time or deadline credit.
            self.forecast_horizon_sec = 300.
        self._last_time = None

    def reset_controller(self) -> None:
        """Clear feedback after an external stop while preserving route/time.

        Bounded execution pauses must use this method, not ``reset()``: the
        vehicle already advanced and its route projection cursor must survive.
        Subsequent observations still need increasing frames and monotonic time.
        """
        self._pid.reset()

    def command(self, *, x: float, y: float, yaw_rad: float, speed_mps: float,
                frame_id: int, sim_time_sec: float, dt_sec: float = .05) -> dict:
        x, y = _number(x, 'x'), _number(y, 'y')
        yaw = _number(yaw_rad, 'yaw_rad')
        speed = _number(speed_mps, 'speed_mps')
        now, dt = _number(sim_time_sec, 'sim_time_sec'), _number(dt_sec, 'dt_sec')
        if speed < 0 or now < 0 or not 0 < dt <= 1:
            raise ValueError('speed/time must be non-negative; dt must be in (0,1]')
        if self._last_time is not None and now < self._last_time:
            raise ValueError('sim_time_sec must be monotonic; reset for a new episode')
        context = self._route.update(frame_id=frame_id, observed_monotonic_ns=round(now * 1e9),
            ego_x=x, ego_y=-y, ego_yaw_deg=-math.degrees(yaw), speed_mps=speed)
        self._last_time = now
        remaining = max(0., context['route_length_m'] - context['route_progress_m'])
        # Slow on path remaining length without depending on legacy 99.5% goal
        # completion. The executor independently verifies distance and stopping.
        speed_target = min(self._cruise, .65 * max(remaining - .25, 0.))
        if remaining <= .33:
            speed_target = 0.
        # A stopped/resumed rover can pass the first sampled preview point
        # while the legacy projection retains its branch-continuity cursor.
        # Keep its signed CTE, tangent and progress unchanged, but hand the
        # ordinary forward-only adapter the remaining real forward preview.
        # Do not invent a waypoint, reset progress or hide an all-behind route;
        # terminal and deviation recovery keep the legacy capture contract.
        conditions = context['conditions']
        if (remaining > self._adapter.config.terminal_capture_remaining_route_m
                and not conditions['route_deviated']
                and not conditions['heading_deviated']):
            preview = context['ego_local_waypoints']
            first_forward = next((index for index, point in enumerate(preview)
                                  if point['x_forward_m'] > 0.), None)
            if first_forward is not None and first_forward > 0:
                context = dict(context, ego_local_waypoints=preview[first_forward:])
        # There is no learned trajectory in the MVP. Supply the measured route
        # errors as the deterministic candidate instead of pretending a model
        # requested straight-ahead motion throughout a bend.
        evidence = self._adapter.adapt(model_control_hint=dict(
            target_speed_mps=speed_target,
            target_lateral=context['signed_distance_to_route_m'],
            target_yaw=context['route_tangent_yaw_rad']),
            route_context=context, ordinary_route_follow=True)
        hint = evidence['effective_control_hint']
        if self._global_curve_profile:
            # Curvature is deterministic route geometry, not a learned action.
            # Encode the Ackermann feedforward in the existing yaw PID input;
            # its unchanged steering cap, slew limit and reset still apply.
            # The legacy context curvature is an UNSIGNED speed indicator.
            # Derive signed curvature from the original world-frame polyline
            # at the measured route-progress cursor, never from that magnitude.
            index = min(max(bisect_right(self._arc_lengths,context['route_progress_m'])-1,0),
                        len(self._points)-3)
            if len(self._points) >= 3:
                a,b,c = self._points[index:index+3]
                denominator = math.dist(a,b)*math.dist(b,c)*math.dist(a,c)
                curvature = 2*((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))/denominator if denominator>1e-12 else 0.
            else:
                curvature = 0.
            heading = context['route_tangent_yaw_rad']
            feedforward = math.atan(curvature*self._plant_config.wheelbase_m) / self._plant_config.max_steer_rad
            hint = dict(hint, target_lateral=context['signed_distance_to_route_m'],
                        target_yaw=max(-math.pi,min(math.pi,heading+feedforward/1.4)))
            # Abrupt arc/connector curvature transitions require time for the
            # preserved PID's steering slew limit. Reduce the speed reference
            # before the transition; do not raise steering authority or mask a
            # safety rejection. This uses reference geometry, never future pose.
            upcoming = bisect_right(self._arc_lengths,context['route_progress_m']+1.2)
            for future in range(max(0,index-2),min(upcoming,len(self._points)-2)):
                aa,bb,cc = self._points[future:future+3]
                denom=math.dist(aa,bb)*math.dist(bb,cc)*math.dist(aa,cc)
                kk=2*((bb[0]-aa[0])*(cc[1]-bb[1])-(bb[1]-aa[1])*(cc[0]-bb[0]))/denom if denom>1e-12 else 0.
                if abs(kk-curvature) > .12:
                    hint['target_speed_mps'] = min(hint['target_speed_mps'],.65)
                    break
        # This flag only enables candidate generation; the external executor
        # must authorize the actual final action after its own safety checks.
        control = self._pid.command(hint, current_speed_mps=speed,
                                    safety={'safe': True}, dt_sec=dt)
        return dict(throttle=control.throttle, brake=control.brake,
                    steer_left_normalized=-control.steer,
                    cte_m=context['distance_to_route_m'],
                    distance_to_goal_m=context['distance_to_goal_m'],
                    remaining_m=remaining, target_speed_mps=hint['target_speed_mps'])
