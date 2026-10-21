"""Known public-map fixture around the unchanged v13 deterministic executor.

One MovingAI start->goal, labelled exit_c solely to reuse the frozen tool API.
No learned perception or temperature labels. Faults and physics are synthetic.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict
import math

import numpy as np

from pathlib import Path
import hashlib
DATA = Path(__file__).resolve().parents[2] / 'sources/movingai-public-maps-20261013'
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
def scenario_record(map_name, file_line):
    if map_name not in ('16room_000', '32room_000', 'maze512-16-0'):
        raise ValueError('Unregistered public map')
    grid = parse_map((DATA / (map_name + '.map')).read_text(encoding='ascii'), map_name)
    rows = parse_scenarios((DATA / (map_name + '.scen')).read_text(encoding='ascii'), grid)
    return next(row for row in rows if row['file_line'] == file_line)
from .map_geometry import RigidFrame, audit_polyline, endpoint_clearance, parse_map, parse_scenarios
from .execution import Mission
from .sim import AckermannPlant, PlantConfig, PoseState, STOP, integrate
from .state import ObservationError, ObservationStore
from .global_planning import GlobalPlanner, PlannerConfig
from .control import LegacyTracker
from .timing import forecast
from .route_validity import static_obstacle_signature


FAULTS = ("none", "freeze_once", "persistent_freeze")


def checked_grid_path(grid, points_world, scale, footprint):
    points = np.asarray(points_world, dtype=float)
    # The original square checker rejects <=1e-9m adjacent vertices. Braking
    # can legitimately produce such increments. Merge only within a fixed
    # anchor tolerance, and inflate the checked footprint by the same bound.
    # Thus tiny motion remains covered; no unsafe segment is silently dropped.
    compact = [points[0]]
    error_bound = 0.
    for point in points[1:]:
        distance = float(np.linalg.norm(point-compact[-1]))
        if distance > 1.01e-9:
            compact.append(point)
        elif distance > 0:
            error_bound = 1.01e-9
    points = np.asarray(compact)
    footprint += error_bound
    if len(points) == 1:
        result = endpoint_clearance(grid, points[0], scale, footprint)
        return {"safe": result["valid"], "minimum_wall_clearance_m": result["wall_clearance_m"],
                "minimum_boundary_clearance_m": result["boundary_clearance_m"], "stationary": True}
    return audit_polyline(grid, points, scale, footprint)


class PublicMapWorld:
    def __init__(self, map_name, file_line, scale, fault="none"):
        if fault not in FAULTS or scale not in (.5, 1.):
            raise ValueError("Unregistered public-map scale/fault")
        self.row = scenario_record(map_name, file_line)
        self.map_name, self.scale, self.fault = map_name, scale, fault
        self.grid = parse_map((DATA / (map_name + ".map")).read_text(encoding="ascii"), map_name)
        self.map_sha256 = sha(DATA / (map_name + ".map"))
        self.config = PlantConfig()
        footprint = self.config.radius_m + self.config.safety_margin_m
        start = self.grid.cell_center(self.row["start_cell"], scale)
        goal = self.grid.cell_center(self.row["goal_cell"], scale)
        self.endpoint_checks = {"start": endpoint_clearance(self.grid, start, scale, footprint),
                                "goal": endpoint_clearance(self.grid, goal, scale, footprint)}
        if not all(packet["valid"] for packet in self.endpoint_checks.values()):
            raise ValueError("PUBLIC_INPUT_FOOTPRINT_REJECTED")
        self.frame = RigidFrame.from_pair(start, goal)
        self.centers_local = self.frame.to_local(self.grid.centers(scale))
        radius = math.sqrt(2) * scale / 2
        self.circles = [{"id": f"cell-{int(col)}-{int(row)}", "x": float(x), "y": float(y),
                         "radius": radius, "vx": 0., "vy": 0.}
                        for (col, row), (x, y) in zip(self.grid.blocked_cells, self.centers_local)]
        width, height = self.grid.width * scale, self.grid.height * scale
        corners = self.frame.to_local(((0., 0.), (width, 0.), (0., height), (width, height)))
        lo, hi = corners.min(axis=0), corners.max(axis=0)
        self.bounds = (float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1]))
        self.checkpoints = {"exit_c": (self.frame.distance, 0.)}
        self.plant = AckermannPlant(self.config)
        self.scenario = "public_map_" + fault
        self.seed = file_line
        self.episode_id = f"public-{map_name}-{file_line}-{scale:g}-{fault}"
        self.frame_id, self.time_sec = 0, 0.
        self._frozen_packet = None
        self._fault_injected = False
        self._refreshed = False
        self.refresh_attempts = 0

    def obstacles_at(self, time_sec=None):
        # Whole static map is known. This local spatial broadphase covers every
        # action+braking horizon at <=1.5m/s and is only for collision evaluation.
        state = self.plant.state
        distance_sq = ((self.centers_local[:, 0] - state.x)**2
                       + (self.centers_local[:, 1] - state.y)**2)
        return [self.circles[int(i)] for i in np.flatnonzero(distance_sq <= 16.)]

    def _known_map_packet_obstacles(self):
        state = self.plant.state
        goal_x = self.frame.distance
        dx = goal_x - state.x
        radius = math.sqrt(2) * self.scale / 2
        margin = radius + self.config.radius_m + self.config.safety_margin_m + .35 + 1e-9
        # For all v13 Hermite/bump candidates from the current measured pose:
        # |u(1-u)^2| <=4/27, blend<=1, warp<=1.2 and excursion<=4m.
        tangent_bound = abs(dx * math.tan(state.yaw_rad)) * (4 / 27) if abs(state.yaw_rad) <= .7 else abs(dx) + abs(state.y)
        lower = min(state.y, 0.) - tangent_bound - 4.8 - margin
        upper = max(state.y, 0.) + tangent_bound + 4.8 + margin
        centers = self.centers_local
        keep = ((centers[:, 0] >= min(state.x, goal_x) - margin)
                & (centers[:, 0] <= max(state.x, goal_x) + margin)
                & (centers[:, 1] >= lower) & (centers[:, 1] <= upper))
        # Also keep every nearby occupied cell for off-reference tracking and
        # braking. This is deterministic from the same-frame pose; two reads
        # of the same frame never silently change obstacle contents.
        keep |= (centers[:, 0] - state.x)**2 + (centers[:, 1] - state.y)**2 <= 16.
        return [self.circles[int(i)] for i in np.flatnonzero(keep)]

    def packet(self):
        state = self.plant.state
        packet = {"episode_id": self.episode_id, "frame_id": self.frame_id,
                  "observed_at": self.time_sec, "pose": {"x": state.x, "y": state.y, "yaw_rad": state.yaw_rad},
                  "speed_mps": state.speed_mps, "obstacles_valid": True,
                  "obstacles": self._known_map_packet_obstacles(), "source": "simulated_sensor"}
        if self.fault != "none" and state.x >= 4. and not self._fault_injected:
            self._frozen_packet = deepcopy(packet)
            self._fault_injected = True
        if self._frozen_packet is not None and not self._refreshed:
            return deepcopy(self._frozen_packet)
        return packet

    def refresh_sensor(self):
        self.refresh_attempts += 1
        if self.fault != "persistent_freeze":
            self._refreshed = True

    def step(self, action, dt_sec=None):
        previous = self.plant.state
        dt = self.config.dt_sec if dt_sec is None else dt_sec
        self.plant.step(action, dt)
        self.time_sec += dt
        self.frame_id += 1
        return previous, self.plant.state

    def metadata(self):
        return {"map": self.map_name, "map_sha256": self.map_sha256, "scenario_file_line": self.row["file_line"],
                "public_start_cell": self.row["start_cell"], "public_goal_cell": self.row["goal_cell"],
                "scale_m_per_cell": self.scale, "metric_scale_is_assumed": True,
                "initial_heading_is_assumed": True, "task_kind": "single_public_goal_arrival_and_stop",
                "frame_origin_world_m": self.frame.origin.tolist(), "frame_cos": self.frame.cos, "frame_sin": self.frame.sin,
                "distance_m": self.frame.distance, "known_static_map_fixture": True,
                "map_is_not_real_sensor_perception": True, "synthetic_fault": self.fault,
                "fault_trigger_forward_x_m": 4., "semantic_inspection_claimed": False,
                "source": "Moving AI Lab 2D Grid Benchmarks", "license": "ODC-By v1.0"}


class PublicMapMission(Mission):
    task_instructions = ("Known public artificial map with an assumed metric scale and heading. "
                         "Only exit_c is required: reach the official scenario goal and stop. "
                         "Arrival evidence is not temperature or semantic inspection. No real hardware/perception claim.")

    def __init__(self, map_name, file_line, scale, fault="none", *, decision_source="public_fixed_client"):
        world = PublicMapWorld(map_name, file_line, scale, fault)
        super().__init__("normal", seed=file_line, decision_source=decision_source)
        self._world = world
        self._store = ObservationStore(world.episode_id)
        self._events = []
        self._map_guard_checks = 0
        self._map_guard_rejections = 0
        self._independent_metrics = {"grid_collisions": 0, "grid_boundary_violations": 0,
                                     "grid_safety_margin_violations": 0}
        self._permitted_routes = ('direct', 'upper', 'lower', 'global')
        self._navigation_window_limit = 2
        self._global_planner = GlobalPlanner(world.grid, world.scale, world.config)
        self._global_diagnostics = []
        self._log("public_map_mission_started", plant_config=asdict(world.config),
                  metadata=world.metadata(), original_modules_modified=False, model_call_verified=False)

    def task_status(self):
        progress = self._task_progress()
        return {"revision": 1, "details_revision": 1, "required_targets": ["exit_c"],
                "excluded_targets": ["panel_a", "pump_b"], "permitted_routes": list(self._permitted_routes),
                "next_target": progress["next_target"], "completed_targets": progress["completed_targets"],
                "brief": "仅到达公开场景终点 exit_c 并停稳；没有温度或语义巡检任务。", "source": "public_map_supervisor"}

    def observe(self):
        result = super().observe()
        result['route_choices'] = list(self._permitted_routes)
        result['navigation_capabilities'] = {
            'global': 'bounded forward curvature-constrained search on the supervisor known static map',
            'map_sha256': self._world.map_sha256,
            'kinematics': 'ackermann_forward_only', 'reverse_supported': False,
            'bounded_search_failure_is_not_global_impossibility': True,
            'navigation_window_limit': self._navigation_window_limit,
            'window_seconds': 15, 'failure_returns_immediately': True,
        }
        return result

    def recovery_context(self):
        result = super().recovery_context()
        tried = {row['route'] for row in result['assessed_routes']}
        result['untried_routes'] = [route for route in self._permitted_routes
                                   if result['next_target'] is not None and route not in tried]
        result['permitted_routes'] = list(self._permitted_routes)
        result['task_revision'] = 1
        if self._global_diagnostics:
            result['last_global_search'] = deepcopy(self._global_diagnostics[-1])
        return result

    def plan_route(self, *, target, route='direct', rationale=''):
        if route != 'global':
            return super().plan_route(target=target, route=route, rationale=rationale)
        def finish(result, observation=None, points=None):
            return self._record_plan(target, route, rationale, result,
                                     observation=observation, points=points)
        unavailable = self._available()
        if unavailable:
            return finish(self._failure(unavailable))
        self._runtime_started = True
        if target != self._task_progress()['next_target']:
            return finish(self._failure('TARGET_OUT_OF_ORDER'))
        try:
            observation = self._observe()
        except ObservationError as exc:
            return finish(self._failure(exc.code))
        if observation['frame_id'] != self._world.frame_id:
            return finish(self._failure('STALE_OBSERVATION'), observation)
        if observation['speed_mps'] > .05:
            return finish(self._failure('NOT_STATIONARY'), observation)
        state = PoseState(**observation['pose'], speed_mps=observation['speed_mps'])
        goal = self._world.checkpoints[target]
        if math.dist((state.x,state.y),goal) <= .6:
            return finish(dict(ok=True,status='ARRIVED',target=target,route_id=None,
                               state=self._feedback_state()), observation)
        world_xy = self._world.frame.to_world(((state.x,state.y),))[0]
        heading = state.yaw_rad + math.atan2(self._world.frame.sin,self._world.frame.cos)
        start_world = PoseState(float(world_xy[0]),float(world_xy[1]),heading,state.speed_mps)
        goal_world = tuple(map(float,self._world.frame.to_world((goal,))[0]))
        result = self._global_planner.plan(start_world, goal_world)
        diagnostics = dict(result.diagnostics, source_observation_frame=observation['frame_id'],
                           map_sha256=self._world.map_sha256, reason=result.reason)
        self._global_diagnostics.append(diagnostics)
        self._log('global_search_finished', diagnostics=diagnostics)
        if not result.ok:
            # Failure categories are explicit; a finite search is never a proof
            # that no vehicle trajectory exists anywhere in the environment.
            return finish(self._failure('GLOBAL_SEARCH_FAILED', search_reason=result.reason,
                                       global_search=diagnostics), observation)
        points = [tuple(map(float,p)) for p in self._world.frame.to_local(result.points)]
        audit = checked_grid_path(self._world.grid,result.points,self._world.scale,
                                  self._world.config.radius_m+self._world.config.safety_margin_m)
        if not audit['safe']:
            return finish(self._failure('GLOBAL_PATH_AUDIT_REJECTED',global_search=diagnostics),observation,points)
        self._route_sequence += 1
        route_id = f'route-{self._route_sequence}'
        self._routes[route_id] = dict(target=target,route=route,points=points,
                                     tracker=LegacyTracker(points,cruise_speed_mps=1.2,global_curve_profile=True,
                                                           plant_config=self._world.config),
                                     static_signature=static_obstacle_signature(observation['obstacles']))
        self._active_route = route_id
        estimate = forecast(points,state,goal,self._world.config,
                            frame_id=observation['frame_id'],sim_time_sec=self._world.time_sec,
                            route_tracker=self._routes[route_id]['tracker'])
        return finish(dict(ok=True,status='PLANNED',target=target,route=route,route_id=route_id,
                           distance_m=sum(math.dist(a,b) for a,b in zip(points,points[1:])),
                           max_speed_mps=self._world.config.max_speed_mps,execution_estimate=estimate,
                           observation_frame=observation['frame_id'],global_search=diagnostics),observation,points)

    def _map_action_verdict(self, state, action):
        cfg = self._world.config
        positions = [(state.x, state.y)]
        predicted = state
        maximum = math.ceil((cfg.max_speed_mps / cfg.braking_mps2 + .2) / cfg.dt_sec) + 2
        for index in range(maximum):
            predicted = integrate(predicted, action if index == 0 else STOP, cfg.dt_sec, cfg)
            positions.append((predicted.x, predicted.y))
            if predicted.speed_mps == 0.:
                break
        else:
            return {"safe": False, "reason": "INDEPENDENT_STOP_HORIZON_EXCEEDED"}
        return checked_grid_path(self._world.grid, self._world.frame.to_world(positions),
                                 self._world.scale, cfg.radius_m + cfg.safety_margin_m)

    def _remaining_static_blockers(self, plan, pose, obstacles):
        if plan['route'] != 'global':
            return super()._remaining_static_blockers(plan,pose,obstacles)
        # Global paths may turn and return in x. Their direction is arc length,
        # not sorted x. The cursor comes solely from previous measured tracking
        # updates; no full-truth pose, forward-x trim, or nearest distant branch.
        context = plan['tracker']._route._last_context
        progress = context['route_progress_m'] if context is not None else 0.
        points = np.asarray(plan['points'],dtype=float)
        cumulative = np.concatenate(([0.],np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))))
        index = min(max(int(np.searchsorted(cumulative,progress,side='right'))-1,0),len(points)-2)
        remaining = np.vstack(((pose.x,pose.y),points[index+1:]))
        a,b = remaining[:-1],remaining[1:]
        delta=b-a
        denom=np.sum(delta*delta,axis=1)
        keep=denom>1e-18;a=a[keep];delta=delta[keep];denom=denom[keep]
        footprint=self._world.config.radius_m+self._world.config.safety_margin_m
        blockers=[]
        for obj in obstacles:
            if obj['vx'] or obj['vy']:
                continue
            point=np.array((obj['x'],obj['y']))
            t=np.clip(np.sum((point-a)*delta,axis=1)/denom,0.,1.)
            if np.any(np.sum((point-(a+t[:,None]*delta))**2,axis=1) <= (obj['radius']+footprint)**2):
                blockers.append(obj['id'])
        return blockers

    def _advance(self, action, **kwargs):
        cfg = self._world.config
        if action != STOP:
            # Never authorize non-emergency movement with evaluator truth if
            # pose input is stale. The original executor already checks this;
            # the independent map layer repeats the actual-frame requirement.
            observation = self._store.current(now=self._world.time_sec)
            if observation["frame_id"] != self._world.frame_id:
                raise ObservationError("STALE_OBSERVATION", "Independent map guard requires current measured pose")
            measured = PoseState(**observation["pose"], speed_mps=observation["speed_mps"])
            audit = self._map_action_verdict(measured, action)
            self._map_guard_checks += 1
            if not audit["safe"]:
                self._map_guard_rejections += 1
                self._log("public_square_action_rejected", action=asdict(action), audit=audit)
                raise ValueError("PUBLIC_SQUARE_MAP_ACTION_OR_BRAKING_REJECTED")
        previous = self._world.plant.state
        super()._advance(action, **kwargs)
        current = self._world.plant.state
        points = self._world.frame.to_world(((previous.x, previous.y), (current.x, current.y)))
        physical = checked_grid_path(self._world.grid, points, self._world.scale, cfg.radius_m)
        margin = checked_grid_path(self._world.grid, points, self._world.scale, cfg.radius_m + cfg.safety_margin_m)
        self._independent_metrics["grid_collisions"] += int(physical["minimum_wall_clearance_m"] < -1e-9)
        self._independent_metrics["grid_boundary_violations"] += int(physical["minimum_boundary_clearance_m"] < -1e-9)
        self._independent_metrics["grid_safety_margin_violations"] += int(not margin["safe"])
        self._trace[-1]["independent_grid_audit"] = {"physical": physical, "safety_margin": margin}
        if not physical["safe"] or not margin["safe"]:
            self._terminated = True
            self._termination_reason = "PUBLIC_GRID_SAFETY_VIOLATION"
            self._log("public_grid_actual_violation", physical=physical, margin=margin)

    def inspect(self, *, target):
        unavailable = self._available()
        if unavailable:
            return self._failure(unavailable)
        if target != "exit_c":
            return self._failure("INVALID_TARGET")
        try:
            observation = self._observe()
        except ObservationError as error:
            return self._failure(error.code)
        if observation["frame_id"] != self._world.frame_id:
            return self._failure("STALE_OBSERVATION")
        pose = PoseState(**observation["pose"], speed_mps=observation["speed_mps"])
        distance = math.dist((pose.x, pose.y), self._world.checkpoints[target])
        if pose.speed_mps > .05 or distance > .6:
            return self._failure("INSPECTION_POSE_NOT_REACHED")
        audit = checked_grid_path(self._world.grid, self._world.frame.to_world(((pose.x, pose.y),)),
                                 self._world.scale, self._world.config.radius_m + self._world.config.safety_margin_m)
        if not audit["safe"]:
            return self._failure("PUBLIC_GOAL_CLEARANCE_REJECTED")
        evidence = {"target": target, "source": "synthetic_public_map_arrival_checker",
                    "frame_id": observation["frame_id"], "observed_at": observation["observed_at"],
                    "pose": self._feedback_state(), "distance_to_goal_m": distance,
                    "arrival_and_stop_only": True, "semantic_inspection_claimed": False,
                    "detection_accuracy_claimed": False, "independent_grid_audit": audit}
        self._inspections[target] = evidence
        self._inspection_event_ids[target] = self._log("inspection_verified", evidence=evidence)
        self._record_completion_receipt()
        return {"ok": True, "evidence": deepcopy(evidence), **self._task_progress(),
                "budget": self._budget_status(), "completion_receipt": deepcopy(self._completion_receipt)}

    def _evaluate_verdict(self):
        verdict = super()._evaluate_verdict()
        independent_safe = not any(self._independent_metrics.values())
        verdict["task_success"] = verdict["task_success"] and independent_safe
        verdict["safe_termination"] = verdict["safe_termination"] and independent_safe
        verdict["public_map"] = self._world.metadata()
        verdict["public_grid_metrics"] = {**self._independent_metrics, "additional_map_guard_checks": self._map_guard_checks,
                                          "additional_map_guard_rejections": self._map_guard_rejections}
        verdict["benchmark_scope"] = "public_artificial_static_map_single_goal_synthetic_ackermann_motion"
        verdict['global_searches'] = deepcopy(self._global_diagnostics)
        return verdict

    def evidence(self):
        evidence = super().evidence()
        evidence["public_map_adapter"] = self._world.metadata()
        return evidence
