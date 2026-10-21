"""One bounded execution interface owns state, actuator access and verification."""
from __future__ import annotations

from copy import copy, deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

from .control import LegacyTracker
from .planning import ROUTES, aisle_path, blocked_by
from .safety import assess_action, inside_bounds, segment_distance, swept_clearance
from .sim import Action, InspectionWorld, PoseState, STOP, finite_number
from .state import ObservationError, ObservationStore
from .timing import forecast
from .route_validity import static_obstacle_signature, observed_remaining_blockers


class Mission:
    """All motion is bounded and ends at rest before returning to the caller.

    Tools cannot supply coordinates, raw actuators, observation facts or success.
    Safety policy is fixed by the process, never an argument from the Agent.
    """
    def __init__(self, scenario='normal', *, seed=0, decision_source='external_mcp_client',
                 max_sim_seconds=300., max_operations=80):
        max_sim_seconds = finite_number(max_sim_seconds, 'max_sim_seconds')
        if max_sim_seconds <= 0 or type(max_operations) is not int or max_operations < 1:
            raise ValueError('mission budgets must be positive; operation budget must be an integer')
        self._world = InspectionWorld(scenario, seed, f'{scenario}-{seed}')
        self._store = ObservationStore(self._world.episode_id)
        self._decision_source = decision_source
        self._max_sim_seconds = max_sim_seconds
        self._max_operations = max_operations
        self._operations = {}
        self._operation_routes = {}
        self._runtime_started = False
        self._routes = {}
        self._plans = []
        self._route_sequence = 0
        self._active_route = None
        self._inspections = {}
        self._inspection_event_ids = {}
        self._completion_receipt = None
        self._terminated = False
        self._termination_reason = None
        self._events = []
        self._trace = []
        self._metrics = dict(collisions=0, boundary_violations=0, speed_violations=0,
                             path_length_m=0., safety_interventions=0, executions=0)
        self._ctes = []
        self._log('mission_started', scenario=scenario, seed=seed, decision_source=decision_source,
                  plant_config=asdict(self._world.plant.config), environment='2d_kinematic_simulation',
                  collision_model='swept_conservative_circles_constant_velocity', model_call_verified=False)

    def _log(self, event, **data):
        self._events.append(dict(**{**deepcopy(data), 'event_id': len(self._events)+1, 'event': event,
                                    'sim_time_sec': self._world.time_sec, 'frame_id': self._world.frame_id}))
        return self._events[-1]['event_id']

    def _task_progress(self):
        remaining = [target for target in self._world.checkpoints if target not in self._inspections]
        return dict(completed_targets=list(self._inspections), remaining_targets=remaining,
                    next_target=remaining[0] if remaining else None)

    def _budget_status(self):
        """Read-only mission counters, not a sensor refresh or a travel-time estimate.

        A direct Mission has no automatic wall clock. Its supervisor may explicitly
        advance idle time; MCP adds its actual clock policy to the tool envelope.
        """
        return dict(schema_version=1, runtime_started=self._runtime_started,
                    sim_time_sec=self._world.time_sec, deadline_sim_sec=self._max_sim_seconds,
                    remaining_sim_sec=max(0., self._max_sim_seconds - self._world.time_sec),
                    completed_execution_operations=len(self._operations),
                    max_execution_operations=self._max_operations,
                    remaining_execution_operations=max(0, self._max_operations - len(self._operations)),
                    clock_policy='simulation_steps_plus_explicit_supervisor_idle',
                    snapshot_phase='mission_return')

    def recovery_context(self):
        """Read only observed state and actual planning history; never resense."""
        target = self._task_progress()['next_target']
        assessed = [plan for plan in self._plans if plan['target'] == target and
            (plan['status'] in ('PLANNED', 'ARRIVED') or
             plan['reason'] in ('PATH_BLOCKED', 'NO_FEASIBLE_CONNECTION'))]
        history = [dict(route=plan['route'], plan_id=plan['plan_id'], status=plan['status'],
            reason=plan['reason'], observation_frame=plan['observation_frame'],
            sim_time_sec=plan['sim_time_sec']) for plan in assessed]
        known_routes = {item['route'] for item in history}
        observation = dict(valid=False, reason='UNKNOWN_OBSERVATION', frame_id=None,
            age_sec=None, moving_obstacles_present=None)
        try:
            packet = copy(self._store).current(now=self._world.time_sec)
        except ObservationError as exc:
            observation['reason'] = exc.code
        else:
            valid = packet['frame_id'] == self._world.frame_id
            observation = dict(valid=valid, reason=None if valid else 'STALE_OBSERVATION',
                frame_id=packet['frame_id'], age_sec=packet['age_sec'],
                moving_obstacles_present=(any(ob['vx'] != 0. or ob['vy'] != 0.
                    for ob in packet['obstacles']) if valid else None))
        return deepcopy(dict(next_target=target, assessed_routes=history,
            untried_routes=[route for route in ROUTES if target is not None and route not in known_routes],
            current_observation=observation, observation_refreshed=False,
            route_history_scope='current target in this Mission; historical planning snapshots, not current feasibility or global completeness',
            source='actual_plans_and_stored_sensor_packets'))

    def _record_plan(self, target, route, rationale, result, *, observation=None, points=None):
        """Immutable spatial plan evidence; no invented arrival times or sensor poses."""
        plan_id = f'plan-{len(self._plans) + 1}'
        route_id = result.get('route_id')
        record = dict(schema_version=1, plan_id=plan_id, route_id=route_id,
                      target=target, route=route, parameterization='spatial', coordinate_frame='world',
                      units=dict(position='m', angle='rad', time='s'), axes='x/y world, yaw counterclockwise',
                      sim_time_sec=self._world.time_sec,
                      observation_frame=observation['frame_id'] if observation else None,
                      source_observation={k: observation[k] for k in
                          ('episode_id', 'frame_id', 'observed_at', 'received_at', 'source')} if observation else None,
                      anchor_pose=deepcopy(observation['pose']) if observation else None,
                      points_xy=[list(point) for point in points] if points is not None else None,
                      status=result['status'] if result.get('ok') else 'REJECTED',
                      reason=result.get('reason'), blocked_by=list(result.get('blocked_by', [])),
                      rationale=str(rationale)[:1000], constraints=asdict(self._world.plant.config))
        record['event_id'] = self._log('planning_decision', plan_id=plan_id, route_id=route_id,
                    target=target, route=route, rationale=record['rationale'], status=record['status'],
                    reason=record['reason'], blocked_by=record['blocked_by'],
                    observation_frame=record['observation_frame'])
        self._plans.append(record)
        if route_id in self._routes:
            self._routes[route_id]['plan_id'] = plan_id
        return dict(result, plan_id=plan_id, **self._task_progress(), budget=self._budget_status())

    def _state(self):
        # Evaluator-only truth. Never return this as a sensor observation.
        return asdict(self._world.plant.state)

    def _feedback_state(self, *, readonly=False):
        # current() advances its reader clock. A shallow store copy isolates that
        # clock for pure evaluation; current() itself only reads stored packets.
        store = copy(self._store) if readonly else self._store
        try:
            observation = store.current(now=self._world.time_sec)
        except ObservationError:
            return None
        if observation['frame_id'] != self._world.frame_id:
            return None
        return dict(**observation['pose'], speed_mps=observation['speed_mps'],
                    frame_id=observation['frame_id'], observed_at=observation['observed_at'],
                    source=observation['source'])

    def _stopping_verified(self):
        return self._world.plant.state.speed_mps == 0.

    def _failure(self, reason, **data):
        return dict(ok=False, status='STOPPED', reason=reason, state=self._feedback_state(),
                    stopping_verified=self._stopping_verified(),
                    verification_source='simulator_evaluator', budget=self._budget_status(), **data)

    def _observe(self):
        self._store.ingest(self._world.packet(), received_at=self._world.time_sec)
        return self._store.current(now=self._world.time_sec)

    def observe(self):
        try:
            observation = self._observe()
        except ObservationError as exc:
            return self._failure(exc.code)
        return dict(ok=True, observation=observation, completed_targets=list(self._inspections),
                    remaining_targets=[x for x in self._world.checkpoints if x not in self._inspections],
                    route_choices=list(ROUTES), terminated=self._terminated, budget=self._budget_status())

    def _advance(self, action: Action, *, operation_id=None, cte=None, dt_sec=None,
                 control_feedback=None, safety_assessment=None):
        dt_sec = self._world.plant.config.dt_sec if dt_sec is None else dt_sec
        objects = self._world.obstacles_at()
        previous, current = self._world.step(action, dt_sec=dt_sec)
        cfg = self._world.plant.config
        clearance = swept_clearance(previous, current, objects, 0., dt_sec, cfg.radius_m)
        collision = clearance < 0
        boundary = not inside_bounds(current, self._world.bounds, cfg.radius_m)
        speeding = current.speed_mps > cfg.max_speed_mps + 1e-9
        self._metrics['collisions'] += int(collision)
        self._metrics['boundary_violations'] += int(boundary)
        self._metrics['speed_violations'] += int(speeding)
        self._metrics['path_length_m'] += math.dist((previous.x, previous.y), (current.x, current.y))
        if cte is not None:
            self._ctes.append(cte)
        route_id = self._operation_routes.get(operation_id)
        plan_id = self._routes.get(route_id, {}).get('plan_id')
        self._trace.append(dict(sim_time_sec=self._world.time_sec, frame_id=self._world.frame_id,
                                state=self._state(), action=asdict(action), operation_id=operation_id,
                                route_id=route_id, plan_id=plan_id,
                                control_feedback=deepcopy(control_feedback),
                                safety_assessment=deepcopy(safety_assessment),
                                swept_clearance_m=clearance if math.isfinite(clearance) else None,
                                geometric_cte_m=cte, collisions=int(collision),
                                obstacles=self._world.obstacles_at(), ground_truth_for_evaluation=True))
        if collision or boundary or speeding:
            self._terminated = True
            self._termination_reason = 'SAFETY_VIOLATION'
            self._log('safety_violation', collision=collision, boundary=boundary, speeding=speeding)

    def _brake(self, operation_id=None):
        # Does not require perception: loss of state must not disable emergency stop.
        for _ in range(100):
            if self._world.plant.state.speed_mps == 0.:
                return
            self._advance(STOP, operation_id=operation_id)
        self._terminated = True
        self._termination_reason = 'STOP_FAILED'

    def _available(self):
        if self._terminated:
            return 'TERMINATED'
        if self._world.time_sec >= self._max_sim_seconds:
            self.stop(terminate=True, reason='TIME_BUDGET_EXHAUSTED')
            return 'TIME_BUDGET_EXHAUSTED'
        return None

    def plan_route(self, *, target: str, route: str = 'direct', rationale: str = ''):
        def finish(result, *, observation=None, points=None):
            return self._record_plan(target, route, rationale, result, observation=observation, points=points)

        unavailable = self._available()
        if unavailable:
            return finish(self._failure(unavailable))
        self._runtime_started = True
        remaining = [x for x in self._world.checkpoints if x not in self._inspections]
        if not remaining or target != remaining[0]:
            return finish(self._failure('TARGET_OUT_OF_ORDER', remaining_targets=remaining))
        if route not in ROUTES:
            return finish(self._failure('INVALID_ROUTE'))
        try:
            observation = self._observe()
        except ObservationError as exc:
            return finish(self._failure(exc.code))
        if observation['speed_mps'] > .05:
            return finish(self._failure('NOT_STATIONARY'), observation=observation)
        state = PoseState(**observation['pose'], speed_mps=observation['speed_mps'])
        goal = self._world.checkpoints[target]
        if math.dist((state.x, state.y), goal) <= .6:
            return finish(dict(ok=True, status='ARRIVED', target=target, route_id=None,
                               state=self._feedback_state()), observation=observation)
        try:
            points = aisle_path(state, goal, route, self._world.plant.config,
                obstacles=observation['obstacles'],bounds=self._world.bounds)
        except ValueError as exc:
            return finish(self._failure('NO_FEASIBLE_CONNECTION', detail=str(exc)), observation=observation)
        obstacles = blocked_by(points, observation['obstacles'], self._world.plant.config)
        if obstacles:
            return finish(self._failure('PATH_BLOCKED', blocked_by=obstacles,
                          route_choices=list(ROUTES), target=target), observation=observation, points=points)
        self._route_sequence += 1
        route_id = f'route-{self._route_sequence}'
        self._routes[route_id] = dict(target=target, route=route, points=points,
                                     tracker=LegacyTracker(points, cruise_speed_mps=1.2),
                                     static_signature=static_obstacle_signature(observation['obstacles']))
        self._active_route = route_id
        estimate = forecast(points, state, goal, self._world.plant.config,
                            frame_id=observation['frame_id'], sim_time_sec=self._world.time_sec)
        return finish(dict(ok=True, status='PLANNED', route_id=route_id, target=target, route=route,
                    distance_m=sum(math.dist(a, b) for a, b in zip(points, points[1:])),
                    max_speed_mps=1.5, execution_estimate=estimate,
                    observation_frame=observation['frame_id']), observation=observation, points=points)

    def elapse_idle_wall(self, seconds: float):
        """Supervisor-only clock bridge, deliberately absent from Agent tools.

        After the first planning attempt, every wall-clock second spent waiting
        for a decision advances this world under STOP. Partial final steps retain
        sub-tick time and long waits consume the entire remaining mission budget.
        The bridge does not fabricate a fresh observation or refresh a sensor.
        """
        seconds = finite_number(seconds, 'idle wall seconds')
        if seconds < 0:
            raise ValueError('idle wall seconds must be nonnegative')
        if not self._runtime_started or seconds == 0:
            return dict(ok=True, advanced_sec=0., reason='NO_ACTIVE_IDLE_INTERVAL', budget=self._budget_status())
        start = self._world.time_sec
        unavailable = self._available()
        if unavailable:
            return dict(ok=False, advanced_sec=0., reason=unavailable, budget=self._budget_status())
        self._brake()
        # Normally every tool has already stopped. Include any emergency-brake
        # time in the requested interval rather than silently charging it twice.
        end = min(start + seconds, self._max_sim_seconds)
        while self._world.time_sec < end:
            unavailable = self._available()
            if unavailable:
                break
            dt = min(self._world.plant.config.dt_sec, end - self._world.time_sec)
            self._advance(STOP, dt_sec=dt)
        unavailable = self._available()
        advanced = self._world.time_sec - start
        self._log('decision_idle_elapsed', requested_wall_seconds=seconds,
                  advanced_sim_seconds=advanced, reason=unavailable or 'OK')
        return dict(ok=unavailable is None, advanced_sec=advanced, reason=unavailable or 'OK',
                    budget=self._budget_status())

    def execute_route(self, *, route_id: str, operation_id: str, max_sim_seconds: float = 8., rationale: str = ''):
        try:
            duration = finite_number(max_sim_seconds, 'max_sim_seconds')
        except ValueError:
            return self._failure('INVALID_ARGUMENT')
        if not .1 <= duration <= 15 or not isinstance(operation_id, str) or not 1 <= len(operation_id) <= 128:
            return self._failure('INVALID_ARGUMENT')
        signature = json.dumps([route_id, duration, str(rationale)], ensure_ascii=False)
        if operation_id in self._operations:
            old_signature, response = self._operations[operation_id]
            return deepcopy(response) if old_signature == signature else self._failure('OPERATION_CONFLICT')
        unavailable = self._available()
        if unavailable:
            return self._failure(unavailable)
        if len(self._operations) >= self._max_operations:
            self.stop(terminate=True, reason='OPERATION_BUDGET_EXHAUSTED')
            return self._failure('OPERATION_BUDGET_EXHAUSTED')
        if route_id != self._active_route or route_id not in self._routes:
            return self._failure('INVALID_ROUTE_HANDLE')
        plan = self._routes[route_id]
        if plan.get('invalidation'):
            # A historical witness can forbid reusing this handle, not prove a
            # current free route. The caller must explicitly request a new plan.
            return self._failure('PATH_BLOCKED', route_id=route_id,
                route_invalidation=deepcopy(plan['invalidation']),
                requires_explicit_new_plan=True, last_validation_is_historical=True)
        # Re-read actual sensor state after the MCP supervisor charged decision
        # waits. A plan's old time/pose never licenses starting a late operation.
        # Existing freshness failure handling below remains authoritative.
        try:
            measured = self._observe()
        except ObservationError:
            measured = None
        estimate = None
        if measured is not None and measured['frame_id'] == self._world.frame_id:
            start_pose = PoseState(**measured['pose'], speed_mps=measured['speed_mps'])
            estimate = forecast(plan['points'], start_pose, self._world.checkpoints[plan['target']],
                                self._world.plant.config, segment_seconds=duration,
                                route_tracker=plan['tracker'], frame_id=measured['frame_id'],
                                sim_time_sec=self._world.time_sec)
            required = estimate['admission_required_sec']
            if required is None or self._budget_status()['remaining_sim_sec'] < required:
                reason = 'BUDGET_ESTIMATE_UNAVAILABLE' if required is None else 'INSUFFICIENT_COMPLETION_BUDGET'
                self._log('execution_budget_rejected', route_id=route_id, operation_id=operation_id,
                          reason=reason, execution_estimate=estimate, budget=self._budget_status())
                return self._failure(reason, execution_estimate=estimate, **self._task_progress())
        self._operation_routes[operation_id] = route_id
        target = self._world.checkpoints[plan['target']]
        self._metrics['executions'] += 1
        self._log('execution_started', operation_id=operation_id, route_id=route_id, plan_id=plan['plan_id'],
                  rationale=str(rationale)[:1000], requested_duration_sec=duration)
        end_time = min(self._world.time_sec + duration, self._max_sim_seconds)
        reason = 'SEGMENT_COMPLETE'
        invalidation = None
        while self._world.time_sec < end_time - 1e-9:
            if self._terminated:
                reason = self._termination_reason
                break
            try:
                observation = self._observe()
            except ObservationError as exc:
                reason = exc.code
                self._metrics['safety_interventions'] += 1
                break
            # This controller has no delay compensator: motion requires a same-frame
            # packet. TTL alone does not license using evaluator truth for feedback.
            if observation['frame_id'] != self._world.frame_id:
                reason = 'STALE_OBSERVATION'
                self._metrics['safety_interventions'] += 1
                break
            pose = PoseState(**observation['pose'], speed_mps=observation['speed_mps'])
            signature_now = static_obstacle_signature(observation['obstacles'])
            if signature_now != plan['static_signature']:
                blockers = self._remaining_static_blockers(plan, pose, observation['obstacles'])
                if blockers:
                    reason = 'PATH_BLOCKED'
                    self._metrics['safety_interventions'] += 1
                    invalidation = dict(reason=reason,blocked_by=blockers,
                        observation_frame=observation['frame_id'],observed_at=observation['observed_at'],
                        source='fresh_measured_static_objects_on_remaining_route',
                        current_pose=deepcopy(observation['pose']))
                    plan['invalidation'] = deepcopy(invalidation)
                    self._log('remaining_route_invalidated',route_id=route_id,
                        operation_id=operation_id,**invalidation)
                    break
                plan['static_signature'] = signature_now
            if math.dist((pose.x, pose.y), target) <= .6 and pose.speed_mps <= .05:
                reason = 'ARRIVED'
                break
            if self._world.scenario == 'execution_fault' and self._world.time_sec >= 4.:
                reason = 'EXECUTION_FAULT'
                break
            try:
                control = plan['tracker'].command(x=pose.x, y=pose.y, yaw_rad=pose.yaw_rad,
                                                   speed_mps=pose.speed_mps, frame_id=self._world.frame_id,
                                                   sim_time_sec=self._world.time_sec,
                                                   dt_sec=self._world.plant.config.dt_sec)
                command = Action(throttle=control['throttle'], brake=control['brake'],
                                 steer_left_normalized=control['steer_left_normalized'])
                verdict = assess_action(pose, command, observation, now=self._world.time_sec,
                                        config=self._world.plant.config, bounds=self._world.bounds)
                if not verdict.allowed:
                    reason = verdict.reason
                    self._metrics['safety_interventions'] += 1
                    self._log('safety_rejection', operation_id=operation_id, verdict=asdict(verdict), command=asdict(command))
                    break
                cte = min(segment_distance(pose.x, pose.y, *a, *b) for a, b in zip(plan['points'], plan['points'][1:]))
                self._advance(command, operation_id=operation_id, cte=cte,
                              control_feedback=control, safety_assessment=asdict(verdict))
            except Exception as exc:
                # Candidate generation failures never skip the mandatory brake below.
                reason = 'EXECUTION_ERROR'
                self._metrics['safety_interventions'] += 1
                self._log('execution_error', operation_id=operation_id,
                          exception_type=type(exc).__name__, detail=str(exc)[:1000])
                break
        self._brake(operation_id)
        plan['tracker'].reset_controller()
        try:
            self._observe()
        except ObservationError:
            pass  # A frozen sensor remains unavailable after braking.
        pose = self._world.plant.state
        if reason == 'SEGMENT_COMPLETE' and math.dist((pose.x, pose.y), target) <= .6:
            reason = 'ARRIVED'
        if self._world.time_sec >= self._max_sim_seconds:
            reason = 'TIME_BUDGET_EXHAUSTED'
            self._terminated, self._termination_reason = True, reason
        if self._terminated:
            reason = self._termination_reason
        ok = reason in ('SEGMENT_COMPLETE', 'ARRIVED')
        result = dict(ok=ok, status=reason if ok else 'STOPPED', reason=reason,
                      operation_id=operation_id, route_id=route_id, plan_id=plan['plan_id'], target=plan['target'],
                      sim_time_sec=self._world.time_sec, state=self._feedback_state(),
                      stopping_verified=pose.speed_mps == 0., verification_source='simulator_evaluator',
                      execution_estimate=estimate,
                      metrics=deepcopy(self._metrics), **self._task_progress())
        if invalidation is not None:
            result.update(route_invalidation=invalidation,requires_explicit_new_plan=True)
        self._operations[operation_id] = (signature, deepcopy(result))
        # Include this completed execution in its immutable operation snapshot.
        # An identical retry returns the original payload; MCP separately refreshes
        # envelope budget metadata without running the operation again.
        result['budget'] = self._budget_status()
        self._operations[operation_id] = (signature, deepcopy(result))
        self._log('execution_finished', **result)
        return result

    def _remaining_static_blockers(self, plan, pose, obstacles):
        return observed_remaining_blockers(plan['points'],pose,obstacles,self._world.plant.config)

    def inspect(self, *, target: str):
        unavailable = self._available()
        if unavailable:
            return self._failure(unavailable)
        if not isinstance(target, str) or target not in self._world.checkpoints:
            return self._failure('INVALID_TARGET')
        try:
            observation = self._observe()
        except ObservationError as exc:
            return self._failure(exc.code)
        if observation['frame_id'] != self._world.frame_id:
            return self._failure('STALE_OBSERVATION')
        pose = PoseState(**observation['pose'], speed_mps=observation['speed_mps'])
        if pose.speed_mps > .05 or math.dist((pose.x, pose.y), self._world.checkpoints[target]) > .6:
            return self._failure('INSPECTION_POSE_NOT_REACHED')
        evidence = dict(target=target, measured_temperature_c=78. if target == 'pump_b' else 35.,
                        anomaly=target == 'pump_b', source='synthetic_fixture',
                        frame_id=observation['frame_id'], observed_at=observation['observed_at'],
                        pose=self._feedback_state(), detection_accuracy_claimed=False)
        self._inspections[target] = evidence
        self._inspection_event_ids[target] = self._log('inspection_verified', evidence=evidence)
        self._record_completion_receipt()
        return dict(ok=True, evidence=deepcopy(evidence), **self._task_progress(),
                    budget=self._budget_status(), completion_receipt=deepcopy(self._completion_receipt))

    def _record_completion_receipt(self):
        """Capture the first complete inspection verdict in the same tool call.

        Historical evidence only: later timeout, stop or safety violations still
        determine verify(). No terminal-success latch or extra sensor acquisition.
        """
        if self._completion_receipt is not None or set(self._inspections) != set(self._world.checkpoints):
            return
        verdict = self._evaluate_verdict()
        criteria = self._completion_criteria()
        references = []
        for target in self._world.checkpoints:
            encoded = json.dumps(self._inspections[target], sort_keys=True, separators=(',', ':'),
                                 ensure_ascii=False, allow_nan=False).encode('utf-8')
            references.append(dict(target=target, event_id=self._inspection_event_ids[target],
                                   evidence_sha256=hashlib.sha256(encoded).hexdigest()))
        receipt = dict(schema_version=1, receipt_id='completion-1', event_id=len(self._events)+1,
                       episode_id=self._world.episode_id, frame_id=self._world.frame_id,
                       sim_time_sec=self._world.time_sec, deadline_sim_sec=self._max_sim_seconds,
                       criteria_met_at_completion=verdict['task_success'], criteria=criteria,
                       inspection_references=references, metrics=deepcopy(verdict['metrics']),
                       validation_scope=verdict['validation_scope'])
        self._completion_receipt = deepcopy(receipt)
        self._log('mission_completion_evaluated', receipt=receipt)

    def reobserve(self, *, rationale: str = ''):
        unavailable = self._available()
        if unavailable:
            return self._failure(unavailable)
        self._brake()
        unavailable = self._available()
        if unavailable:
            return self._failure(unavailable)
        # Recovery consumes simulated time and returns a new frame, not rewritten old data.
        for _ in range(5):
            dt = min(self._world.plant.config.dt_sec, self._max_sim_seconds - self._world.time_sec)
            self._advance(STOP, dt_sec=dt)
            unavailable = self._available()
            if unavailable:
                return self._failure(unavailable)
        self._world.refresh_sensor()
        self._log('sensor_reobservation', rationale=str(rationale)[:1000])
        return self.observe()

    def wait(self, *, seconds: float = 1., rationale: str = ''):
        try:
            seconds = finite_number(seconds, 'seconds')
        except ValueError:
            return self._failure('INVALID_ARGUMENT')
        if not 0 < seconds <= 5:
            return self._failure('INVALID_ARGUMENT')
        unavailable = self._available()
        if unavailable:
            return self._failure(unavailable)
        self._brake()
        unavailable = self._available()
        if unavailable:
            return self._failure(unavailable)
        end = min(self._world.time_sec + seconds, self._max_sim_seconds)
        while self._world.time_sec < end:
            dt = min(self._world.plant.config.dt_sec, end - self._world.time_sec)
            self._advance(STOP, dt_sec=dt)
            unavailable = self._available()
            if unavailable:
                self._log('wait_interrupted', reason=unavailable, requested_seconds=seconds)
                return self._failure(unavailable)
        self._log('waited', requested_seconds=seconds, rationale=str(rationale)[:1000])
        return self.observe()

    def stop(self, *, terminate: bool = False, reason: str = 'requested_stop'):
        if type(terminate) is not bool:
            return self._failure('INVALID_ARGUMENT')
        self._brake()
        if terminate:
            self._terminated, self._termination_reason = True, str(reason)[:1000]
        self._log('stopped', terminate=terminate, reason=str(reason)[:1000], state=self._state())
        return dict(ok=True, status='TERMINATED' if self._terminated else 'STOPPED', state=self._feedback_state(),
                    stopping_verified=self._stopping_verified(), verification_source='simulator_evaluator',
                    budget=self._budget_status())

    def verify(self):
        # Budget termination is enforced here as well as in motion/recovery.
        # _available -> stop does not call verify, so this cannot recurse.
        self._available()
        return self._evaluate_verdict()

    def _completion_criteria(self):
        pose = self._world.plant.state
        return dict(all_targets_inspected=set(self._inspections) == set(self._world.checkpoints),
                    at_exit=math.dist((pose.x, pose.y), self._world.checkpoints['exit_c']) <= .6,
                    stopped=pose.speed_mps <= .05, not_terminated=not self._terminated,
                    no_safety_violations=all(self._metrics[k] == 0 for k in
                        ('collisions', 'boundary_violations', 'speed_violations')),
                    before_deadline=self._world.time_sec < self._max_sim_seconds)

    def _evaluate_verdict(self):
        """Pure evaluator shared by current verification and completion receipts."""
        pose = self._world.plant.state
        violations = sum(self._metrics[k] for k in ('collisions', 'boundary_violations', 'speed_violations'))
        success = all(self._completion_criteria().values())
        ctes = sorted(self._ctes)
        metrics = dict(self._metrics, cte_rmse_m=math.sqrt(sum(x*x for x in ctes) / len(ctes)) if ctes else None,
                       cte_p95_m=ctes[min(len(ctes)-1, math.ceil(len(ctes)*.95)-1)] if ctes else None)
        return dict(task_success=success, safe_termination=self._terminated and pose.speed_mps == 0. and violations == 0,
                    terminated=self._terminated, termination_reason=self._termination_reason,
                    completed_targets=list(self._inspections), missing_targets=[t for t in self._world.checkpoints if t not in self._inspections],
                    final_state=self._feedback_state(readonly=True), stopping_verified=self._stopping_verified(),
                    verification_source='simulator_evaluator', sim_time_sec=self._world.time_sec, metrics=metrics,
                    decision_source=self._decision_source, model_call_verified=False,
                    model_usage=None, model_cost=None, inspections=deepcopy(self._inspections),
                    budget=self._budget_status(), completion_receipt=deepcopy(self._completion_receipt),
                    validation_scope='synthetic_2d_kinematics_only')

    def evidence(self):
        return deepcopy(dict(schema_version=3, summary=self.verify(), events=self._events, trace=self._trace, plans=self._plans,
                    completion_receipt=self._completion_receipt,
                    trajectory_contract=dict(plan_parameterization='spatial', coordinate_frame='world',
                        yaw_positive='counterclockwise', position_unit='m', angle_unit='rad', time_unit='s',
                        nominal_control_step_sec=self._world.plant.config.dt_sec,
                        geometric_cte_sample_phase='before_action', recorded_state_sample_phase='after_action',
                        timed_reference_trajectory_available=False),
                    episode_id=self._world.episode_id,
                    scenario=self._world.scenario, seed=self._world.seed, bounds=self._world.bounds,
                    checkpoints=self._world.checkpoints, plant_config=asdict(self._world.plant.config),
                    evaluation_final_state=self._state(), evaluation_only_fields=['evaluation_final_state', 'trace']))

    def save(self, directory: str | Path, *, tool_context=None):
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)
        evidence = self.evidence()
        if hasattr(self, 'task_status'):
            # Persist the same supervisor-owned CURRENT envelope as MCP.
            # Never infer it from a cached receipt or an old planning event.
            evidence['summary']['task'] = deepcopy(self.task_status())
        if tool_context is not None:
            # Only additive supervisor metadata; it cannot replace a verdict,
            # sensor field, mission deadline, operation count or receipt.
            evidence['summary']['budget'].update(
                clock_policy=tool_context['clock_policy'],
                idle_wall_charged_between_tools=tool_context['idle_wall_charged_between_tools'],
                snapshot_phase=tool_context.get('snapshot_phase', 'tool_return'))
            evidence['summary']['tool_budget'] = deepcopy(tool_context['tool_budget'])
            if 'recovery' in tool_context:
                evidence['summary']['recovery'] = deepcopy(tool_context['recovery'])
        (path / 'summary.json').write_text(json.dumps(evidence['summary'], ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        for name in ('events', 'trace', 'plans'):
            (path / f'{name}.jsonl').write_text(''.join(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n'
                                                     for row in evidence[name]), encoding='utf-8')
        (path / 'run.json').write_text(json.dumps({k:v for k,v in evidence.items() if k not in ('events','trace','plans')},
                                                 ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
