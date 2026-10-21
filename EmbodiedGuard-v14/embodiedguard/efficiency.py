"""Optional decision-preserving transport optimizations around the v7 executor.

No alternative route, recovery or next goal is selected here. Resume metadata is
read-only; a conditional forecast never replaces current per-step safety checks.
"""
from copy import copy, deepcopy
import json
import hashlib

from .sim import PoseState
from .state import ObservationError
from .timing import forecast

PROFILES = ('baseline', 'resume', 'bundle', 'lean', 'focused')
RESUME_CONTRACT = (' recovery.resume is read-only, conditional observed-state metadata, not permission or a sensor refresh. '
    'Eligible unfinished handles may be continued by Agent execute_route with a NEW operation_id; all execution checks remain mandatory.')
LEAN_CONTRACT = (
    'Synthetic 2D Ackermann only. Current supervisor-owned task revision, required order and permitted routes are authoritative; '
    'the Agent cannot edit them. Unknown/stale sensing is not free space. '
    'navigate explicitly selects ONE target and route: reuse the exact matching active handle for the current target, otherwise checked planning; '
    'at most 15s tracking, braking, inspection only on ARRIVED plus verified stop. Reuse preserves the tracker and still invokes every original execution check; '
    'an invalid matching handle is rejected, never silently replaced. plan_route explicitly requests a new geometry. '
    'Failure or partial completion returns; no automatic alternative, recovery, next target or verify. '
    'execute_route continues a chosen handle with a NEW operation_id; fixed duration=15 and inspect_on_arrival=true. '
    'Identical operation retries are historical, never new motion or sampling; current envelope budgets/task still apply. '
    'After failure each dispatched same-target plan (including navigate planning), reobserve or wait consumes one of 3 mission-wide recovery actions. '
    'Clock includes fixed-step motion plus real between-tool waits AFTER first planning; deadline=300s, executions=80, requests=120. '
    'stop and verify remain available but extend no budget. Forecasts exclude future model waits, failures and targets; they are conditional, not guarantees. '
    'recovery.resume does not resense or authorize motion; execute always checks fresh state and safety. '
    'Unchanged long task descriptions may be omitted with details_revision; current constraints stay present. '
    'Agent must call final verify immediately after actual final inspection or termination; historical completion is not the current verdict.'
)
EFFICIENT_CONTRACT = (
    ' The disclosed efficiency interface adds recovery.resume: an observed-state, '
    'conditional suggestion, never motion permission or a sensor refresh. '
    'If an unfinished route is eligible_to_attempt, the Agent may continue it '
    'using execute_route with a NEW operation_id instead of planning it again. '
    'navigate(target,route,operation_id) explicitly selects ONE target and route '
    'and requests arrival-inspection-15s-v1 execution on its exact matching '
    'active current-target handle, or a checked new plan when none matches. '
    'Reuse keeps the route/tracker; every original execution check remains. '
    'A rejected matching handle is not silently replanned; plan_route explicitly requests new geometry. '
    'Failure or partial completion returns immediately; no alternative, '
    'reobserve, wait, next target or final verify is dispatched automatically. '
    'Nested planning consumes the same mission-wide recovery budget. '
    'An identical navigate retry returns a historical receipt without new work. '
    'Current task constraints and budgets remain in every return; unchanged '
    'long task descriptions may be omitted with details_revision. Full task '
    'details are sent on revision change and in final verify.'
)


class RevisionFeedback:
    """Drop known repeated explanations only; keep unknown and current fields."""
    def __init__(self):
        self._last_revision = None

    def compact(self, result, *, tool):
        value = deepcopy(result)
        task = value.get('task')
        if not isinstance(task, dict) or type(task.get('revision')) is not int:
            return value
        inspection = value.get('inspection')
        if isinstance(inspection, dict) and inspection.get('task') == task:
            inspection.pop('task', None)  # Identical authoritative enclosing task.
        revision = task['revision']
        if revision == self._last_revision and tool != 'verify':
            task.pop('brief', None)
            task.pop('updates', None)
            task['details_revision'] = revision
        self._last_revision = revision
        return value


class EfficiencyLayer:
    def __init__(self, mission, ledger, execution_bundle, profile):
        self.mission = mission
        self.ledger = ledger
        self.execution_bundle = execution_bundle
        self.profile = profile
        self.feedback = RevisionFeedback()
        self._navigation_receipts = {}
        self._forecast_cache = None

    def resume_context(self, context):
        mission = self.mission
        route_id = mission._active_route
        route = mission._routes.get(route_id)
        revision = getattr(mission, '_task_revision', None)
        permitted = getattr(mission, '_permitted_routes', ('direct', 'upper', 'lower'))
        observation = context['current_observation']
        budget = mission._budget_status()
        if route is None or route['target'] != context['next_target'] or mission._terminated:
            return dict(route_id=None, eligible_to_attempt=False,
                reason='TERMINATED' if mission._terminated else 'NO_CURRENT_ROUTE',
                requires_execution_safety_check=True, sensor_refreshed=False)
        if route.get('invalidation'):
            return dict(route_id=route_id,target=route['target'],route=route['route'],
                eligible_to_attempt=False,reason='ROUTE_INVALIDATED',
                last_validation=deepcopy(route['invalidation']),last_validation_is_historical=True,
                requires_explicit_new_plan=True,requires_execution_safety_check=True,
                sensor_refreshed=False)
        checks = dict(route_known=route is not None,
            target_current=bool(route and route['target'] == context['next_target']),
            task_revision_matches=bool(route and (revision is None or route.get('task_revision') == revision)),
            route_permitted=bool(route and route['route'] in permitted),
            fresh_observation=observation['valid'], stationary=None,
            mission_available=not mission._terminated and budget['remaining_sim_sec'] > 0
                and budget['remaining_execution_operations'] > 0,
            completion_budget_conditional=None)
        estimate = None
        if all(checks[key] for key in ('route_known', 'target_current', 'task_revision_matches',
                                      'route_permitted', 'fresh_observation', 'mission_available')):
            try:
                # copy.current updates only the copied reader clock, never the store.
                packet = copy(mission._store).current(now=mission._world.time_sec)
            except ObservationError:
                checks['fresh_observation'] = False
            else:
                checks['stationary'] = packet['speed_mps'] == 0.
                if checks['stationary']:
                    key = (route_id, revision, packet['frame_id'], mission._world.time_sec,
                           tuple(packet['pose'].values()), packet['speed_mps'])
                    if self._forecast_cache is None or self._forecast_cache[0] != key:
                        predicted = forecast(route['points'], PoseState(**packet['pose'], speed_mps=packet['speed_mps']),
                            mission._world.checkpoints[route['target']], mission._world.plant.config,
                            route_tracker=route['tracker'], frame_id=packet['frame_id'],
                            sim_time_sec=mission._world.time_sec)
                        self._forecast_cache = (key, deepcopy(predicted))
                    predicted = self._forecast_cache[1]
                    estimate = {key:deepcopy(predicted.get(key)) for key in
                        ('estimated_arrival_and_stop_sec', 'admission_required_sec', 'post_arrival_reserve_sec', 'guaranteed')}
                    required = predicted.get('admission_required_sec')
                    checks['completion_budget_conditional'] = required is not None and required <= budget['remaining_sim_sec']
        eligible = all(value is True for value in checks.values())
        failing = [name for name, value in checks.items() if value is not True]
        return dict(route_id=route_id, target=route['target'] if route else None,
            route=route['route'] if route else None, task_revision=revision,
            eligible_to_attempt=eligible, checks=checks, unavailable_checks=failing,
            conditional_estimate=estimate, requires_execution_safety_check=True,
            sensor_refreshed=False, source='active_handle_and_stored_measured_packet',
            freshness_at_frame=observation['frame_id'])

    def execute(self, arguments):
        # The outer navigation operation owns its id even if planning failed.
        if arguments['operation_id'] in self._navigation_receipts:
            return self.mission._failure('OPERATION_CONFLICT')
        return self.execution_bundle.execute(arguments)

    def navigate(self, arguments, *, parent_call_id):
        args = dict(arguments)
        operation = args['operation_id']
        signature_parts = [args['target'], args['route'], str(args.get('rationale', ''))]
        if self.profile == 'focused':
            signature_parts.append(args['route_mode'])
        signature = json.dumps(signature_parts, ensure_ascii=False)
        if operation in self._navigation_receipts:
            old_signature, receipt = self._navigation_receipts[operation]
            if signature != old_signature:
                return self.mission._failure('OPERATION_CONFLICT')
            result = deepcopy(receipt)
            result['navigation']['replayed'] = True
            self.ledger.note_replay(execution_occurred=result['navigation']['execution_dispatched'])
            return result
        if operation in self.mission._operations or operation in self.mission._operation_routes:
            return self.mission._failure('OPERATION_CONFLICT')
        plan_args = {key:args[key] for key in ('target', 'route')}
        if 'rationale' in args:
            plan_args['rationale'] = args['rationale']
        # This selects only the route explicitly requested by the caller. An
        # exact active semantic match avoids rebuilding its geometry/tracker.
        # Its validity is NOT inferred here: execute_route remains the sole
        # owner of task-version, permission, sensing, budget and action checks.
        route_id = self.mission._active_route
        selected = self.mission._routes.get(route_id)
        reused = bool(selected and selected['target'] == args['target']
            and selected['route'] == args['route']
            and self.mission.recovery_context()['next_target'] == args['target'])
        if self.profile == 'focused' and args['route_mode'] == 'plan':
            reused = False
        resume_unavailable = self.profile == 'focused' and args['route_mode'] == 'resume' and not reused
        planning = None
        planning_dispatched = False
        source_plan_id = selected.get('plan_id') if reused else None
        if resume_unavailable:
            route_id = None
            result = self.mission._failure('NO_MATCHING_RESUME_ROUTE')
        elif reused:
            self.mission._log('navigation_route_reused', parent_tool_call_id=parent_call_id,
                target=args['target'], route=args['route'], route_id=route_id,
                source_plan_id=source_plan_id, task_revision=selected.get('task_revision'),
                scope='exact active handle for explicitly selected current target/route',
                requires_execution_safety_check=True, sensor_refreshed=False)
        elif self.ledger.is_recovery('plan_route', plan_args) and len(self.ledger.actions) >= self.ledger.limit:
            route_id = None
            result = dict(self.mission.stop(reason='RECOVERY_BUDGET_EXHAUSTED'),
                ok=False, reason='RECOVERY_BUDGET_EXHAUSTED', recovery_action_dispatched=False)
        else:
            planning_dispatched = True
            planning = self._child('plan_route', plan_args, parent_call_id,
                lambda: self.mission.plan_route(**plan_args))
            self.ledger.record('plan_route', plan_args, planning,
                next_target=self.mission.recovery_context()['next_target'])
            result = deepcopy(planning)
            route_id = planning.get('route_id') if planning.get('ok') else None
            source_plan_id = planning.get('plan_id')
        execution_dispatched = False
        window_outcomes = []
        window_limit = getattr(self.mission, '_navigation_window_limit', 1)
        if type(window_limit) is not int or window_limit not in (1,2):
            raise ValueError('Invalid supervisor navigation window bound')
        if route_id:
            for window in range(1,window_limit+1):
                child_operation = (operation if window_limit==1 else
                    'nav-'+hashlib.sha256(operation.encode('utf-8')).hexdigest()+'-'+str(window))
                execute_args = dict(route_id=route_id, operation_id=child_operation,
                    max_sim_seconds=15, inspect_on_arrival=True, rationale=args.get('rationale', ''))
                result = self._child('execute_route', execute_args, parent_call_id,
                    lambda: self.execution_bundle.execute(execute_args))
                execution_dispatched = True
                self.ledger.record('execute_route', execute_args, result,
                    next_target=self.mission.recovery_context()['next_target'])
                window_outcomes.append(dict(operation_id=child_operation,status=result.get('status'),
                    reason=result.get('reason'),stopping_verified=result.get('stopping_verified')))
                # Continuation is confined to the explicitly chosen handle.
                # Every original primitive reobserves, forecasts, checks every
                # action, counts its operation and brakes. NO failure recovery,
                # alternative planning, new goal or verify is done here.
                if (result.get('ok') is not True or result.get('status')!='SEGMENT_COMPLETE'
                        or self.mission.recovery_context()['next_target']!=args['target']):
                    break
        elif planning and planning.get('ok') and planning.get('status') == 'ARRIVED':
            # No route was needed. Verify a real stopped inspection; do not infer it.
            self._child('stop', dict(reason='navigation_arrival_stop'), parent_call_id,
                lambda: self.mission.stop(reason='navigation_arrival_stop'))
            measured = self._child('inspect', dict(target=args['target']), parent_call_id,
                lambda: self.mission.inspect(target=args['target']))
            self.ledger.record('inspect', dict(target=args['target']), measured,
                next_target=self.mission.recovery_context()['next_target'])
            result = dict(measured, status='ARRIVED' if measured.get('ok') else 'INSPECTION_FAILED',
                stopping_verified=self.mission._stopping_verified(),
                inspection=dict(requested=True, performed=True, **deepcopy(measured)))
        result.setdefault('inspection', dict(requested=True, performed=False, reason='PLANNING_NOT_ADMITTED'))
        result['operation_id'] = operation
        result['navigation'] = dict(profile='selected-route-arrival-inspection-15s-v2', replayed=False,
            execution_dispatched=execution_dispatched,
            planning_status=planning.get('status') if planning else None,
            planning_reason=planning.get('reason') if planning else None if reused else result.get('reason'),
            planning_dispatched=planning_dispatched, route_reused=reused,
            selected_route_id=route_id, source_plan_id=source_plan_id,
            target=args['target'], route=args['route'])
        if window_limit>1:
            result['navigation'].update(profile='selected-route-event-or-two-15s-windows-v14',
                original_window_limit=window_limit,original_windows_dispatched=len(window_outcomes),
                window_outcomes=window_outcomes,alternative_or_recovery_dispatched=False)
        self._navigation_receipts[operation] = (signature, deepcopy(result))
        return result

    def _child(self, tool, arguments, parent, invoke):
        started = self.mission._log('navigation_child_started', parent_tool_call_id=parent,
            tool=tool, arguments=arguments)
        result = invoke()
        self.mission._log('navigation_child_finished', parent_tool_call_id=parent,
            start_event_id=started, tool=tool, result=result)
        return result
