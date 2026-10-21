"""Complete task-aware fixed comparator; reads only the same tool feedback."""
ROUTES = ('direct', 'upper', 'lower')
INVALID = {'STALE_OBSERVATION', 'UNKNOWN_OBSERVATION', 'INVALID_OBSERVATION'}


def run_policy(call, *, recovery=True):
    result = call('observe')
    operation = 0
    route_id, context_key, tried = None, None, set()

    def finish(reason=None):
        if reason:
            call('stop', terminate=True, reason='Task-aware fixed policy: ' + reason)
        return call('verify')

    for _ in range(120):
        if not result.get('ok'):
            return finish(result.get('reason', 'OBSERVE_FAILED'))
        task = result.get('task')
        if not isinstance(task, dict):
            return finish('TASK_CONTRACT_UNAVAILABLE')
        target = task['next_target']
        if target is None:
            return finish()
        key = (task['revision'], target)
        if key != context_key:
            context_key, route_id, tried = key, None, set()
            route = next((r for r in ROUTES if r in task['permitted_routes']), None)
            if route is None:
                return finish('NO_PERMITTED_ROUTE')
        if route_id is None:
            tried.add(route)
            result = call('plan_route', target=target, route=route,
                rationale='Complete fixed policy: current task target and a permitted candidate.')
            if result.get('ok') and result.get('status') == 'ARRIVED':
                result = call('inspect', target=target)
                continue
            if result.get('ok'):
                route_id = result['route_id']
                continue
        else:
            operation += 1
            result = call('execute_route', route_id=route_id, operation_id=f'fixed-{operation}',
                rationale='Complete fixed policy: resume the actual handle with a new operation id.')
            if result.get('ok'):
                if result.get('status') == 'ARRIVED' and not (
                    result.get('inspection', {}).get('performed') and result['inspection'].get('ok')):
                    return finish('INSPECTION_NOT_VERIFIED')
                continue
        reason = result.get('reason', 'UNSPECIFIED_FAILURE')
        context = result.get('recovery', {})
        if not recovery or context.get('remaining_actions', 0) <= 0:
            return finish(reason)
        if reason in INVALID:
            result = call('reobserve', rationale='Complete fixed policy: actual invalid-state feedback requires reacquisition.')
            continue
        if reason == 'TASK_REVISION_CHANGED':
            route_id = None
            result = call('observe')
            continue
        if reason == 'PATH_BLOCKED' and context.get('current_observation', {}).get('moving_obstacles_present') is True:
            result = call('wait', seconds=5, rationale='Complete fixed policy: bounded wait for observed moving blockage.')
            continue
        if reason in ('PATH_BLOCKED', 'NO_FEASIBLE_CONNECTION'):
            available = result['task']['permitted_routes']
            candidates = context.get('untried_routes', [])
            route = next((r for r in ROUTES if r not in tried and r in available and r in candidates), None)
            if route is not None:
                route_id = None
                # Preserve real failure feedback while letting the bounded loop plan.
                result = dict(result, ok=True)
                continue
        return finish(reason)
    return finish('CLIENT_LOOP_LIMIT')
