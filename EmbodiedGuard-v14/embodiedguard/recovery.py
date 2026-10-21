"""Transport-owned recovery accounting; no planning or actuator decisions."""
from copy import deepcopy

RECOVERABLE = frozenset(('PATH_BLOCKED', 'NO_FEASIBLE_CONNECTION', 'STALE_OBSERVATION',
    'UNKNOWN_OBSERVATION', 'INVALID_OBSERVATION', 'EXECUTION_FAULT', 'EXECUTION_ERROR',
    'GLOBAL_SEARCH_FAILED', 'GLOBAL_PATH_AUDIT_REJECTED'))


class RecoveryLedger:
    def __init__(self):
        self.limit = 3
        self.actions = []
        self.failure = None
        self._seen_operations = set()
        self._last_execution_replayed = False

    def is_recovery(self, tool, arguments):
        return self.failure is not None and (tool in ('reobserve', 'wait') or
            tool == 'plan_route' and arguments.get('target') == self.failure['target'])

    def note_replay(self, *, execution_occurred):
        """A transport cache replay must not reopen failure or consume recovery."""
        self._last_execution_replayed = bool(execution_occurred)

    def record(self, tool, arguments, result, *, next_target):
        self._last_execution_replayed = False
        if result.get('reason') == 'RECOVERY_BUDGET_EXHAUSTED':
            return  # A rejected attempt did not dispatch the recovery action.
        if self.is_recovery(tool, arguments):
            self.actions.append(dict(tool=tool, target=self.failure['target'],
                route=arguments.get('route', 'direct') if tool == 'plan_route' else None,
                plan_id=result.get('plan_id'), reason=result.get('reason'), ok=result.get('ok')))
        if tool == 'execute_route' and result.get('operation_id'):
            operation = result['operation_id']
            if operation in self._seen_operations:
                self._last_execution_replayed = True
                return  # A cached old outcome cannot reopen a resolved failure.
            self._seen_operations.add(operation)
        if tool in ('plan_route', 'execute_route', 'inspect', 'observe') and result.get('reason') in RECOVERABLE:
            self.failure = dict(tool=tool, reason=result['reason'],
                target=next_target,
                plan_id=result.get('plan_id'), operation_id=result.get('operation_id'))
        elif tool in ('inspect', 'execute_route') and result.get('ok'):
            self.failure = None

    def snapshot(self, context):
        remaining = max(0, self.limit - len(self.actions))
        hint = 'compare_feedback_and_budget'
        observation = context['current_observation']
        if remaining == 0 and self.failure:
            hint = 'verify_or_safe_stop'
        elif self.failure:
            if not observation['valid']:
                hint = 'restore_observation'
            elif self.failure['reason'] in ('STALE_OBSERVATION', 'UNKNOWN_OBSERVATION', 'INVALID_OBSERVATION'):
                hint = 'resume_or_replan_from_valid_state'
            elif observation['moving_obstacles_present']:
                hint = 'consider_wait_or_replan'
            elif context['untried_routes']:
                hint = 'compare_untried_routes'
            else:
                hint = 'verify_or_safe_stop'
        return deepcopy(dict(**context, max_actions=self.limit, consumed_actions=len(self.actions),
            remaining_actions=remaining, active_failure=self.failure, actions=self.actions,
            decision_hint=hint, decision_hint_is_not_an_action=True,
            last_execution_result_replayed=self._last_execution_replayed,
            counting_contract='After a failure, each dispatched plan for that target and each reobserve/wait consumes one; initial failed attempt, execution, inspect, observe, stop and verify do not. No reset after target completion.'))
