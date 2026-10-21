"""Local semantic receipts for actual tool requests, with no extra model request."""
from __future__ import annotations

from copy import deepcopy


_CONTRACTS = {
    'observe': ('read_state', ['strict measured packet'], 'structured state or explicit invalid-state failure'),
    'plan_route': ('select_route', ['next known target','enumerated route','valid observation','stationary'], 'accepted route handle or explicit infeasibility'),
    'execute_route': ('bounded_move', ['active route handle','unique operation ID','current measured state','completion budget','per-step safety'], 'bounded motion ends at rest; independent execution result'),
    'inspect': ('inspect_target', ['known next target','at target','independently verified stop','valid observation'], 'synthetic inspection evidence or explicit rejection'),
    'reobserve': ('refresh_sensor', ['remaining recovery and tool budget'], 'valid measured packet or explicit failure'),
    'wait': ('bounded_wait', ['fixed maximum wait','mission/tool/recovery budget'], 'bounded stopped wait and measured feedback'),
    'stop': ('safe_stop', ['always available'], 'independent stopped/terminated state'),
    'verify': ('verify_completion', ['always available'], 'independent final verdict; safe stop is not task success'),
    'navigate': ('select_route_and_bounded_move', ['explicit next target and route', 'unique operation ID',
        'task revision and permissions', 'planning and recovery budget', 'fresh state', 'per-step safety'],
        'single chosen route is planned and bounded; failure returns to Agent; inspection only after real arrival and stopping'),
}


class ActionReceipts:
    """Describe requests and outcomes already enforced by the frozen executor.

    The receipts cannot dispatch tools, choose a replacement route, grant safety
    permission, infer model identity or fill in an absent decision rationale.
    """
    def __init__(self):
        self.contracts = []
        self.recovery_decisions = []
        self._pending = {}
        self._failure = None

    def accept_event(self, event):
        if event['event'] == 'tool_call_started':
            tool = event['tool']
            if tool not in _CONTRACTS:
                return
            kind, preconditions, outcome = _CONTRACTS[tool]
            raw = deepcopy(event['arguments'])
            effective = deepcopy(event.get('effective_arguments', raw))
            receipt = dict(schema_version=1, contract_id=f"action-{len(self.contracts)+1}",
                action=kind, tool=tool, tool_call_id=event['tool_call_id'],
                request_event_id=event['event_id'], request_sim_time_sec=event['sim_time_sec'],
                raw_arguments=raw, effective_arguments=effective,
                target=effective.get('target'), route=effective.get('route'),
                route_id=effective.get('route_id'), operation_id=effective.get('operation_id'),
                decision_summary=raw.get('rationale', raw.get('reason')),
                preconditions=preconditions, expected_observable_outcome=outcome,
                enforcement_owner='frozen_executor_and_mcp', motion_authorized_by_receipt=False,
                decision_source='recorded_tool_request; model_origin_not_authenticated_here',
                result_event_id=None, result_status=None, result_reason=None)
            self.contracts.append(receipt)
            self._pending[event['tool_call_id']] = receipt
            if self._failure and tool in ('plan_route','navigate','reobserve','wait','stop'):
                self.recovery_decisions.append(dict(schema_version=1,
                    decision_id=f"recovery-decision-{len(self.recovery_decisions)+1}",
                    failure_code=self._failure['code'],
                    evidence_refs=[dict(kind='mission_event', event_id=self._failure['event_id']),
                                   dict(kind='mission_event', event_id=event['event_id'])],
                    selected_action=dict(tool=tool, arguments=effective),
                    decision_summary=receipt['decision_summary'],
                    expected_observable_outcome=outcome,
                    stop_condition='frozen safety, freshness, completion and recovery budgets remain authoritative',
                    interpretation='request following observed failure, not inferred root cause or hidden reasoning'))
        elif event['event'] == 'tool_call_finished':
            result = event['result']
            receipt = self._pending.pop(event['tool_call_id'], None)
            if receipt is not None:
                receipt.update(result_event_id=event['event_id'],
                    result_status=result.get('status'), result_reason=result.get('reason'),
                    result_ok=result.get('ok'), stopping_verified=result.get('stopping_verified'))
            if result.get('ok') is False:
                self._failure = dict(code=result.get('reason'), event_id=event['event_id'])
            elif event['tool'] in ('execute_route', 'navigate') and result.get('status') == 'ARRIVED':
                self._failure = None
