"""Lossless decision fields, with repeated explanatory metadata removed.

The full result remains in the local execution journal. This module cannot
read the simulator, refresh sensors, change a command or authorize motion.
Unknown fields are preserved rather than silently dropping future failures.
"""
from copy import deepcopy


def compact_feedback(result):
    value = deepcopy(result)
    inspection = value.get('inspection')
    if isinstance(inspection, dict):
        for key in ('budget', 'completed_targets', 'remaining_targets', 'next_target', 'completion_receipt'):
            # The enclosing result has the authoritative current envelope and
            # progress. The full historical inspector payload stays in evidence.
            if key in value:
                inspection.pop(key, None)
    recovery = value.get('recovery')
    if isinstance(recovery, dict):
        for key in ('route_history_scope', 'source', 'counting_contract',
                    'decision_hint_is_not_an_action', 'observation_refreshed'):
            recovery.pop(key, None)
        # The history and numerical limit remain; long duplicate action rows
        # are evidence, not an additional observation or planning candidate.
        recovery.pop('actions', None)
    estimate = value.get('execution_estimate')
    if isinstance(estimate, dict):
        for key in ('assumptions', 'excludes', 'method', 'start_pose',
                    'assumed_followup_segment_seconds', 'validation_scope'):
            estimate.pop(key, None)
    budget = value.get('budget')
    if isinstance(budget, dict):
        for key in ('schema_version', 'snapshot_phase', 'recovery', 'idle_wall_charged_between_tools'):
            budget.pop(key, None)
        if budget.get('max_execution_operations') == 80:
            budget.pop('max_execution_operations')
    tool_budget = value.get('tool_budget')
    if isinstance(tool_budget, dict):
        tool_budget.pop('exempt_tools', None)
        if tool_budget.get('max_tool_requests') == 120:
            tool_budget.pop('max_tool_requests')
    return value


COMPACT_DESCRIPTIONS = {
    'observe': 'Read synthetic sensor state, freshness, task progress and budgets. Unknown is not free space.',
    'plan_route': 'Choose a direct/upper/lower candidate for next_target from feedback. Returns a checked handle, conditional time estimate and recovery history. No motion; no guarantee of future feasibility.',
    'execute_route': 'Selecting this tool requests the fixed arrival-inspection-15s-v1 profile: checked route execution for at most 15 simulation seconds, braking, and inspection ONLY on ARRIVED+verified stop. Pass route_id, a new operation_id, optional brief rationale. Duration=15 and inspect_on_arrival=true are immutable profile constants; omit them or pass exactly those values. Freshness, safety, time admission and recovery limits remain enforced. Identical retries replay historical results, never new motion or sampling.',
    'inspect': 'Measure a synthetic fixture only at its target and stopped. Historical completion_receipt cannot override final verify.',
    'reobserve': 'Stay stopped, consume time and reacquire an actual sensor frame. Use for missing/stale state.',
    'wait': 'Stay stopped for REQUIRED 0<seconds<=5; environment advances, then observe. Use with evidence of a transient moving blockage.',
    'stop': 'Brake; terminate=true irreversibly ends the incomplete mission. Does not extend budgets.',
    'verify': 'Independent CURRENT verdict, inspection evidence and metrics. Call immediately after final inspection or termination. Safe termination is not task success.',
}

COMPACT_CONTRACT = (
    'Synthetic 2D Ackermann only; targets panel_a,pump_b,exit_c in order. '
    'Missing/stale state must not be invented. Each tool returns current budgets and '
    'stored-observation recovery status; metadata does not refresh sensors. '
    'Recovery limit=3 for the whole mission: after failure, each dispatched same-target '
    'plan/reobserve/wait consumes one; initial failure, execute,inspect,observe,stop,verify '
    'do not. Historical candidate checks are not current/global feasibility proof. '
    'Clock: fixed-step motion plus real between-tool decision/approval waits AFTER '
    'first planning; tool computation excluded. Stop/verify remain callable after '
    'request cap=120 but do not extend deadline=300s or operations=80. '
    'Time forecasts assume the same controller/plant, no future obstacle/sensor fault '
    'and fresh state; include braking and fixed 5s post-arrival reserve, exclude '
    'future model waits/recovery/targets. They are not guarantees. '
    'Cached operation state is historical; recovery.last_execution_result_replayed '
    'marks replay, while envelope budgets are current. Full explanations and action '
    'history are retained in local evidence, not repeated in each model result. '
    'Selecting execute_route requests immutable arrival-inspection-15s-v1 '
    '(max_sim_seconds=15,inspect_on_arrival=true) to combine mechanical execution and '
    'inspection; failure/partial movement never runs inspection or another route. '
    'No automatic replan, reobserve, wait, recovery, movement to another target or final verify.'
)
