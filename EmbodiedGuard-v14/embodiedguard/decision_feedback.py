"""Optional brief projection of known duplicates; never an execution authority.

All original results remain in the journal. Unknown fields, negative or unknown
checks, current sensing, handles, task constraints and numerical budgets survive.
Final verify is unchanged. No simulator reads, mutations, sampling or model calls.
"""
from copy import deepcopy

RESUME_CHECKS = frozenset(('route_known', 'target_current', 'task_revision_matches',
    'route_permitted', 'fresh_observation', 'stationary', 'mission_available',
    'completion_budget_conditional'))
RECEIPT_CRITERIA = frozenset(('all_targets_inspected', 'at_exit', 'stopped',
    'not_terminated', 'no_safety_violations', 'before_deadline'))
METRICS = frozenset(('collisions', 'boundary_violations', 'speed_violations',
    'path_length_m', 'safety_interventions', 'executions', 'cte_rmse_m',
    'cte_p95_m', 'task_constraint_rejections'))
ESTIMATE = frozenset(('guaranteed', 'estimated_arrival_and_stop_sec',
    'estimated_execution_segments', 'post_arrival_reserve_sec',
    'admission_required_sec', 'segment_seconds'))
POSE_FIELDS = frozenset(('x', 'y', 'yaw_rad', 'speed_mps', 'frame_id', 'observed_at', 'source'))


def decision_feedback(result, *, tool):
    value = deepcopy(result)
    if tool == 'verify':
        return value
    task, recovery = value.get('task'), value.get('recovery')
    if isinstance(task, dict) and value.get('route_choices') == task.get('permitted_routes'):
        value.pop('route_choices', None)
    if value.get('verification_source') == 'simulator_evaluator':
        value.pop('verification_source')
    navigation = value.get('navigation')
    if isinstance(navigation, dict) and navigation.get('profile') == 'plan-arrival-inspection-15s-v1':
        navigation.pop('profile')
    if isinstance(recovery, dict):
        if isinstance(task, dict):
            for duplicate, authoritative in (('task_revision', 'revision'),
                    ('next_target', 'next_target'), ('permitted_routes', 'permitted_routes')):
                if duplicate in recovery and authoritative in task and recovery[duplicate] == task[authoritative]:
                    recovery.pop(duplicate)
        if recovery.get('decision_hint') == 'compare_feedback_and_budget':
            recovery.pop('decision_hint')
        resume = recovery.get('resume')
        if isinstance(resume, dict):
            checks = resume.get('checks')
            if (resume.get('eligible_to_attempt') is True and isinstance(checks, dict)
                    and set(checks) == RESUME_CHECKS and all(v is True for v in checks.values())):
                resume.pop('checks')
                resume['checks_all_true'] = True
                if resume.get('unavailable_checks') == []:
                    resume.pop('unavailable_checks')
            if isinstance(task, dict):
                for duplicate, authoritative in (('task_revision', 'revision'), ('target', 'next_target')):
                    if duplicate in resume and authoritative in task and resume[duplicate] == task[authoritative]:
                        resume.pop(duplicate)
            if resume.get('source') == 'active_handle_and_stored_measured_packet':
                resume.pop('source')
    # A previous forecast is not needed after actual arrival. Keep future schema
    # extensions and any uncertain/failed outcome rather than hiding new facts.
    estimate = value.get('execution_estimate')
    if (value.get('ok') is True and value.get('status') == 'ARRIVED'
            and isinstance(estimate, dict) and set(estimate) <= ESTIMATE
            and estimate.get('guaranteed') is False):
        value.pop('execution_estimate')
    inspection = value.get('inspection')
    evidence = inspection.get('evidence') if isinstance(inspection, dict) else None
    if isinstance(evidence, dict):
        pose, state = evidence.get('pose'), value.get('state')
        if (isinstance(pose, dict) and isinstance(state, dict) and set(pose) <= POSE_FIELDS
                and all(k in state and state[k] == v for k, v in pose.items())):
            evidence.pop('pose')
    # Only a fully passing historical receipt may elide its known detail rows.
    # Identity, time and history status stay visible. Current verdict requires
    # the Agent's separate native verify, which returns all original details.
    receipt = value.get('completion_receipt')
    if isinstance(receipt, dict):
        criteria = receipt.get('criteria')
        metrics = receipt.get('metrics')
        refs = receipt.get('inspection_references')
        ordinary_refs = isinstance(refs, list) and all(isinstance(r, dict)
            and set(r) == {'target', 'event_id', 'evidence_sha256'} for r in refs)
        safe_metrics = isinstance(metrics, dict) and set(metrics) <= METRICS and all(
            metrics.get(k) == 0 for k in ('collisions', 'boundary_violations', 'speed_violations'))
        if (receipt.get('criteria_met_at_completion') is True and isinstance(criteria, dict)
                and set(criteria) == RECEIPT_CRITERIA and all(v is True for v in criteria.values())
                and safe_metrics and ordinary_refs):
            for key in ('criteria', 'metrics', 'inspection_references'):
                receipt.pop(key)
            receipt['historical_details_in_local_journal'] = True
    return value


BRIEF_CONTRACT = (
    'Synthetic 2D Ackermann. task revision/order/permitted_routes are supervisor-owned. '
    'Missing/stale sensing is unknown. Agent chooses route, recovery, next goal and final verify. '
    'navigate chooses one target/route, plans, executes at most15s, brakes, inspects only ARRIVED+verified stop; '
    'failure/partial returns immediately. execute_route continues an existing handle with a NEW id; '
    'fixed15s and arrival inspection. Idempotent retries are historical. Resume/forecasts are conditional, '
    'never permission or sensing. All original checks apply. '
    'Recovery3 mission-wide: after failure each same-target plan/reobserve/wait consumes1. '
    'Deadline300s includes real tool-between waits after first plan; executions80, requests120; stop/verify exempt from request cap. '
    'Brief returns omit equal task duplicates, all-true resume check details, past arrival forecasts and passing historical receipt details; '
    'all full results remain logged. Unknown/negative fields and current budgets remain. '
    'Immediately call final verify after last inspection or termination; safe termination is not success.'
)

BRIEF_DESCRIPTIONS = {
    'observe': 'Read actual synthetic state, freshness, task and budgets; unknown is not free space.',
    'plan_route': 'Choose one legal route to task.next_target; checked handle and conditional forecast, no motion.',
    'navigate': 'Choose one next target/route; checked plan+at most15s motion+brake+inspection only ARRIVED/stopped. Failure/partial returns; Agent chooses recovery. NEW operation_id.',
    'execute_route': 'Continue chosen route_id with NEW operation_id. Fixed15s+brake+inspect only ARRIVED/stopped. Omit overrides; guards enforced. Retry identical id replays history.',
    'inspect': 'Synthetic measurement only at target and stopped; no invented evidence.',
    'reobserve': 'Stay stopped, spend time, reacquire sensing after missing/stale feedback; consumes recovery.',
    'wait': 'Stay stopped REQUIRED0<seconds<=5, advance environment then observe; consumes recovery.',
    'stop': 'Brake; terminate=true ends incomplete mission. Budgets unchanged.',
    'verify': 'Independent CURRENT full verdict/evidence. Call immediately after last inspection or termination.',
}
