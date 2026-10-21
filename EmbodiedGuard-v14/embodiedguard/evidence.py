"""Read recorded evidence; bind optional external audit without rewriting facts."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from .feedback import compact_feedback
from .efficiency import RevisionFeedback, PROFILES
from .decision_feedback import decision_feedback


def _json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def _rows(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line.strip()]


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _validate_completion_receipt(evidence):
    """Check local receipt consistency, not authorship or physical correctness.

    A historical completion evaluation must not silently replace a later final
    verdict. Older recordings without this additive contract remain readable.
    """
    def require(condition, detail):
        if not condition:
            raise ValueError(f'Completion receipt mismatch: {detail}')

    def number(value):
        return type(value) in (int, float) and math.isfinite(value)

    summary = evidence.get('summary', {})
    events = evidence.get('events', [])
    evaluations = [event for event in events if event.get('event') == 'mission_completion_evaluated']
    receipt = evidence.get('completion_receipt')
    if evidence.get('schema_version') == 3:
        require('completion_receipt' in evidence and 'completion_receipt' in summary,
                'schema 3 must explicitly record receipt or null')
    require(receipt == summary.get('completion_receipt'), 'run and summary differ')
    if receipt is None:
        require(not evaluations, 'completion event exists without its receipt')
        return
    require(isinstance(receipt, dict), 'receipt must be an object')
    require(type(receipt.get('schema_version')) is int and receipt['schema_version'] == 1
            and receipt.get('receipt_id') == 'completion-1',
            'unsupported receipt identity or schema')
    require(len(evaluations) == 1 and evaluations[0].get('receipt') == receipt,
            'exactly one matching completion event is required')
    event = evaluations[0]
    event_id, frame, timestamp = (receipt.get(k) for k in ('event_id', 'frame_id', 'sim_time_sec'))
    require(type(event_id) is int and event_id > 0 and type(frame) is int and frame >= 0,
            'invalid event or frame identifier')
    require(number(timestamp) and timestamp >= 0, 'invalid completion time')
    require(all(event.get(key) == receipt.get(key) for key in ('event_id', 'frame_id', 'sim_time_sec')),
            'event identity/time differs')
    deadline = receipt.get('deadline_sim_sec')
    require(number(deadline) and deadline > 0, 'invalid deadline')
    require(number(summary.get('sim_time_sec')) and timestamp <= summary['sim_time_sec'],
            'completion occurs after the final verdict')
    if 'budget' in summary:
        require(isinstance(summary['budget'], dict) and summary['budget'].get('deadline_sim_sec') == deadline,
                'deadline differs from final budget')
    expected_episode = evidence.get('episode_id', f"{evidence.get('scenario')}-{evidence.get('seed')}")
    require(receipt.get('episode_id') == expected_episode, 'episode differs')
    require(receipt.get('validation_scope') == summary.get('validation_scope') == 'synthetic_2d_kinematics_only',
            'unsupported or inconsistent validation scope')
    criteria = receipt.get('criteria')
    keys = {'all_targets_inspected', 'at_exit', 'stopped', 'not_terminated',
            'no_safety_violations', 'before_deadline'}
    require(isinstance(criteria, dict) and set(criteria) == keys
            and all(type(value) is bool for value in criteria.values()), 'invalid criteria')
    require(type(receipt.get('criteria_met_at_completion')) is bool
            and receipt['criteria_met_at_completion'] == all(criteria.values()), 'criteria verdict differs')
    require(criteria['before_deadline'] == (timestamp < deadline), 'deadline criterion differs')
    metrics = receipt.get('metrics')
    safety_keys = ('collisions', 'boundary_violations', 'speed_violations')
    require(isinstance(metrics, dict) and all(type(metrics.get(k)) is int and metrics[k] >= 0
                                            for k in safety_keys), 'invalid safety counters')
    require(criteria['no_safety_violations'] == all(metrics[k] == 0 for k in safety_keys),
            'safety criterion differs')
    references = receipt.get('inspection_references')
    checkpoints = evidence.get('checkpoints', {})
    require(isinstance(checkpoints, dict) and bool(checkpoints) and isinstance(references, list)
            and all(isinstance(ref, dict) for ref in references), 'invalid inspection references')
    require(all(isinstance(ref.get('target'), str) for ref in references), 'invalid reference target')
    targets = [ref['target'] for ref in references]
    require(len(targets) == len(set(targets)) and set(targets) == set(checkpoints)
            and criteria['all_targets_inspected'], 'references must cover each checkpoint exactly once')
    by_id = {}
    previous_id = 0
    for recorded in events:
        identifier = recorded.get('event_id')
        require(type(identifier) is int and identifier > previous_id, 'unordered or invalid event identifier')
        by_id[identifier] = recorded
        previous_id = identifier
    inspections = []
    for reference in references:
        identifier = reference.get('event_id')
        require(type(identifier) is int and identifier in by_id and identifier < event_id,
                'reference must point to an earlier event')
        inspected = by_id[identifier]
        measurement = inspected.get('evidence')
        require(inspected.get('event') == 'inspection_verified' and isinstance(measurement, dict)
                and measurement.get('target') == reference['target'], 'reference target/event differs')
        require(number(inspected.get('sim_time_sec')) and inspected['sim_time_sec'] <= timestamp
                and type(inspected.get('frame_id')) is int and inspected['frame_id'] <= frame,
                'inspection occurs after completion')
        require(measurement.get('frame_id') == inspected['frame_id']
                and measurement.get('observed_at') == inspected['sim_time_sec'], 'inspection observation differs')
        try:
            digest = hashlib.sha256(json.dumps(measurement, sort_keys=True, separators=(',', ':'),
                                    ensure_ascii=False, allow_nan=False).encode('utf-8')).hexdigest()
        except (ValueError, TypeError) as exc:
            raise ValueError('Completion receipt mismatch: invalid inspection JSON') from exc
        require(reference.get('evidence_sha256') == digest, 'inspection evidence hash differs')
        inspections.append(inspected)
    last = max(inspections, key=lambda item: item['event_id'])
    require(last['sim_time_sec'] == timestamp and last['frame_id'] == frame,
            'completion must be captured with the last inspection')
    pose = last['evidence'].get('pose')
    exit_point = checkpoints.get('exit_c')
    require(isinstance(pose, dict) and all(number(pose.get(k)) for k in ('x', 'y', 'speed_mps'))
            and isinstance(exit_point, (tuple, list)) and len(exit_point) == 2
            and all(number(value) for value in exit_point), 'missing completion pose/exit evidence')
    require(criteria['at_exit'] == (math.dist((pose['x'], pose['y']), exit_point) <= .6)
            and criteria['stopped'] == (pose['speed_mps'] <= .05), 'pose criteria differ')


def load_run(directory: Path, *, verification_directory: Path | None = None) -> dict:
    """Load an existing run, optionally recheck a locally reviewed AGH evidence bundle.

    The external annotation establishes agreement of local records, not provider
    authentication or billing. It never promotes the simulator's provenance flag.
    """
    directory = Path(directory)
    evidence = _json(directory / 'run.json')
    # Only this loader can attach a checked annotation; discard self-declarations.
    evidence.pop('external_verification', None)
    for name in ('events', 'trace'):
        evidence[name] = _rows(directory / f'{name}.jsonl')
    plan_file = directory / 'plans.jsonl'
    evidence['plans'] = _rows(plan_file) if plan_file.is_file() else []
    evidence['plan_evidence_available'] = plan_file.is_file()
    _validate_completion_receipt(evidence)
    if verification_directory is None:
        return evidence

    review = Path(verification_directory)
    check_path = review / 'exact-cross-check.json'
    check = _json(check_path)
    if check.get('passed') is not True:
        raise ValueError('External cross-check did not pass')
    manifest = _json(review / 'evidence-manifest.json')['files']
    for name in ('exact-cross-check.json', 'tool-evidence.exact.jsonl', 'session.redacted.jsonl'):
        if manifest.get(name, {}).get('sha256') != _sha(review / name):
            raise ValueError(f'External audit file hash mismatch: {name}')
    names = ['run.json', 'summary.json', 'events.jsonl', 'trace.jsonl']
    if plan_file.is_file():
        names.append('plans.jsonl')
    hashes = {}
    for name in names:
        digest = _sha(directory / name)
        if manifest.get(f'frozen-run/{name}', {}).get('sha256') != digest:
            raise ValueError(f'External evidence hash mismatch: {name}')
        hashes[name] = digest
    if _json(directory / 'summary.json') != evidence['summary']:
        raise ValueError('Saved summary does not match run summary')

    projection = _rows(review / 'tool-evidence.exact.jsonl')
    calls = [r for r in projection if r['type'] == 'tool/call']
    results = {r['toolUseId']: r for r in projection if r['type'] == 'tool/result'}
    starts = [r for r in evidence['events'] if r['event'] == 'tool_call_started']
    finishes = {r['tool_call_id']: r for r in evidence['events'] if r['event'] == 'tool_call_finished'}
    if not calls or len(calls) != len(starts) or len(results) != len(calls) or len(finishes) != len(starts):
        raise ValueError('External tool record counts do not match')
    if len({r['toolUseId'] for r in calls}) != len(calls):
        raise ValueError('Duplicate external tool call identifiers')
    ledger = _rows(review / 'session.redacted.jsonl')
    ledger_by_seq = {r['seq']: r for r in ledger}
    lower, upper = check['ledger_range']
    verification_results = []
    revision_feedback = RevisionFeedback()
    for call, started in zip(calls, starts):
        result = results[call['toolUseId']]
        finished = finishes[started['tool_call_id']]
        # Native clients receive model_result, while local result retains full
        # supervisor evidence. Recompute the exact projection; neither channel
        # may be used to invent fields or hide a changed completion verdict.
        mode = finished.get('response_mode', 'full')
        if mode not in ('full', 'compact'):
            raise ValueError('Unsupported recorded response mode')
        expected_model_result = compact_feedback(finished['result']) if mode == 'compact' else finished['result']
        profile = finished.get('interface_profile', 'baseline')
        if profile not in PROFILES:
            raise ValueError('Unsupported recorded interface profile')
        if profile in ('lean','focused') and mode == 'compact':
            expected_model_result = revision_feedback.compact(expected_model_result, tool=finished['tool'])
            if profile == 'focused':
                expected_model_result = decision_feedback(expected_model_result, tool=finished['tool'])
        recorded_model_result = finished.get('model_result', finished['result'])
        if recorded_model_result != expected_model_result:
            raise ValueError('Recorded model results do not match full result projection')
        if not (lower <= call['seq'] < result['seq'] <= upper
                and call['origin'] == 'model'
                and call['tool'] == started['tool'] == finished['tool']
                and call['args'] == started['arguments']
                and result['result'] == recorded_model_result
                and call['seq'] in result['sourceEventSeqs']):
            raise ValueError('External tool arguments or results do not match recording')
        ledger_call = ledger_by_seq[call['seq']]
        ledger_result = ledger_by_seq[result['seq']]
        if not (ledger_call['type'] == 'tool/call' and ledger_call.get('origin') == 'model'
                and ledger_call['data']['toolUseId'] == call['toolUseId']
                and ledger_call['data']['name'].startswith('mcp_embodiedguard_')
                and ledger_call['data']['name'].endswith('_' + call['tool'])
                and ledger_call['data']['args'] == call['args']
                and ledger_result['type'] == 'tool/result'
                and ledger_result['data']['toolUseId'] == call['toolUseId']
                and ledger_result['data']['isError'] == result['isError']
                and call['seq'] in ledger_result['sourceEventSeqs']):
            raise ValueError('Projection does not match AGH ledger call links')
        if call['tool'] == 'verify':
            verification_results.append(finished['result'])
    if not verification_results or verification_results[-1] != evidence['summary']:
        raise ValueError('Final external verify does not match saved summary')
    round_events = [r for r in ledger if lower <= r['seq'] <= upper]
    models = sorted({r['data']['model']['id'] for r in round_events if r['type'] == 'request/sent'})
    if not models:
        raise ValueError('AGH ledger has no model request identity')
    costs = [r['data'] for r in round_events if r['type'] == 'cost/ledger' and r['data'].get('purpose') == 'inference']
    evidence['external_verification'] = dict(
        status='matched', model=', '.join(models), tool_pairs=len(calls),
        report_path=str(check_path.resolve()), run_sha256=hashes['run.json'], cross_check_sha256=_sha(check_path),
        evidence_bundle_sha256=hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest(),
        ledger_sha256=_sha(review / 'session.redacted.jsonl'),
        projection_sha256=_sha(review / 'tool-evidence.exact.jsonl'), ledger_range=[lower, upper],
        inference_requests=len(costs) if costs else None,
        usage={k: sum(c.get('tokens', {}).get(k, 0) for c in costs)
               for k in ('input', 'output', 'cacheRead', 'cacheWrite')} if costs else None,
        model_duration_ms=sum(c.get('timing', {}).get('durationMs', 0) for c in costs) if costs else None,
        billed_cost=None, cost_note='Actual billed cost is unavailable; estimated zeros are not a bill.',
        scope='Local AGH and simulator records match; not runtime authentication, independence or real-world validation.',
    )
    return evidence
