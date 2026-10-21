import hashlib
import json
from copy import deepcopy

import pytest

from embodiedguard.evidence import load_run
from embodiedguard.cli import run_demo


def write_json(path, value):
    path.write_text(json.dumps(value), encoding='utf-8')


def make_recording(tmp_path):
    verification = tmp_path / 'review'
    run = verification / 'frozen-run'
    run.mkdir(parents=True)
    summary = dict(task_success=True, model_call_verified=False, model_cost=None)
    write_json(run / 'run.json', {'summary': summary, 'scenario': 'normal'})
    write_json(run / 'summary.json', summary)
    events = [dict(event='tool_call_started', event_id=1, tool_call_id='rpc-1', tool='verify', arguments={}),
              dict(event='tool_call_finished', event_id=2, tool_call_id='rpc-1', tool='verify', result=summary)]
    (run / 'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in events), encoding='utf-8')
    (run / 'trace.jsonl').write_text('', encoding='utf-8')
    projection = [dict(seq=12, type='tool/call', toolUseId='use-1', tool='verify', args={}, origin='model'),
                  dict(seq=14, type='tool/result', toolUseId='use-1', result=summary, isError=False, sourceEventSeqs=[12])]
    (verification / 'tool-evidence.exact.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in projection), encoding='utf-8')
    ledger = [dict(seq=11, type='request/sent', data={'model': {'id': 'agnes-test'}}),
              dict(seq=12, type='tool/call', origin='model', data=dict(toolUseId='use-1', name='mcp_embodiedguard_5a0a503e_verify', args={})),
              dict(seq=14, type='tool/result', data=dict(toolUseId='use-1', isError=False), sourceEventSeqs=[12])]
    (verification / 'session.redacted.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in ledger), encoding='utf-8')
    write_json(verification / 'exact-cross-check.json', dict(passed=True, ledger_range=[10, 15]))
    files = {f'frozen-run/{p.name}': {'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in run.iterdir()}
    files.update({name: {'sha256': hashlib.sha256((verification / name).read_bytes()).hexdigest()}
                  for name in ('exact-cross-check.json', 'tool-evidence.exact.jsonl', 'session.redacted.jsonl')})
    write_json(verification / 'evidence-manifest.json', {'files': files})
    return run, verification


def test_historical_recording_without_plans_stays_honest(tmp_path):
    run, _ = make_recording(tmp_path)
    evidence = load_run(run)
    assert evidence['plans'] == []
    assert evidence['plan_evidence_available'] is False
    assert 'external_verification' not in evidence
    assert evidence['summary']['model_call_verified'] is False


def test_external_annotation_requires_bound_files_and_matching_tool_results(tmp_path):
    run, verification = make_recording(tmp_path)
    evidence = load_run(run, verification_directory=verification)
    assert evidence['external_verification']['status'] == 'matched'
    assert evidence['external_verification']['model'] == 'agnes-test'
    assert evidence['external_verification']['tool_pairs'] == 1
    assert evidence['summary']['model_call_verified'] is False
    assert evidence['summary']['model_cost'] is None
    (run / 'trace.jsonl').write_text('{}\n', encoding='utf-8')
    with pytest.raises(ValueError, match='hash'):
        load_run(run, verification_directory=verification)


def test_passed_flag_cannot_override_a_mismatched_tool_projection(tmp_path):
    run, verification = make_recording(tmp_path)
    path = verification / 'tool-evidence.exact.jsonl'
    projection = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
    projection[1]['result']['task_success'] = False
    path.write_text(''.join(json.dumps(row) + '\n' for row in projection), encoding='utf-8')
    manifest = json.loads((verification / 'evidence-manifest.json').read_text(encoding='utf-8'))
    manifest['files'][path.name]['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    write_json(verification / 'evidence-manifest.json', manifest)
    with pytest.raises(ValueError, match='results do not match'):
        load_run(run, verification_directory=verification)


def test_changed_model_identity_in_a_previously_reviewed_ledger_is_rejected(tmp_path):
    run, verification = make_recording(tmp_path)
    path = verification / 'session.redacted.jsonl'
    path.write_text(path.read_text(encoding='utf-8').replace('agnes-test', 'unreviewed-model'), encoding='utf-8')
    with pytest.raises(ValueError, match='hash'):
        load_run(run, verification_directory=verification)


@pytest.fixture(scope='module')
def completed_recording():
    """Use an actual deterministic run, never a claimed Agnes/model result."""
    mission = run_demo('normal', policy='fixed_recovery')
    assert mission.verify()['task_success']
    return mission.evidence()


def save_recording(path, evidence):
    evidence = deepcopy(evidence)
    path.mkdir(parents=True, exist_ok=True)
    for key in ('events', 'trace', 'plans'):
        (path / f'{key}.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in evidence.pop(key)),
                                         encoding='utf-8')
    write_json(path / 'run.json', evidence)
    write_json(path / 'summary.json', evidence['summary'])


def update_receipt(evidence, mutate):
    """Keep redundant copies equal so tests reach substantive validation."""
    receipt = deepcopy(evidence['completion_receipt'])
    mutate(receipt)
    evidence['completion_receipt'] = deepcopy(receipt)
    evidence['summary']['completion_receipt'] = deepcopy(receipt)
    event = next(row for row in evidence['events'] if row['event'] == 'mission_completion_evaluated')
    event['receipt'] = deepcopy(receipt)


def test_completion_receipt_round_trip_is_historical_not_final_success(tmp_path):
    mission = run_demo('normal', policy='fixed_recovery')
    before = mission.verify()['completion_receipt']
    assert before['criteria_met_at_completion'] is True
    mission.elapse_idle_wall(300.)
    mission.save(tmp_path)
    loaded = load_run(tmp_path)
    assert loaded['summary']['task_success'] is False
    assert loaded['summary']['termination_reason'] == 'TIME_BUDGET_EXHAUSTED'
    assert loaded['summary']['safe_termination'] is True
    assert loaded['completion_receipt'] == before
    assert loaded['summary']['completion_receipt'] == before
    assert 'external_verification' not in loaded


def test_fresh_recording_with_explicit_null_receipt_remains_readable(tmp_path):
    from embodiedguard.execution import Mission
    Mission('normal').save(tmp_path)
    loaded = load_run(tmp_path)
    assert loaded['completion_receipt'] is None
    assert loaded['summary']['completion_receipt'] is None


@pytest.mark.parametrize('mutate', [
    lambda r: r.update(criteria_met_at_completion=False),
    lambda r: r['criteria'].update(before_deadline=False),
    lambda r: r['criteria'].update(stopped=False),
    lambda r: r['criteria'].update(at_exit=False),
    lambda r: r['criteria'].update(not_terminated='true'),
    lambda r: r['metrics'].update(collisions=1),
    lambda r: r['inspection_references'][0].update(evidence_sha256='0' * 64),
    lambda r: r['inspection_references'][0].update(event_id=r['event_id']),
    lambda r: r['inspection_references'][0].update(target='exit_c'),
    lambda r: r.update(episode_id='other-episode'),
    lambda r: r.update(sim_time_sec=float('nan')),
    lambda r: r.update(sim_time_sec=True),
])
def test_matching_receipt_copies_cannot_hide_invalid_evidence(tmp_path, completed_recording, mutate):
    recording = deepcopy(completed_recording)
    update_receipt(recording, mutate)
    save_recording(tmp_path, recording)
    with pytest.raises(ValueError, match='Completion receipt mismatch'):
        load_run(tmp_path)


def test_changed_inspection_payload_is_rejected_even_with_matching_receipt_copies(tmp_path, completed_recording):
    recording = deepcopy(completed_recording)
    inspected = next(row for row in recording['events'] if row['event'] == 'inspection_verified')
    inspected['evidence']['measured_temperature_c'] = 999.
    save_recording(tmp_path, recording)
    with pytest.raises(ValueError, match='inspection evidence hash differs'):
        load_run(tmp_path)


@pytest.mark.parametrize('damage', ['remove_event', 'duplicate_event', 'remove_receipt', 'alter_summary'])
def test_completion_receipt_requires_one_consistent_event(tmp_path, completed_recording, damage):
    recording = deepcopy(completed_recording)
    event = next(row for row in recording['events'] if row['event'] == 'mission_completion_evaluated')
    if damage == 'remove_event':
        recording['events'].remove(event)
    elif damage == 'duplicate_event':
        recording['events'].append(deepcopy(event))
    elif damage == 'remove_receipt':
        recording.pop('completion_receipt')
        recording['summary'].pop('completion_receipt')
    else:
        recording['summary']['completion_receipt']['criteria_met_at_completion'] = False
    save_recording(tmp_path, recording)
    with pytest.raises(ValueError, match='Completion receipt mismatch'):
        load_run(tmp_path)
