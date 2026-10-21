import json

import pytest

from embodiedguard.experiments import prepare_agent_trials, run_experiments


def test_paired_blocked_runs_are_isolated_and_keep_both_outcomes(tmp_path):
    result = run_experiments(tmp_path / 'experiment', scenarios=('blocked',), repeats=2)
    assert result['status'] == 'complete'
    assert result['planned_trials'] == result['attempted_trials'] == 4
    assert len({row['trial_id'] for row in result['results']}) == 4
    for row in result['results']:
        assert row['status'] == 'recorded', row
        assert row['task_success'] == (row['policy'] == 'fixed_recovery')
        assert row['model_usage'] is row['model_cost'] is None
        assert row['model_call_verified'] is False
        assert row['encountered_failure'] is True
        assert 'PATH_BLOCKED' in row['failure_reasons']
        assert row['metrics']['collisions'] == 0
        directory = tmp_path / 'experiment' / row['evidence_directory']
        first_event = json.loads((directory / 'events.jsonl').read_text(encoding='utf-8').splitlines()[0])
        assert first_event['event'] == 'mission_started'
        assert first_event['frame_id'] == 0
        assert first_event['sim_time_sec'] == 0
        saved = json.loads((directory / 'summary.json').read_text(encoding='utf-8'))
        assert saved['task_success'] == row['task_success']
    failures = [row for row in result['results'] if not row['task_success']]
    assert all(row['safe_termination'] and row['termination_reason'] == 'PATH_BLOCKED' for row in failures)
    protocol = json.loads((tmp_path / 'experiment' / 'protocol.json').read_text(encoding='utf-8'))
    assert protocol['source']['files']['embodiedguard/sim.py']
    assert 'not independent samples' in protocol['design']['repeat_interpretation']
    prepared = json.loads((tmp_path / 'experiment' / 'agent-trials.json').read_text(encoding='utf-8'))
    assert prepared['status'] == 'prepared_not_executed'
    assert prepared['required_source_sha256'] == result['source_sha256']
    grouped = {row['policy']: row for row in result['aggregate']}
    assert grouped['no_recovery']['encountered_failure_then_task_success_rate'] == 0
    assert grouped['fixed_recovery']['encountered_failure_then_task_success_rate'] == 1


def test_synthetic_decision_delay_is_preserved_and_labelled(tmp_path):
    result = run_experiments(tmp_path, scenarios=('normal',), decision_delay_sec=0.1)
    assert result['status'] == 'complete'
    for row in result['results']:
        assert row['task_success']
        directory = tmp_path / row['evidence_directory']
        events = [json.loads(line) for line in (directory / 'events.jsonl').read_text(encoding='utf-8').splitlines()]
        delays = [event for event in events if event['event'] == 'decision_idle_elapsed']
        assert delays
        assert all(event['requested_wall_seconds'] == 0.1 for event in delays)
    protocol = json.loads((tmp_path / 'protocol.json').read_text(encoding='utf-8'))
    assert protocol['clock']['actual_agnes_latency'] is False
    assert protocol['clock']['synthetic_delay_sec'] == 0.1


def test_prepared_agent_trials_identify_seed_aliases_without_claiming_runs():
    prepared = prepare_agent_trials(scenarios=('blocked', 'dynamic'), seeds=(0, 1, 3), repeats=2)
    assert len(prepared['trials']) == 12
    assert all(row['status'] == 'not_run' and row['agh_session_id'] is None for row in prepared['trials'])
    assert {row['fixture_id'] for row in prepared['trials'] if row['scenario'] == 'blocked'} == {'blocked-fixed'}
    dynamic = {row['seed']: row['fixture_id'] for row in prepared['trials'] if row['scenario'] == 'dynamic'}
    assert dynamic[0] == dynamic[3]
    assert dynamic[1] != dynamic[0]
    assert prepared['model_calls_made'] is False
    assert prepared['clock_comparison']['matched_clock_claimed'] is False


def test_experiments_refuse_to_overwrite_prior_evidence(tmp_path):
    marker = tmp_path / 'prior.json'
    marker.write_text('{"original":true}', encoding='utf-8')
    with pytest.raises(FileExistsError):
        run_experiments(tmp_path, scenarios=('normal',))
    assert marker.read_text(encoding='utf-8') == '{"original":true}'
    assert list(tmp_path.iterdir()) == [marker]


@pytest.mark.parametrize('kwargs', [dict(repeats=0), dict(seeds=(True,)), dict(scenarios=('unknown',)),
                                   dict(segment_seconds=16), dict(decision_delay_sec=float('nan'))])
def test_invalid_protocol_does_not_create_evidence(tmp_path, kwargs):
    destination = tmp_path / 'invalid'
    with pytest.raises(ValueError):
        run_experiments(destination, **kwargs)
    assert not destination.exists()
