"""Completion receipts record historical evidence, never override live safety.

Normal completion is exercised through the real planner/controller. Private
state changes below are explicitly fault-injection unit fixtures, not measured
robot experiments or model trials.
"""

from copy import deepcopy
from dataclasses import replace
import hashlib
import json

import pytest

from embodiedguard.evidence import load_run
from embodiedguard.execution import Mission
from embodiedguard.sim import STOP


def _reach(mission, target):
    plan = mission.plan_route(target=target, route='direct')
    assert plan['ok'], plan
    for index in range(10):
        result = mission.execute_route(
            route_id=plan['route_id'], operation_id=f'{target}-{index}',
            max_sim_seconds=15.,
        )
        assert result['ok'], result
        if result['status'] == 'ARRIVED':
            return
    pytest.fail(f'{target} did not arrive: {result}')


@pytest.fixture(scope='module')
def final_arrival_template():
    mission = Mission('normal')
    for target in ('panel_a', 'pump_b'):
        _reach(mission, target)
        assert mission.inspect(target=target)['ok']
    _reach(mission, 'exit_c')
    assert mission.verify()['missing_targets'] == ['exit_c']
    return mission


@pytest.fixture
def final_arrival(final_arrival_template):
    return deepcopy(final_arrival_template)


def _completion_events(mission):
    return [event for event in mission.evidence()['events']
            if event['event'] == 'mission_completion_evaluated']


def _assert_no_receipt(mission, result=None):
    if result is not None:
        assert result.get('completion_receipt') is None
    assert mission.verify().get('completion_receipt') is None
    assert mission.evidence().get('completion_receipt') is None
    assert _completion_events(mission) == []


def test_completion_receipt_uses_actual_inspection_events_in_same_call(final_arrival):
    mission = final_arrival
    before_time = mission._world.time_sec
    before_frame = mission._world.frame_id
    result = mission.inspect(target='exit_c')
    assert result['ok']
    receipt = result['completion_receipt']
    assert receipt['schema_version'] == 1
    assert receipt['receipt_id'] == 'completion-1'
    assert receipt['episode_id'] == 'normal-0'
    assert receipt['frame_id'] == before_frame == mission._world.frame_id
    assert receipt['sim_time_sec'] == before_time == mission._world.time_sec
    assert receipt['deadline_sim_sec'] == 300.
    assert receipt['criteria_met_at_completion'] is True
    assert receipt['criteria'] == {
        'all_targets_inspected': True, 'at_exit': True, 'stopped': True,
        'not_terminated': True, 'no_safety_violations': True,
        'before_deadline': True,
    }
    assert receipt['validation_scope'] == 'synthetic_2d_kinematics_only'
    assert 'task_success' not in result
    assert mission.verify()['task_success'] is True
    assert receipt['metrics'] == mission.verify()['metrics']

    events = {event['event_id']: event for event in mission.evidence()['events']}
    completion_event = events[receipt['event_id']]
    assert completion_event['event'] == 'mission_completion_evaluated'
    assert completion_event['frame_id'] == before_frame
    assert completion_event['sim_time_sec'] == before_time
    assert [reference['target'] for reference in receipt['inspection_references']] == [
        'panel_a', 'pump_b', 'exit_c',
    ]
    for reference in receipt['inspection_references']:
        event = events[reference['event_id']]
        assert event['event'] == 'inspection_verified'
        assert event['event_id'] < completion_event['event_id']
        assert event['evidence']['target'] == reference['target']
        encoded = json.dumps(event['evidence'], sort_keys=True, separators=(',', ':'),
                             ensure_ascii=False, allow_nan=False).encode('utf-8')
        assert reference['evidence_sha256'] == hashlib.sha256(encoded).hexdigest()


def test_incomplete_mission_has_no_completion_receipt():
    mission = Mission('normal')
    _reach(mission, 'panel_a')
    result = mission.inspect(target='panel_a')
    assert result['ok']
    assert result['remaining_targets'] == ['pump_b', 'exit_c']
    _assert_no_receipt(mission, result)


@pytest.mark.parametrize('fault', ['wrong_pose', 'moving', 'stale', 'frame_mismatch',
                                  'unknown_obstacles', 'missing_obstacles', 'terminated'])
def test_invalid_final_inspection_cannot_create_receipt(final_arrival, monkeypatch, fault):
    mission = final_arrival
    if fault == 'terminated':
        mission.stop(terminate=True, reason='injected_test_termination')
    elif fault in ('stale', 'frame_mismatch'):
        frozen_packet = mission._world.packet()
        # A stale or previous-frame packet must not borrow evaluator pose.
        for _ in range(7 if fault == 'stale' else 1):
            mission._advance(STOP)
        monkeypatch.setattr(mission._world, 'packet', lambda: deepcopy(frozen_packet))
    else:
        mission._advance(STOP)
        if fault == 'wrong_pose':
            mission._world.plant._state = replace(mission._world.plant.state, x=34.)
        elif fault == 'moving':
            mission._world.plant._state = replace(mission._world.plant.state, speed_mps=.2)
        else:
            packet = mission._world.packet()
            if fault == 'unknown_obstacles':
                packet['obstacles_valid'] = False
            else:
                del packet['obstacles']
            monkeypatch.setattr(mission._world, 'packet', lambda: deepcopy(packet))
    result = mission.inspect(target='exit_c')
    assert result['ok'] is False
    assert 'exit_c' not in mission.verify()['completed_targets']
    assert mission.verify()['task_success'] is False
    _assert_no_receipt(mission, result)


@pytest.mark.parametrize('metric', ['collisions', 'boundary_violations', 'speed_violations'])
def test_recorded_safety_violation_prevents_successful_receipt(final_arrival, metric):
    mission = final_arrival
    # Inject evaluator history; normal execution would also terminate on a violation.
    mission._metrics[metric] = 1
    result = mission.inspect(target='exit_c')
    assert result['ok']
    receipt = result['completion_receipt']
    assert receipt['criteria_met_at_completion'] is False
    assert receipt['criteria']['no_safety_violations'] is False
    assert receipt['metrics'][metric] == 1
    assert mission.verify()['task_success'] is False


@pytest.mark.parametrize('remaining', [1e-6, 0., -1e-6])
def test_final_inspection_must_precede_deadline_strictly(final_arrival, remaining):
    mission = final_arrival
    # Deadline injection isolates the boundary without claiming a timed mission trial.
    mission._max_sim_seconds = mission._world.time_sec + remaining
    result = mission.inspect(target='exit_c')
    if remaining > 0:
        assert result['ok']
        assert result['completion_receipt']['criteria_met_at_completion'] is True
        assert result['completion_receipt']['criteria']['before_deadline'] is True
        assert mission.verify()['task_success'] is True
    else:
        assert not result['ok']
        assert result['reason'] == 'TIME_BUDGET_EXHAUSTED'
        assert mission.verify()['task_success'] is False
        _assert_no_receipt(mission, result)


def test_completion_receipt_survives_later_idle_timeout_without_changing_verdict(final_arrival):
    mission = final_arrival
    # Reproduce the measured trial's timing shape, not its actual model transcript.
    mission.elapse_idle_wall(258.676 - mission._world.time_sec)
    receipt = deepcopy(mission.inspect(target='exit_c')['completion_receipt'])
    assert receipt['sim_time_sec'] == pytest.approx(258.676)
    assert receipt['criteria_met_at_completion'] is True
    assert mission.verify()['task_success'] is True
    mission.elapse_idle_wall(49.828)
    result = mission.verify()
    assert result['sim_time_sec'] == 300.
    assert result['task_success'] is False
    assert result['safe_termination'] is True
    assert result['termination_reason'] == 'TIME_BUDGET_EXHAUSTED'
    assert result['completion_receipt'] == receipt
    assert mission.evidence()['completion_receipt'] == receipt
    assert len(_completion_events(mission)) == 1


def test_receipt_is_deeply_immutable_across_returns_and_inspection_retries(final_arrival):
    mission = final_arrival
    result = mission.inspect(target='exit_c')
    original = deepcopy(result['completion_receipt'])
    result['completion_receipt']['criteria']['at_exit'] = False
    result['completion_receipt']['inspection_references'][0]['evidence_sha256'] = 'rewritten'
    result['completion_receipt']['metrics']['collisions'] = 99
    assert mission.verify()['completion_receipt'] == original

    mission.elapse_idle_wall(.2)
    retried = mission.inspect(target='exit_c')
    assert retried['ok']
    assert retried['completion_receipt'] == original
    assert mission._world.time_sec > original['sim_time_sec']
    returned_verdict = mission.verify()
    returned_verdict['completion_receipt']['criteria_met_at_completion'] = False
    returned_evidence = mission.evidence()
    returned_evidence['completion_receipt']['metrics']['collisions'] = 999
    assert mission.verify()['completion_receipt'] == original
    assert mission.evidence()['completion_receipt'] == original
    assert len(_completion_events(mission)) == 1


def test_pure_verdict_evaluation_never_enforces_budget_or_refreshes_sensor(final_arrival):
    mission = final_arrival
    mission.inspect(target='exit_c')
    mission._advance(STOP)
    mission._max_sim_seconds = mission._world.time_sec
    snapshot = deepcopy({
        'events': mission._events, 'trace': mission._trace,
        'inspections': mission._inspections, 'metrics': mission._metrics,
        'sensor_records': mission._store._records,
    })
    before_clock = mission._store._clock
    before_time, before_frame = mission._world.time_sec, mission._world.frame_id
    before_termination = (mission._terminated, mission._termination_reason)
    assert before_clock < before_time
    mission._evaluate_verdict()
    assert mission._store._clock == before_clock
    assert mission._world.time_sec == before_time
    assert mission._world.frame_id == before_frame
    assert (mission._terminated, mission._termination_reason) == before_termination
    assert mission._events == snapshot['events']
    assert mission._trace == snapshot['trace']
    assert mission._inspections == snapshot['inspections']
    assert mission._metrics == snapshot['metrics']
    assert mission._store._records == snapshot['sensor_records']
    # Public verify retains mandatory deadline enforcement.
    assert mission.verify()['termination_reason'] == 'TIME_BUDGET_EXHAUSTED'
    assert mission.verify()['task_success'] is False


def test_saved_run_preserves_historical_receipt_alongside_current_failure(final_arrival, tmp_path):
    mission = final_arrival
    receipt = deepcopy(mission.inspect(target='exit_c')['completion_receipt'])
    mission.stop(terminate=True, reason='explicit_stop_after_completion')
    mission.save(tmp_path)
    run = json.loads((tmp_path / 'run.json').read_text(encoding='utf-8'))
    summary = json.loads((tmp_path / 'summary.json').read_text(encoding='utf-8'))
    assert run['completion_receipt'] == receipt
    assert summary['completion_receipt'] == receipt
    assert run['summary'] == summary
    assert summary['task_success'] is False
    assert summary['safe_termination'] is True
    loaded = load_run(tmp_path)
    assert loaded['completion_receipt'] == receipt
    assert loaded['summary']['task_success'] is False
    assert loaded['summary']['completion_receipt']['criteria_met_at_completion'] is True
