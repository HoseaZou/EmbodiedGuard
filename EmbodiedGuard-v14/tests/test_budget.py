from copy import deepcopy

import pytest

from embodiedguard.execution import Mission


def test_budget_reads_neither_advance_clock_nor_refresh_unknown_sensor():
    mission = Mission('stale', max_sim_seconds=10., max_operations=2)
    before = deepcopy(mission._store.__dict__)
    first = mission._budget_status()
    assert first['remaining_sim_sec'] == 10.
    assert not first['runtime_started']
    assert first['remaining_execution_operations'] == 2
    assert mission._feedback_state(readonly=True) is None
    first['remaining_sim_sec'] = 1000.
    assert mission._budget_status()['remaining_sim_sec'] == 10.
    assert mission._store.__dict__ == before
    assert mission._world.time_sec == 0.


def test_budget_counts_completed_operations_once_and_preserves_retry_snapshot():
    mission = Mission('normal', max_operations=2)
    plan = mission.plan_route(target='panel_a')
    result = mission.execute_route(route_id=plan['route_id'], operation_id='one', max_sim_seconds=1.)
    assert result['budget']['completed_execution_operations'] == 1
    assert result['budget']['remaining_execution_operations'] == 1
    mission.elapse_idle_wall(.125)
    current = mission._budget_status()
    assert current['remaining_sim_sec'] == pytest.approx(result['budget']['remaining_sim_sec'] - .125)
    retry = mission.execute_route(route_id=plan['route_id'], operation_id='one', max_sim_seconds=1.)
    assert retry == result  # Direct Mission retains an immutable historical operation result.
    assert mission._budget_status() == current
    conflict = mission.execute_route(route_id=plan['route_id'], operation_id='one', max_sim_seconds=2.)
    assert conflict['reason'] == 'OPERATION_CONFLICT'
    assert conflict['budget'] == current


def test_idle_budget_fractional_clamp_and_unavailable_feedback_without_pose():
    mission = Mission('stale', max_sim_seconds=4.323)
    mission.elapse_idle_wall(100.)  # Preparation time remains uncharged.
    assert mission.observe()['budget']['remaining_sim_sec'] == 4.323
    mission.plan_route(target='panel_a')
    mission.elapse_idle_wall(4.)
    assert mission.observe()['ok']  # First packet at the fault trigger is frozen.
    mission.elapse_idle_wall(.32)
    observed = mission.observe()
    assert not observed['ok']
    assert observed['state'] is None
    assert observed['budget']['remaining_sim_sec'] == pytest.approx(.003)
    mission.elapse_idle_wall(2.)
    result = mission.plan_route(target='panel_a')
    assert result['budget']['remaining_sim_sec'] == 0.
    assert result['budget']['sim_time_sec'] == pytest.approx(4.323)
    assert mission.verify()['safe_termination']
    assert not mission.verify()['task_success']


@pytest.mark.parametrize('method,args', [
    ('observe', {}), ('plan_route', {'target': 'pump_b'}),
    ('execute_route', {'route_id': 'missing', 'operation_id': 'one'}),
    ('inspect', {'target': 'missing'}), ('reobserve', {}),
    ('wait', {'seconds': 0}), ('stop', {}), ('verify', {}),
])
def test_success_and_error_feedback_contains_current_mission_limits(method, args):
    mission = Mission('normal', max_sim_seconds=37., max_operations=7)
    result = getattr(mission, method)(**args)
    assert result['budget'] == mission._budget_status()
    assert result['budget']['deadline_sim_sec'] == 37.
    assert result['budget']['max_execution_operations'] == 7
