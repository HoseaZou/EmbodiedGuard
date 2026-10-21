import pytest

from embodiedguard.execution import Mission


def reach(mission, target, route='direct'):
    plan = mission.plan_route(target=target, route=route)
    assert plan['ok'], plan
    result = None
    for i in range(10):
        result = mission.execute_route(route_id=plan['route_id'], operation_id=f'{target}-{route}-{i}', max_sim_seconds=8.)
        if result['status'] == 'ARRIVED' or not result['ok']:
            return result
    pytest.fail(f'route did not finish: {result}')


def test_normal_inspection_requires_arrival_stopping_and_measurements():
    mission = Mission('normal')
    assert mission.verify()['task_success'] is False
    assert not mission.inspect(target='panel_a')['ok']
    for target in ('panel_a', 'pump_b', 'exit_c'):
        assert reach(mission, target)['status'] == 'ARRIVED'
        assert mission.inspect(target=target)['ok']
    result = mission.verify()
    assert result['task_success'] is True
    assert result['metrics']['collisions'] == 0
    assert result['metrics']['boundary_violations'] == 0
    assert result['metrics']['speed_violations'] == 0
    assert result['final_state']['speed_mps'] <= .05


def test_blocked_aisle_requires_an_explicit_new_route_choice():
    mission = Mission('blocked')
    assert reach(mission, 'panel_a')['status'] == 'ARRIVED'
    mission.inspect(target='panel_a')
    blocked = mission.plan_route(target='pump_b', route='direct')
    assert blocked['reason'] == 'PATH_BLOCKED'
    assert mission.verify()['completed_targets'] == ['panel_a']
    assert reach(mission, 'pump_b', 'upper')['status'] == 'ARRIVED'
    assert mission.inspect(target='pump_b')['ok']
    assert mission.verify()['metrics']['collisions'] == 0


def test_repeated_operation_is_idempotent_and_changed_arguments_are_rejected():
    mission = Mission('normal')
    plan = mission.plan_route(target='panel_a', route='direct')
    first = mission.execute_route(route_id=plan['route_id'], operation_id='move-1', max_sim_seconds=2.)
    time_before = mission.verify()['sim_time_sec']
    again = mission.execute_route(route_id=plan['route_id'], operation_id='move-1', max_sim_seconds=2.)
    assert again == first
    assert mission.verify()['sim_time_sec'] == time_before
    changed = mission.execute_route(route_id=plan['route_id'], operation_id='move-1', max_sim_seconds=3.)
    assert changed['reason'] == 'OPERATION_CONFLICT'


def test_stale_state_stops_then_reobserve_allows_resumption():
    mission = Mission('stale')
    plan = mission.plan_route(target='panel_a', route='direct')
    result = mission.execute_route(route_id=plan['route_id'], operation_id='first', max_sim_seconds=8.)
    assert result['reason'] == 'STALE_OBSERVATION'
    assert result['state'] is None
    assert result['stopping_verified']
    assert not mission.verify()['task_success']
    assert mission.reobserve()['ok']
    result = mission.execute_route(route_id=plan['route_id'], operation_id='second', max_sim_seconds=8.)
    assert result['ok'], result


def test_safe_termination_is_not_task_success_and_cannot_resume():
    mission = Mission('unreachable')
    result = mission.stop(terminate=True, reason='all routes unavailable')
    assert result['stopping_verified']
    assert mission.plan_route(target='panel_a', route='direct')['reason'] == 'TERMINATED'
    report = mission.verify()
    assert report['safe_termination']
    assert not report['task_success']


def test_clock_driven_crossing_requires_wait_and_then_can_recover():
    from embodiedguard.cli import run_demo
    baseline = run_demo('dynamic', policy='no_recovery').verify()
    recovery = run_demo('dynamic', policy='fixed_recovery').verify()
    assert not baseline['task_success']
    assert baseline['termination_reason'] == 'PATH_BLOCKED'
    assert recovery['task_success']
    assert recovery['metrics']['collisions'] == 0
