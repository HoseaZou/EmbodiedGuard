"""Bounded-stop/resume behavior through the public Mission tool boundary."""

import pytest

from embodiedguard.execution import Mission


@pytest.mark.parametrize('first,second', [('upper', 'lower'), ('lower', 'upper')])
def test_four_second_segments_can_complete_opposite_side_detour(first, second):
    mission = Mission('blocked')
    for target, route in (('panel_a', first), ('pump_b', second), ('exit_c', 'direct')):
        plan = mission.plan_route(target=target, route=route)
        assert plan['ok'], plan
        for index in range(20):
            result = mission.execute_route(route_id=plan['route_id'],
                operation_id=f'{target}-{index}', max_sim_seconds=4.)
            assert result['ok'], result
            assert result['stopping_verified'], result
            if result['status'] == 'ARRIVED':
                break
        assert result['status'] == 'ARRIVED', result
        assert mission.inspect(target=target)['ok']
    report = mission.verify()
    assert report['task_success'], report
    for metric in ('collisions', 'boundary_violations', 'speed_violations'):
        assert report['metrics'][metric] == 0, report


@pytest.mark.parametrize('duration', [4., 8.])
@pytest.mark.parametrize('route', ['upper', 'lower'])
def test_same_side_segment_continuation_still_stops_before_unsafe_motion(duration, route):
    mission = Mission('blocked')
    first = mission.plan_route(target='panel_a', route=route)
    assert first['ok'], first
    for index in range(20):
        result = mission.execute_route(route_id=first['route_id'],
            operation_id=f'panel-{index}', max_sim_seconds=duration)
        assert result['ok'], result
        if result['status'] == 'ARRIVED':
            break
    assert result['status'] == 'ARRIVED', result
    assert mission.inspect(target='panel_a')['ok']
    onward = mission.plan_route(target='pump_b', route=route)
    assert onward['ok'], onward
    for index in range(20):
        result = mission.execute_route(route_id=onward['route_id'],
            operation_id=f'pump-{index}', max_sim_seconds=duration)
        if not result['ok']:
            break
    assert result['reason'] == 'PATH_BLOCKED', result
    assert result['stopping_verified'], result
    assert result['state']['speed_mps'] == 0., result
    report = mission.verify()
    assert not report['task_success'], report
    for metric in ('collisions', 'boundary_violations', 'speed_violations'):
        assert report['metrics'][metric] == 0, report
