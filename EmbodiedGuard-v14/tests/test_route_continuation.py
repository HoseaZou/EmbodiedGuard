"""Mission-level regressions from the first real Agnes inspection run."""
import pytest

from embodiedguard.execution import Mission


def reach_planned_target(mission, target, plan):
    assert plan['ok'], plan
    for index in range(4):
        result = mission.execute_route(route_id=plan['route_id'],
                                       operation_id=f'{target}-{index}', max_sim_seconds=15.)
        assert result['ok'], result
        if result['status'] == 'ARRIVED':
            break
    assert result['status'] == 'ARRIVED', result
    assert mission.inspect(target=target)['ok']


@pytest.mark.parametrize('first_route', ['upper', 'lower'])
@pytest.mark.parametrize('second_route', ['upper', 'lower'])
def test_detour_arrival_at_first_checkpoint_can_continue_inspection(first_route, second_route):
    mission = Mission('blocked')
    first = mission.plan_route(target='panel_a', route=first_route)
    assert first['ok'], first
    arrived = mission.execute_route(route_id=first['route_id'],
                                    operation_id='panel-detour', max_sim_seconds=15.)
    assert arrived['status'] == 'ARRIVED', arrived
    assert mission.inspect(target='panel_a')['ok']

    # Curvature alternatives do not turn a blocked corridor into a valid one.
    assert mission.plan_route(target='pump_b', route='direct')['reason'] == 'PATH_BLOCKED'
    # A valid upstream choice must not strand the rover at the next checkpoint.
    onward = mission.plan_route(target='pump_b', route=second_route)
    reach_planned_target(mission, 'pump_b', onward)

    final = mission.plan_route(target='exit_c', route='direct')
    reach_planned_target(mission, 'exit_c', final)
    report = mission.verify()
    assert report['task_success'], report
    assert report['metrics']['collisions'] == 0
    assert report['metrics']['boundary_violations'] == 0
    assert report['metrics']['speed_violations'] == 0
