import pytest

from embodiedguard.execution import Mission
from embodiedguard.mcp_server import MCPServer


def test_plan_exposes_conditional_travel_forecast_without_motion():
    mission = Mission('normal')
    plan = mission.plan_route(target='panel_a')
    estimate = plan['execution_estimate']
    assert estimate['method'] == 'isolated_kinematic_controller_rollout'
    assert estimate['estimated_arrival_and_stop_sec'] > plan['distance_m'] / plan['max_speed_mps']
    assert estimate['guaranteed'] is False
    assert estimate['post_arrival_reserve_sec'] == 5.
    assert mission._world.time_sec == 0
    assert mission._world.plant.state.x == 0
    assert mission._metrics['executions'] == 0


def test_live_budget_rejects_optimistic_last_leg_without_any_motion():
    mission = Mission('normal')
    plan = mission.plan_route(target='panel_a')
    mission.elapse_idle_wall(291.57)
    before = mission._world.plant.state
    result = mission.execute_route(route_id=plan['route_id'], operation_id='too-late', max_sim_seconds=15)
    assert result['reason'] == 'INSUFFICIENT_COMPLETION_BUDGET'
    assert result['budget']['remaining_sim_sec'] == pytest.approx(8.43)
    assert result['execution_estimate']['estimated_arrival_and_stop_sec'] > 8.43
    assert mission._world.plant.state == before
    assert result['stopping_verified']
    assert mission._metrics['executions'] == 0
    mission.stop(terminate=True)
    assert mission.verify()['terminated']
    assert mission.verify()['safe_termination']


def test_mcp_checks_budget_after_actual_decision_delay(monkeypatch):
    ticks = [0.]
    monkeypatch.setattr('embodiedguard.mcp_server.time.monotonic', lambda: ticks[0])
    mission = Mission('normal')
    server = MCPServer(mission, charge_idle_wall=True)
    server.handle(dict(jsonrpc='2.0', id=1, method='initialize', params={}))
    server.handle(dict(jsonrpc='2.0', method='notifications/initialized'))
    def call(name, args, ident):
        return server.handle(dict(jsonrpc='2.0', id=ident, method='tools/call',
                                 params=dict(name=name, arguments=args)))['result']['structuredContent']
    plan = call('plan_route', dict(target='panel_a'), 2)
    ticks[0] = 283.241
    # Return a plan based on a fresh sensor at this time; tools still own time.
    plan = call('plan_route', dict(target='panel_a'), 3)
    assert plan['budget']['remaining_sim_sec'] == pytest.approx(16.759)
    ticks[0] += 8.329
    result = call('execute_route', dict(route_id=plan['route_id'], operation_id='late', max_sim_seconds=15), 4)
    assert result['reason'] == 'INSUFFICIENT_COMPLETION_BUDGET'
    assert result['budget']['remaining_sim_sec'] == pytest.approx(8.43)
    assert mission._world.plant.state.x == 0


def test_short_segments_and_operation_retry_remain_supported():
    mission = Mission('normal')
    plan = mission.plan_route(target='panel_a')
    first = mission.execute_route(route_id=plan['route_id'], operation_id='one', max_sim_seconds=1)
    assert first['ok'] and first['state']['x'] > 0
    assert mission.execute_route(route_id=plan['route_id'], operation_id='one', max_sim_seconds=1) == first


def test_unavailable_forecast_never_invents_time_or_starts_motion(monkeypatch):
    mission = Mission('normal')
    plan = mission.plan_route(target='panel_a')
    def unavailable(*args, **kwargs):
        raise ValueError('predictor fault')
    monkeypatch.setattr('embodiedguard.timing._rollout', unavailable)
    result = mission.execute_route(route_id=plan['route_id'], operation_id='unknown', max_sim_seconds=15)
    assert result['reason'] == 'BUDGET_ESTIMATE_UNAVAILABLE'
    assert result['execution_estimate']['estimated_arrival_and_stop_sec'] is None
    assert mission._metrics['executions'] == 0
    assert mission._world.plant.state.x == 0
