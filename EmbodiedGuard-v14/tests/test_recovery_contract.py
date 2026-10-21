"""Public MCP/mission behavior, with real planning and simulation."""
from embodiedguard.execution import Mission
from embodiedguard.mcp_server import MCPServer


def connected(scenario='normal'):
    server = MCPServer(Mission(scenario))
    server.handle(dict(jsonrpc='2.0', id=1, method='initialize',
        params=dict(protocolVersion='2025-11-25')))
    server.handle(dict(jsonrpc='2.0', method='notifications/initialized'))
    return server


def request(server, name, **arguments):
    return server.handle(dict(jsonrpc='2.0', id=2, method='tools/call',
        params=dict(name=name, arguments=arguments)))


def call(server, name, **arguments):
    return request(server, name, **arguments)['result']['structuredContent']


def test_missing_execution_duration_is_rejected_before_motion():
    server = connected()
    plan = call(server, 'plan_route', target='panel_a', route='direct')
    response = request(server, 'execute_route', route_id=plan['route_id'], operation_id='missing-duration')
    assert response['error']['code'] == -32602
    observed = call(server, 'observe')['observation']
    assert observed['pose'] == dict(x=0., y=0., yaw_rad=0.)
    assert observed['frame_id'] == 0
    final = call(server, 'verify')
    assert final['metrics']['executions'] == 0
    assert final['sim_time_sec'] == 0.


def reach_panel(server):
    plan = call(server, 'plan_route', target='panel_a', route='direct')
    arrived = call(server, 'execute_route', route_id=plan['route_id'],
        operation_id='panel-a', max_sim_seconds=15)
    assert arrived['status'] == 'ARRIVED'
    assert call(server, 'inspect', target='panel_a')['ok']


def test_route_failure_feedback_exposes_untried_candidates_and_recovery_budget():
    server = connected('unreachable')
    reach_panel(server)
    rejected = call(server, 'plan_route', target='pump_b', route='direct')
    assert rejected['reason'] == 'PATH_BLOCKED'
    recovery = rejected['recovery']
    assert recovery['next_target'] == 'pump_b'
    assert recovery['untried_routes'] == ['upper', 'lower']
    assert recovery['consumed_actions'] == 0
    assert recovery['remaining_actions'] == 3
    assert recovery['current_observation']['valid'] is True
    assert recovery['decision_hint'] == 'compare_untried_routes'
    upper = call(server, 'plan_route', target='pump_b', route='upper')
    assert upper['recovery']['remaining_actions'] == 2
    assert upper['recovery']['untried_routes'] == ['lower']


def test_exhausted_recovery_cannot_wait_or_replan_but_can_stop_and_verify():
    server = connected('unreachable')
    reach_panel(server)
    for route in ('direct', 'upper', 'lower'):
        assert call(server, 'plan_route', target='pump_b', route=route)['reason'] == 'PATH_BLOCKED'
    assert call(server, 'reobserve')['recovery']['remaining_actions'] == 0
    before = call(server, 'verify')['sim_time_sec']
    for tool, args in [('wait', dict(seconds=5)), ('reobserve', {}),
                       ('plan_route', dict(target='pump_b', route='direct'))]:
        denied = call(server, tool, **args)
        assert denied['reason'] == 'RECOVERY_BUDGET_EXHAUSTED'
        assert denied['stopping_verified'] is True
        assert denied['recovery']['consumed_actions'] == 3
        assert call(server, 'verify')['sim_time_sec'] == before
    stopped = call(server, 'stop', terminate=True, reason='provided candidates exhausted')
    assert stopped['status'] == 'TERMINATED'
    final = call(server, 'verify')
    assert final['safe_termination'] is True
    assert final['task_success'] is False


def test_stale_feedback_requests_observation_then_allows_same_route_resumption():
    server = connected('stale')
    plan = call(server, 'plan_route', target='panel_a', route='direct')
    arguments = dict(route_id=plan['route_id'], operation_id='stale-first', max_sim_seconds=15)
    failed = call(server, 'execute_route', **arguments)
    assert failed['reason'] == 'STALE_OBSERVATION'
    assert failed['recovery']['current_observation']['valid'] is False
    assert failed['recovery']['decision_hint'] == 'restore_observation'
    fresh = call(server, 'reobserve')
    assert fresh['recovery']['remaining_actions'] == 2
    assert fresh['recovery']['decision_hint'] == 'resume_or_replan_from_valid_state'
    resumed = call(server, 'execute_route', route_id=plan['route_id'],
        operation_id='stale-resumed', max_sim_seconds=15)
    assert resumed['status'] == 'ARRIVED'
    replayed = call(server, 'execute_route', **arguments)
    assert replayed['reason'] == 'STALE_OBSERVATION'  # Immutable historical result.
    assert replayed['recovery']['active_failure'] is None
    assert replayed['recovery']['last_execution_result_replayed'] is True
    assert replayed['recovery']['consumed_actions'] == 1


def test_recovery_context_reads_do_not_invent_or_refresh_sensor_state():
    mission = Mission('stale')
    assert mission.recovery_context()['current_observation']['reason'] == 'UNKNOWN_OBSERVATION'
    assert mission.verify()['final_state'] is None
    plan = mission.plan_route(target='panel_a')
    assert mission.execute_route(route_id=plan['route_id'], operation_id='first',
        max_sim_seconds=15)['reason'] == 'STALE_OBSERVATION'
    before = mission.verify()['sim_time_sec']
    for _ in range(3):
        context = mission.recovery_context()
        assert context['current_observation']['valid'] is False
        assert context['current_observation']['reason'] == 'STALE_OBSERVATION'
        assert context['observation_refreshed'] is False
    after = mission.verify()
    assert after['sim_time_sec'] == before
    assert after['final_state'] is None


def test_last_allowed_replan_can_execute_and_continue_without_resetting_budget():
    server = connected('blocked')
    reach_panel(server)
    assert call(server, 'plan_route', target='pump_b', route='direct')['reason'] == 'PATH_BLOCKED'
    for _ in range(3):
        plan = call(server, 'plan_route', target='pump_b', route='upper')
        assert plan['ok']
    assert plan['recovery']['remaining_actions'] == 0
    arrived = call(server, 'execute_route', route_id=plan['route_id'],
        operation_id='pump-b', max_sim_seconds=15)
    assert arrived['status'] == 'ARRIVED'
    assert arrived['recovery']['active_failure'] is None
    assert call(server, 'inspect', target='pump_b')['recovery']['consumed_actions'] == 3
    exit_plan = call(server, 'plan_route', target='exit_c', route='direct')
    assert call(server, 'execute_route', route_id=exit_plan['route_id'],
        operation_id='exit-c', max_sim_seconds=15)['status'] == 'ARRIVED'
    assert call(server, 'inspect', target='exit_c')['ok']
    final = call(server, 'verify')
    assert final['task_success'] is True
    assert final['recovery']['remaining_actions'] == 0


def test_agent_cannot_set_recovery_budget_or_mutate_returned_history():
    server = connected('unreachable')
    reach_panel(server)
    feedback = call(server, 'plan_route', target='pump_b', route='direct')['recovery']
    feedback['consumed_actions'] = 0
    feedback['assessed_routes'].clear()
    denied = request(server, 'wait', seconds=1, max_actions=999)
    assert denied['error']['code'] == -32602
    current = call(server, 'verify')['recovery']
    assert current['max_actions'] == 3
    assert [row['route'] for row in current['assessed_routes']] == ['direct']
    assert current['untried_routes'] == ['upper', 'lower']
