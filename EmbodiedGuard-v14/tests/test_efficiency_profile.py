"""Real MCP/plant integration checks for fewer calls without weaker guards."""
from copy import deepcopy
import json

import pytest

from embodiedguard.execution import Mission
from embodiedguard.mcp_server import MCPServer
from embodiedguard.motion_faults import MotionFaultMission
from embodiedguard.task_contract import TaskMission


def connect(mission, profile='lean', **kwargs):
    server = MCPServer(mission, response_mode='compact', interface_profile=profile, **kwargs)
    server.handle(dict(jsonrpc='2.0', id=0, method='initialize', params={}))
    server.handle(dict(jsonrpc='2.0', method='notifications/initialized'))
    return server


def request(server, tool, **args):
    return server.handle(dict(jsonrpc='2.0', id=server._tool_calls+1, method='tools/call',
                             params=dict(name=tool, arguments=args)))


def call(server, tool, **args):
    response = request(server, tool, **args)
    assert 'error' not in response, response
    return response['result']['structuredContent']


def test_navigate_merges_one_chosen_route_and_actual_inspection():
    mission = TaskMission('conditional_skip')
    server = connect(mission)
    call(server, 'observe')
    result = call(server, 'navigate', target='panel_a', route='direct', operation_id='panel')
    assert result['status'] == 'ARRIVED' and result['stopping_verified']
    assert result['inspection']['performed'] and result['inspection']['evidence']['anomaly'] is False
    assert result['task']['revision'] == 2 and result['next_target'] == 'exit_c'
    assert result['task']['required_targets'] == ['panel_a', 'exit_c']
    assert result['tool_budget']['counted_requests'] == 2
    assert mission._metrics['executions'] == 1
    assert len(mission._plans) == 1
    assert any(event['event']=='navigation_child_finished' and event['tool']=='plan_route' for event in mission._events)
    assert not mission.verify()['task_success']  # No automatic next target or final verify.


def test_partial_execution_exposes_existing_route_without_new_planning():
    mission = TaskMission('selective_targets')
    server = connect(mission, 'resume')
    plan = call(server, 'plan_route', target='pump_b', route='direct')
    partial = call(server, 'execute_route', route_id=plan['route_id'], operation_id='first')
    assert partial['reason'] == 'SEGMENT_COMPLETE'
    resume = partial['recovery']['resume']
    assert resume['route_id'] == plan['route_id'] and resume['eligible_to_attempt'] is True
    assert resume['requires_execution_safety_check'] is True
    before = (deepcopy(mission._trace), mission._store._clock, len(mission._plans), mission._world.time_sec)
    for _ in range(3):
        server._tool_context()
    assert (mission._trace, mission._store._clock, len(mission._plans), mission._world.time_sec) == before
    result = call(server, 'execute_route', route_id=plan['route_id'], operation_id='continued')
    assert result['status'] == 'ARRIVED' and len(mission._plans) == 1


def test_sensor_restore_allows_same_route_but_metadata_never_resenses():
    mission = MotionFaultMission('motion_sensor_freeze')
    server = connect(mission)
    failed = call(server, 'navigate', target='panel_a', route='direct', operation_id='first')
    assert failed['reason'] == 'STALE_OBSERVATION'
    assert failed['inspection']['performed'] is False
    assert failed['recovery']['resume']['eligible_to_attempt'] is False
    assert failed['recovery']['resume']['checks']['fresh_observation'] is False
    restored = call(server, 'reobserve')
    assert restored['recovery']['resume']['eligible_to_attempt'] is True
    result = call(server, 'execute_route', route_id=failed['route_id'], operation_id='resume')
    assert result['status'] == 'ARRIVED' and len(mission._plans) == 1
    assert result['recovery']['consumed_actions'] == 1


def test_bundle_plan_failure_never_moves_inspects_or_selects_alternative():
    mission = TaskMission('restricted_detour')
    server = connect(mission)
    call(server, 'navigate', target='panel_a', route='direct', operation_id='panel')
    before = (mission._world.plant.state, mission._world.time_sec, mission._metrics['executions'])
    failed = call(server, 'navigate', target='pump_b', route='direct', operation_id='blocked')
    assert failed['reason'] == 'PATH_BLOCKED' and failed['inspection']['performed'] is False
    assert (mission._world.plant.state, mission._world.time_sec, mission._metrics['executions']) == before
    assert failed['recovery']['consumed_actions'] == 0
    arrived = call(server, 'navigate', target='pump_b', route='lower', operation_id='chosen-recovery')
    assert arrived['status'] == 'ARRIVED' and arrived['recovery']['consumed_actions'] == 1
    assert [plan['route'] for plan in mission._plans] == ['direct', 'direct', 'lower']


def test_nested_planning_cannot_bypass_recovery_limit():
    mission = TaskMission('restricted_unreachable')
    server = connect(mission)
    call(server, 'navigate', target='panel_a', route='direct', operation_id='panel')
    for index in range(4):
        result = call(server, 'navigate', target='pump_b', route='direct', operation_id=f'fail-{index}')
        assert result['reason'] == 'PATH_BLOCKED'
    plans = len(mission._plans)
    denied = call(server, 'navigate', target='pump_b', route='direct', operation_id='over-cap')
    assert denied['reason'] == 'RECOVERY_BUDGET_EXHAUSTED'
    assert denied['recovery']['consumed_actions'] == 3 and len(mission._plans) == plans
    call(server, 'stop', terminate=True, reason='no legal feasible candidate')
    assert call(server, 'verify')['safe_termination']


def test_navigate_retries_are_historical_and_conflicting_calls_do_not_replan():
    mission = TaskMission('restricted_detour')
    server = connect(mission)
    first = call(server, 'navigate', target='panel_a', route='direct', operation_id='panel')
    snapshot = (len(mission._plans), mission._metrics['executions'], len(mission._inspections))
    replay = call(server, 'navigate', target='panel_a', route='direct', operation_id='panel')
    assert replay['status'] == first['status'] and replay['navigation']['replayed']
    assert replay['task']['revision'] == 2
    assert (len(mission._plans), mission._metrics['executions'], len(mission._inspections)) == snapshot
    assert call(server, 'navigate', target='pump_b', route='lower', operation_id='panel')['reason'] == 'OPERATION_CONFLICT'
    assert call(server, 'execute_route', route_id=first['route_id'], operation_id='panel')['reason'] == 'OPERATION_CONFLICT'
    assert len(mission._plans) == snapshot[0]


def test_cached_failed_navigation_never_spends_new_recovery_or_clears_failure():
    mission = TaskMission('restricted_detour')
    server = connect(mission)
    call(server, 'navigate', target='panel_a', route='direct', operation_id='panel')
    args = dict(target='pump_b', route='direct', operation_id='failed')
    call(server, 'navigate', **args)
    call(server, 'navigate', target='pump_b', route='lower', operation_id='recover')
    before = len(mission._plans)
    historical = call(server, 'navigate', **args)
    assert historical['reason'] == 'PATH_BLOCKED'
    assert historical['recovery']['active_failure'] is None
    assert historical['recovery']['consumed_actions'] == 1 and len(mission._plans) == before


@pytest.mark.parametrize('args', [dict(max_sim_seconds=30), dict(inspect_on_arrival=False),
    dict(max_actions=999), dict(operation_id=''), dict(route='teleport')])
def test_navigation_profile_is_fixed_and_schema_rejects_before_motion(args):
    mission = TaskMission('conditional_skip')
    server = connect(mission)
    parameters = dict(target='panel_a', route='direct', operation_id='id')
    parameters.update(args)
    assert request(server, 'navigate', **parameters)['error']['code'] == -32602
    assert len(mission._plans) == mission._metrics['executions'] == 0


def test_task_version_or_permission_changes_invalidate_resume():
    mission = TaskMission('conditional_skip')
    server = connect(mission, 'resume')
    plan = call(server, 'plan_route', target='panel_a')
    call(server, 'execute_route', route_id=plan['route_id'], operation_id='panel')
    current = server._tool_context()['recovery']['resume']
    assert not current['eligible_to_attempt']
    denied = call(server, 'execute_route', route_id=plan['route_id'], operation_id='obsolete')
    assert denied['reason'] == 'TASK_REVISION_CHANGED'


def test_lean_feedback_keeps_current_constraints_and_full_final_verify():
    mission = TaskMission('conditional_skip')
    server = connect(mission)
    initial = call(server, 'observe')
    assert 'brief' in initial['task'] and initial['task']['revision'] == 1
    repeated = call(server, 'observe')
    assert 'brief' not in repeated['task']
    for key in ('revision', 'required_targets', 'permitted_routes', 'next_target', 'conditional_rule'):
        assert repeated['task'][key] == initial['task'][key]
    assert len(json.dumps(repeated, ensure_ascii=False).encode()) < len(json.dumps(initial, ensure_ascii=False).encode())
    changed = call(server, 'navigate', target='panel_a', route='direct', operation_id='panel')
    assert 'brief' in changed['task'] and changed['task']['updates']
    final = call(server, 'verify')
    assert 'brief' in final['task'] and 'updates' in final['task']
    assert final['task_success'] is False


def test_idle_time_is_charged_and_stale_hint_cannot_authorize_motion(monkeypatch):
    clock = [100.]
    monkeypatch.setattr('embodiedguard.mcp_server.time.monotonic', lambda: clock[0])
    mission = TaskMission('conditional_skip')
    server = connect(mission, charge_idle_wall=True)
    call(server, 'observe')
    first = call(server, 'navigate', target='panel_a', route='direct', operation_id='panel')
    clock[0] += 300.
    final = call(server, 'verify')
    assert final['termination_reason'] == 'TIME_BUDGET_EXHAUSTED'
    assert not final['task_success']
    assert final['sim_time_sec'] >= 300 and final['budget']['remaining_sim_sec'] == 0
    assert final['recovery']['resume']['eligible_to_attempt'] is False


def test_unknown_state_has_no_invented_route_or_budget_forecast():
    mission = Mission('normal')
    server = connect(mission, 'resume')
    initial = server._tool_context()['recovery']['resume']
    assert initial['route_id'] is None and initial['eligible_to_attempt'] is False
    assert initial.get('conditional_estimate') is None
    assert mission._store._records == mission._store._records.__class__(maxlen=256)


def test_direct_execution_ids_cannot_be_reused_to_plan_navigation():
    mission = TaskMission('selective_targets')
    server = connect(mission)
    plan = call(server, 'plan_route', target='pump_b')
    call(server, 'execute_route', route_id=plan['route_id'], operation_id='direct-id')
    before = len(mission._plans)
    result = call(server, 'navigate', target='pump_b', route='direct', operation_id='direct-id')
    assert result['reason'] == 'OPERATION_CONFLICT' and len(mission._plans) == before


def test_native_request_cap_stops_macro_before_nested_work():
    mission = TaskMission('conditional_skip')
    server = connect(mission)
    server._tool_calls = 120
    result = call(server, 'navigate', target='panel_a', route='direct', operation_id='over-request-cap')
    assert result['reason'] == 'TOOL_BUDGET_EXHAUSTED'
    assert mission._metrics['executions'] == len(mission._plans) == 0
    assert call(server, 'verify')['safe_termination']


def test_insufficient_budget_rejects_navigation_motion_without_inspection():
    mission = Mission('normal', max_sim_seconds=10)
    server = connect(mission)
    result = call(server, 'navigate', target='panel_a', route='direct', operation_id='too-late')
    assert result['reason'] == 'INSUFFICIENT_COMPLETION_BUDGET'
    assert mission._metrics['executions'] == 0 and not mission._inspections
    assert result['inspection']['performed'] is False and result['stopping_verified']


def test_eligible_hint_does_not_bypass_new_sensor_failure_or_cancellation():
    mission = MotionFaultMission('motion_sensor_unavailable')
    server = connect(mission)
    plan = call(server, 'plan_route', target='panel_a')
    assert plan['recovery']['resume']['eligible_to_attempt']
    failed = call(server, 'execute_route', route_id=plan['route_id'], operation_id='sensor-failure')
    assert failed['reason'] == 'UNKNOWN_OBSERVATION' and not mission._inspections
    assert failed['metrics']['collisions'] == 0 and failed['stopping_verified']
    server.handle(dict(jsonrpc='2.0', method='notifications/cancelled'))
    assert mission._world.plant.state.speed_mps == 0
