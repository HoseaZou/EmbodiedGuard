"""Six public tools, explicit planning/resume, original checked15s primitives."""
import pytest

from embodiedguard.mcp_server import MCPServer
from embodiedguard.motion_faults import MotionFaultMission
from embodiedguard.task_contract import TaskMission


def server(mission):
    value = MCPServer(mission, response_mode='compact', interface_profile='focused')
    value.handle(dict(jsonrpc='2.0', id=0, method='initialize', params={}))
    value.handle(dict(jsonrpc='2.0', method='notifications/initialized'))
    return value


def call(value, name, **args):
    response = value.handle(dict(jsonrpc='2.0', id=value._tool_calls+1, method='tools/call',
        params=dict(name=name, arguments=args)))
    assert 'error' not in response, response
    return response['result']['structuredContent']


def move(value, target, mode, operation, route='direct'):
    return call(value, 'navigate', target=target, route=route, route_mode=mode, operation_id=operation)


def test_public_scope_and_schema_reject_hidden_primitives_and_overrides():
    value = server(TaskMission('selective_targets'))
    assert {t['name'] for t in value.tools} == {'observe','navigate','reobserve','wait','stop','verify'}
    for name, args in [('plan_route', dict(target='pump_b')), ('execute_route', {}), ('inspect', dict(target='pump_b')),
        ('navigate', dict(target='pump_b', route='direct', operation_id='x', route_mode='plan', max_sim_seconds=30)),
        ('navigate', dict(target='pump_b', route='direct', operation_id='x'))]:
        response = value.handle(dict(jsonrpc='2.0', id=99, method='tools/call', params=dict(name=name, arguments=args)))
        assert response['error']['code'] == -32602
    assert value.mission._metrics['executions'] == 0


def test_one_window_partial_requires_new_agent_resume_decision():
    mission = TaskMission('selective_targets')
    value = server(mission)
    partial = move(value, 'pump_b', 'plan', 'first')
    assert partial['reason'] == 'SEGMENT_COMPLETE' and mission._metrics['executions'] == 1
    assert partial['inspection']['performed'] is False
    result = move(value, 'pump_b', 'resume', 'second')
    assert result['status'] == 'ARRIVED' and result['navigation']['route_reused'] is True
    assert result['navigation']['planning_dispatched'] is False
    assert len(mission._plans) == 1 and mission._metrics['executions'] == 2
    assert result['recovery']['consumed_actions'] == 0


def test_resume_cannot_silently_plan_or_change_target():
    mission = TaskMission('selective_targets')
    value = server(mission)
    before = (len(mission._trace), len(mission._plans), mission._world.time_sec)
    failed = move(value, 'pump_b', 'resume', 'missing')
    assert failed['ok'] is False and failed['reason'] == 'NO_MATCHING_RESUME_ROUTE'
    assert (len(mission._trace), len(mission._plans), mission._world.time_sec) == before
    move(value, 'pump_b', 'plan', 'created')
    before = (len(mission._trace), len(mission._plans))
    failed = move(value, 'exit_c', 'resume', 'wrong-target')
    assert failed['ok'] is False and (len(mission._trace), len(mission._plans)) == before


def test_explicit_plan_rebuilds_even_on_matching_handle():
    mission = TaskMission('selective_targets')
    value = server(mission)
    move(value, 'pump_b', 'plan', 'a')
    move(value, 'pump_b', 'plan', 'b')
    assert len(mission._plans) == 2
    assert not mission._terminated


def test_mode_is_part_of_immutable_operation_identity():
    mission = TaskMission('selective_targets')
    value = server(mission)
    first = move(value, 'pump_b', 'plan', 'same')
    before = (len(mission._trace), len(mission._plans), mission._metrics['executions'])
    replay = move(value, 'pump_b', 'plan', 'same')
    assert replay['navigation']['replayed'] is True
    assert replay['state'] == first['state']
    assert (len(mission._trace), len(mission._plans), mission._metrics['executions']) == before
    conflict = move(value, 'pump_b', 'resume', 'same')
    assert conflict['reason'] == 'OPERATION_CONFLICT'


def test_actual_sensor_failure_requires_agent_reobserve_then_resume():
    mission = MotionFaultMission('motion_sensor_freeze')
    value = server(mission)
    failed = move(value, 'panel_a', 'plan', 'fault')
    assert failed['reason'] == 'STALE_OBSERVATION'
    assert failed['recovery']['resume']['checks']['fresh_observation'] is False
    assert failed['inspection']['performed'] is False
    call(value, 'reobserve')
    result = move(value, 'panel_a', 'resume', 'restored')
    assert result['status'] == 'ARRIVED' and len(mission._plans) == 1
    assert result['recovery']['consumed_actions'] == 1


def test_route_permissions_and_three_action_recovery_remain_enforced():
    mission = TaskMission('restricted_detour')
    value = server(mission)
    move(value, 'panel_a', 'plan', 'panel')
    invalid = move(value, 'pump_b', 'plan', 'closed', 'upper')
    assert invalid['ok'] is False and invalid['reason'] == 'ROUTE_NOT_PERMITTED'
    failed = move(value, 'pump_b', 'plan', 'blocked')
    assert failed['reason'] == 'PATH_BLOCKED'
    recovered = move(value, 'pump_b', 'plan', 'lower', 'lower')
    assert recovered['status'] == 'ARRIVED'
    assert recovered['recovery']['consumed_actions'] == 1


def test_success_requires_actual_inspections_and_separate_final_verify():
    mission = TaskMission('conditional_skip')
    value = server(mission)
    first = move(value, 'panel_a', 'plan', 'panel')
    assert first['task']['revision'] == 2
    assert first['task']['required_targets'] == ['panel_a','exit_c']
    last = move(value, 'exit_c', 'plan', 'exit')
    if last.get('reason') == 'SEGMENT_COMPLETE':
        move(value, 'exit_c', 'resume', 'exit-resume')
    final = call(value, 'verify')
    assert final['task_success'] is True and final['missing_targets'] == []
    assert 'criteria' in final['completion_receipt'] and 'metrics' in final['completion_receipt']
    assert final['model_call_verified'] is False and final['model_cost'] is None
