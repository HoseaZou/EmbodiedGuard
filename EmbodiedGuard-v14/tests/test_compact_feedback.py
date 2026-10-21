from copy import deepcopy
import json

from embodiedguard.execution import Mission
from embodiedguard.feedback import compact_feedback
from embodiedguard.mcp_server import MCPServer
from test_mcp import initialize, call


def test_compaction_preserves_unknown_failure_fields_and_does_not_modify_evidence():
    raw = dict(ok=False, reason='UNKNOWN_OBSERVATION', state=None, future_failure={'detail': None},
        recovery=dict(current_observation={'valid': False, 'reason': 'UNKNOWN_OBSERVATION'},
            active_failure={'reason': 'UNKNOWN_OBSERVATION'}, remaining_actions=0,
            actions=[{'tool': 'reobserve'}], counting_contract='constant'),
        execution_estimate=dict(guaranteed=False, estimated_arrival_and_stop_sec=None,
            unavailable_reason='RuntimeError', assumptions=['constant']),
        budget=dict(remaining_sim_sec=0., clock_policy='real_idle', schema_version=1,
            max_execution_operations=9), tool_budget=dict(max_tool_requests=10, remaining_requests=0))
    before = deepcopy(raw)
    projected = compact_feedback(raw)
    assert raw == before
    assert projected['state'] is None and projected['future_failure'] == {'detail': None}
    assert projected['reason'] == raw['reason']
    assert projected['recovery']['current_observation'] == raw['recovery']['current_observation']
    assert projected['recovery']['active_failure'] == raw['recovery']['active_failure']
    assert projected['execution_estimate']['estimated_arrival_and_stop_sec'] is None
    assert projected['execution_estimate']['unavailable_reason'] == 'RuntimeError'
    assert projected['budget']['remaining_sim_sec'] == 0.
    assert projected['budget']['max_execution_operations'] == 9
    assert projected['tool_budget']['max_tool_requests'] == 10


def test_compact_transport_has_matching_wire_and_structured_data_and_full_local_log():
    mission = Mission('normal')
    server = MCPServer(mission, response_mode='compact')
    initialize(server)
    received = call(server, 'observe')
    payload = received['structuredContent']
    assert json.loads(received['content'][0]['text']) == payload
    finished = mission._events[-1]
    assert finished['model_result'] == payload
    assert 'counting_contract' in finished['result']['recovery']
    assert 'counting_contract' not in payload['recovery']
    assert payload['observation']['frame_id'] == 0
    assert payload['observation']['obstacles_valid'] is True


def test_compact_and_full_paths_preserve_identical_motion_and_verdict_without_wall_wait():
    outcomes = []
    for mode in ('full', 'compact'):
        mission = Mission('normal')
        server = MCPServer(mission, response_mode=mode)
        initialize(server)
        for target in ('panel_a', 'pump_b', 'exit_c'):
            route = call(server, 'plan_route', dict(target=target, route='direct'))['structuredContent']['route_id']
            result = call(server, 'execute_route', dict(route_id=route, operation_id=target,
                max_sim_seconds=15, inspect_on_arrival=True))['structuredContent']
            assert result['status'] == 'ARRIVED' and result['stopping_verified']
            assert call(server, 'inspect', dict(target=target))['structuredContent']['ok']
        verdict = call(server, 'verify')['structuredContent']
        assert verdict['task_success'] and verdict['inspections']['pump_b']['anomaly']
        outcomes.append((mission.verify(), mission._trace))
    assert outcomes[0] == outcomes[1]
