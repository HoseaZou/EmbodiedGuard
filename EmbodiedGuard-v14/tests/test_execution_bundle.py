from copy import deepcopy
import hashlib
import json

from embodiedguard.execution import Mission
from embodiedguard.mcp_server import MCPServer
from test_mcp import initialize, call


def server_for(mission):
    # The general optional bundle remains available through full mode. Compact
    # mode now discloses an immutable profile, tested separately below.
    server = MCPServer(mission, response_mode='full')
    initialize(server)
    return server


def plan(server, target='panel_a'):
    return call(server, 'plan_route', dict(target=target, route='direct'))['structuredContent']['route_id']


def execute(server, route, operation='first', duration=15, inspect=True):
    return call(server, 'execute_route', dict(route_id=route, operation_id=operation,
        max_sim_seconds=duration, inspect_on_arrival=inspect))['structuredContent']


def test_combined_execution_uses_real_inspector_and_preserves_original_motion():
    final = []
    for combined in (False, True):
        mission = Mission('normal')
        server = server_for(mission)
        for target in ('panel_a', 'pump_b', 'exit_c'):
            result = execute(server, plan(server, target), operation=target, inspect=combined)
            assert result['status'] == 'ARRIVED' and result['stopping_verified']
            if combined:
                inspected = result['inspection']
                assert inspected['performed'] and inspected['ok']
                assert inspected['evidence']['target'] == target
                assert inspected['evidence']['source'] == 'synthetic_fixture'
            else:
                assert 'inspection' not in result
                call(server, 'inspect', dict(target=target))
        assert call(server, 'verify')['structuredContent']['task_success']
        verdict = mission.verify()
        # Bundling removes transport events, so receipt event references must
        # differ but still resolve to the real evidence in each local journal.
        receipt = verdict.pop('completion_receipt')
        assert receipt['criteria_met_at_completion'] is True
        for reference in receipt['inspection_references']:
            event = next(row for row in mission._events if row['event_id'] == reference['event_id'])
            assert event['event'] == 'inspection_verified'
            evidence = event['evidence']
            assert evidence['target'] == reference['target']
            encoded = json.dumps(evidence, sort_keys=True, separators=(',', ':'),
                ensure_ascii=False, allow_nan=False).encode('utf-8')
            assert hashlib.sha256(encoded).hexdigest() == reference['evidence_sha256']
        final.append((verdict, mission._trace))
    assert final[0] == final[1]


def test_partial_execution_does_not_inspect_or_advance_target():
    mission = Mission('normal')
    server = server_for(mission)
    result = execute(server, plan(server), duration=1)
    assert result['status'] != 'ARRIVED'
    assert not result['inspection']['performed']
    assert result['completed_targets'] == [] and result['next_target'] == 'panel_a'
    assert mission.verify()['inspections'] == {}


def test_stale_failure_requires_agent_reobserve_and_new_operation_before_inspection():
    mission = Mission('stale')
    server = server_for(mission)
    route = plan(server)
    first = execute(server, route)
    assert first['reason'] == 'STALE_OBSERVATION' and not first['inspection']['performed']
    assert mission.verify()['inspections'] == {}
    call(server, 'reobserve')
    old = execute(server, route)
    assert old['reason'] == 'STALE_OBSERVATION'
    assert old['recovery']['last_execution_result_replayed'] is True
    assert not old['inspection']['performed']
    resumed = execute(server, route, operation='resumed')
    assert resumed['inspection']['performed'] and resumed['inspection']['ok']
    assert resumed['completed_targets'] == ['panel_a']
    assert resumed['recovery']['consumed_actions'] == 1


def test_combined_retry_does_not_repeat_inspection_or_motion_and_mode_changes_conflict():
    mission = Mission('normal')
    server = server_for(mission)
    route = plan(server)
    first = execute(server, route)
    before = deepcopy(mission.verify())
    inspection_events = [r for r in mission._events if r['event'] == 'inspection_verified']
    again = execute(server, route)
    assert again['inspection'] == first['inspection']
    assert again['recovery']['last_execution_result_replayed'] is True
    assert mission.verify() == before
    assert [r for r in mission._events if r['event'] == 'inspection_verified'] == inspection_events
    conflict = execute(server, route, inspect=False)
    assert conflict['reason'] == 'OPERATION_CONFLICT'
    assert mission.verify() == before


def test_separate_execution_cannot_be_replayed_as_a_new_combined_inspection():
    mission = Mission('normal')
    server = server_for(mission)
    route = plan(server)
    execute(server, route, inspect=False)
    conflict = execute(server, route, inspect=True)
    assert conflict['reason'] == 'OPERATION_CONFLICT'
    assert mission.verify()['inspections'] == {}
    assert mission.verify()['metrics']['executions'] == 1


def test_inspector_failure_is_reported_instead_of_inventing_completion(monkeypatch):
    mission = Mission('normal')
    server = server_for(mission)
    route = plan(server)
    monkeypatch.setattr(mission, 'inspect', lambda **kwargs: mission._failure('UNKNOWN_OBSERVATION'))
    result = execute(server, route)
    assert result['execution_status'] == 'ARRIVED'
    assert result['status'] == 'INSPECTION_FAILED' and result['reason'] == 'UNKNOWN_OBSERVATION'
    assert result['ok'] is False and result['inspection']['ok'] is False
    assert mission.verify()['inspections'] == {} and not mission.verify()['task_success']


def test_insufficient_budget_prevents_both_motion_and_inspection():
    mission = Mission('normal', max_sim_seconds=10)
    server = server_for(mission)
    result = execute(server, plan(server))
    assert result['reason'] == 'INSUFFICIENT_COMPLETION_BUDGET'
    assert not result['inspection']['performed']
    verdict = mission.verify()
    assert verdict['sim_time_sec'] == 0 and verdict['metrics']['executions'] == 0
    assert verdict['inspections'] == {}
