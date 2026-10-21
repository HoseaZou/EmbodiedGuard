"""A saved run must explain intended motion separately from executed motion."""
import json
import pytest

from embodiedguard.execution import Mission


def test_plans_survive_save_and_link_recovery_to_actual_motion(tmp_path):
    mission = Mission('blocked')
    first = mission.plan_route(target='panel_a', route='upper', rationale='Visit the first panel.')
    arrival = mission.execute_route(route_id=first['route_id'], operation_id='panel', max_sim_seconds=15.)
    assert arrival['status'] == 'ARRIVED'
    mission.inspect(target='panel_a')
    failed = mission.plan_route(target='pump_b', route='direct', rationale='Check the direct aisle.')
    assert failed['reason'] == 'PATH_BLOCKED'
    recovered = mission.plan_route(target='pump_b', route='upper', rationale='Direct was blocked; choose upper.')
    result = mission.execute_route(route_id=recovered['route_id'], operation_id='pump', max_sim_seconds=15.)
    assert result['status'] == 'ARRIVED'
    mission.save(tmp_path)

    plans = [json.loads(line) for line in (tmp_path / 'plans.jsonl').read_text(encoding='utf-8').splitlines()]
    assert len(plans) == 3
    assert [p['status'] for p in plans] == ['PLANNED', 'REJECTED', 'PLANNED']
    assert plans[0]['points_xy'][0] == [0., 0.]
    assert plans[0]['points_xy'][-1] == pytest.approx([12., 0.], abs=1e-9)
    assert plans[0]['parameterization'] == 'spatial'
    assert plans[0]['coordinate_frame'] == 'world'
    assert plans[0]['source_observation']['frame_id'] == 0
    assert plans[1]['route_id'] is None
    assert plans[1]['reason'] == 'PATH_BLOCKED'
    assert plans[1]['blocked_by'] == ['blocked_aisle']
    assert plans[1]['points_xy']
    assert plans[2]['rationale'] == 'Direct was blocked; choose upper.'
    assert recovered['plan_id'] == plans[2]['plan_id'] == result['plan_id']
    assert result['remaining_targets'] == ['pump_b', 'exit_c']
    records = [json.loads(line) for line in (tmp_path / 'trace.jsonl').read_text(encoding='utf-8').splitlines()]
    assert all(row['plan_id'] == plans[0 if row['operation_id'] == 'panel' else 2]['plan_id'] for row in records)
    assert not any(row['plan_id'] == plans[1]['plan_id'] for row in records)
    assert any(row['safety_assessment']['allowed'] for row in records if row['safety_assessment'])
    assert all(row['swept_clearance_m'] is None or row['swept_clearance_m'] >= 0 for row in records)
    exported = mission.evidence()
    exported['plans'][0]['points_xy'][0][0] = 900
    assert mission.evidence()['plans'][0]['points_xy'][0] == [0., 0.]


def test_unavailable_observation_does_not_invent_a_plan_or_anchor_pose():
    mission = Mission('stale')
    first = mission.plan_route(target='panel_a')
    result = mission.execute_route(route_id=first['route_id'], operation_id='stale', max_sim_seconds=8.)
    assert result['reason'] == 'STALE_OBSERVATION'
    mission.elapse_idle_wall(1.)
    failed = mission.plan_route(target='panel_a', route='upper')
    assert not failed['ok']
    record = mission.evidence()['plans'][-1]
    assert record['status'] == 'REJECTED'
    assert record['reason'] == 'STALE_OBSERVATION'
    assert record['points_xy'] is None
    assert record['anchor_pose'] is None
    assert record['source_observation'] is None
