"""Verify observation, idempotency, safety and continuation through real interfaces."""
from copy import deepcopy
import json

import pytest

from embodiedguard.control import LegacyTracker
from embodiedguard.evidence import load_run
from embodiedguard.execution import Mission
from embodiedguard.mcp_server import MCPServer
from embodiedguard.shadow import ShadowMission, attach_shadow
from embodiedguard.trajectory_preview import preview_trajectory


def call(server, tool, arguments=None):
    return server.handle(dict(jsonrpc='2.0', id=1, method='tools/call',
        params=dict(name=tool, arguments=arguments or {})))['result']


def server(mission, mode='compact'):
    result = MCPServer(mission, response_mode=mode)
    result.handle(dict(jsonrpc='2.0', id=1, method='initialize',
                       params={'protocolVersion':'2025-11-25'}))
    result.handle(dict(jsonrpc='2.0', method='notifications/initialized'))
    return result


@pytest.mark.parametrize('mode', ['full','compact'])
def test_shadow_does_not_change_tools_sensor_polls_or_mcp_feedback(mode):
    plain, shadow = Mission('normal'), ShadowMission('normal')
    polls = [0,0]
    for index, mission in enumerate((plain,shadow)):
        packet = mission._world.packet
        def counted(packet=packet, index=index):
            polls[index] += 1
            return packet()
        mission._world.packet = counted
    left, right = server(plain,mode), server(shadow,mode)
    catalog = dict(jsonrpc='2.0', id=2, method='tools/list')
    assert left.handle(catalog) == right.handle(catalog)
    for tool,args in [('observe',{}), ('plan_route',{'target':'panel_a','route':'upper'}),
        ('execute_route',{'route_id':'route-1','operation_id':'one',**({'max_sim_seconds':4.} if mode=='full' else {})})]:
        assert call(left,tool,args) == call(right,tool,args)
    assert plain._trace == shadow._trace
    assert plain.verify() == shadow.verify()
    assert polls[0] == polls[1]
    evidence = shadow.shadow_evidence()
    assert not evidence['shadow_errors']
    assert len(evidence['action_contracts']) == 3
    assert evidence['action_contracts'][-1]['effective_arguments']['max_sim_seconds'] == (4. if mode=='full' else 15.)
    assert evidence['trajectory_previews'][0]['motion_authorized'] is False


def test_continuation_uses_the_live_tracker_and_cached_retry_creates_no_preview():
    mission = ShadowMission('normal')
    plan = mission.plan_route(target='panel_a',route='upper')
    for index in range(2):
        args = dict(route_id=plan['route_id'],operation_id=f'segment-{index}',max_sim_seconds=4.)
        result = mission.execute_route(**args)
        evidence = mission.shadow_evidence()
        count = len(evidence['trajectory_previews'])
        trace_length = len(mission._trace)
        assert mission.execute_route(**args) == result
        assert len(mission.shadow_evidence()['trajectory_previews']) == count
        assert len(mission._trace) == trace_length
    evidence = mission.shadow_evidence()
    assert evidence['trajectory_previews'][1]['anchor_state']['x'] > 1.
    for check in evidence['preview_checks']:
        assert check['matched_samples'] > 10
        assert check['position_max_error_m'] < 1e-10


def test_recovery_receipt_references_the_actual_failure_without_invented_reasoning(tmp_path):
    mission = ShadowMission('blocked')
    client = server(mission)
    call(client,'plan_route',{'target':'panel_a','route':'direct'})
    call(client,'execute_route',{'route_id':'route-1','operation_id':'one'})
    call(client,'plan_route',{'target':'pump_b','route':'direct'})
    call(client,'plan_route',{'target':'pump_b','route':'upper'})
    decisions = mission.shadow_evidence()['recovery_decisions']
    assert decisions[-1]['failure_code'] == 'PATH_BLOCKED'
    assert decisions[-1]['decision_summary'] is None
    events = {row['event_id']:row for row in mission._events}
    refs = decisions[-1]['evidence_refs']
    assert events[refs[0]['event_id']]['result']['reason'] == 'PATH_BLOCKED'
    assert events[refs[1]['event_id']]['arguments']['route'] == 'upper'
    mission.save(tmp_path)
    attached = attach_shadow(load_run(tmp_path),tmp_path)
    assert attached['summary']['model_call_verified'] is False
    assert attached['recovery_decisions'] == decisions
    (tmp_path/'recovery_decisions.jsonl').write_text('{}\n',encoding='utf-8')
    with pytest.raises(ValueError,match='hash mismatch'):
        attach_shadow(load_run(tmp_path),tmp_path)


@pytest.mark.parametrize('mutation,now,frame,expected', [
    ('missing',0.,0,'UNKNOWN_OBSERVATION'),
    ('missing_pose',0.,0,'INVALID_OBSERVATION'),
    ('nan_pose',0.,0,'INVALID_OBSERVATION'),
    ('unknown_obstacles',0.,0,'UNKNOWN_OBSERVATION'),
    ('other_episode',0.,0,'EPISODE_MISMATCH'),
    ('unchanged',.31,0,'STALE_OBSERVATION'),
    ('unchanged',0.,1,'STALE_OBSERVATION'),
])
def test_no_prediction_from_unknown_or_stale_state(mutation,now,frame,expected):
    mission = Mission('normal')
    observed = deepcopy(mission.observe()['observation'])
    if mutation == 'missing': observed = None
    elif mutation == 'missing_pose': observed.pop('pose')
    elif mutation == 'nan_pose': observed['pose']['x'] = float('nan')
    elif mutation == 'unknown_obstacles': observed['obstacles_valid'] = False
    elif mutation == 'other_episode': observed['episode_id'] = 'wrong'
    result = preview_trajectory(observation=observed,episode_id='normal-0',
        current_frame_id=frame,now=now,tracker=LegacyTracker([(0.,0.),(12.,0.)]),
        target_xy=(12.,0.),config=mission._world.plant.config,segment_seconds=15.,
        preview_id='test',route_id='route-1',operation_id='one')
    assert result['status'] == 'unavailable'
    assert result['reason'] == expected
    assert result['samples'] == []
    assert result['motion_authorized'] is False
    assert 'anchor_state' not in result
    json.dumps(result,allow_nan=False)


def test_fault_and_shadow_calculation_error_cannot_disable_the_original_stop(monkeypatch):
    mission = ShadowMission('stale')
    plan = mission.plan_route(target='panel_a')
    result = mission.execute_route(route_id=plan['route_id'],operation_id='one',max_sim_seconds=15.)
    assert result['reason'] == 'STALE_OBSERVATION'
    assert result['stopping_verified']
    monkeypatch.setattr('embodiedguard.shadow.preview_trajectory',lambda **kwargs: (_ for _ in ()).throw(RuntimeError('test observer failure')))
    mission.reobserve()
    plan = mission.plan_route(target='panel_a')
    result = mission.execute_route(route_id=plan['route_id'],operation_id='two',max_sim_seconds=15.)
    assert result['status'] == 'ARRIVED'
    assert result['stopping_verified']
    assert mission.shadow_evidence()['shadow_errors'][0]['stage'] == 'trajectory_preview'


def test_ego_local_samples_are_rotated_from_the_measured_anchor():
    import math
    mission = Mission('normal')
    observed = mission.observe()['observation']
    observed['pose'] = dict(x=2.,y=3.,yaw_rad=math.pi/2)
    result = preview_trajectory(observation=observed,episode_id='normal-0',
        current_frame_id=0,now=0.,tracker=LegacyTracker([(2.,3.),(2.,15.)]),
        target_xy=(2.,15.),config=mission._world.plant.config,segment_seconds=15.,
        preview_id='test',route_id='route-1',operation_id='one')
    assert result['status'] == 'available'
    assert result['samples'][0]['x_forward_m'] == 0.
    assert result['samples'][-1]['x_forward_m'] > 1.
    assert abs(result['samples'][-1]['y_left_m']) < 1e-10


def test_actual_replay_displays_predictions_and_cannot_show_a_future_preview(tmp_path):
    from test_replay import run_artifact
    mission = ShadowMission('normal')
    client = server(mission,'full')
    call(client,'plan_route',{'target':'panel_a','route':'upper'})
    for index in range(2):
        call(client,'execute_route',dict(route_id='route-1',operation_id=f'part-{index}',max_sim_seconds=4.))
    evidence = {**mission.evidence(),**mission.shadow_evidence()}
    run_artifact(tmp_path,evidence,r'''
seek(.5);
assert.equal(document.getElementById('preview-select').children.length,2);
assert.match(document.getElementById('preview-detail').textContent,/preview-1/);
assert.match(document.getElementById('preview-detail').textContent,/不授权运动/);
assert.ok(document.getElementById('map').draws.some(d=>d.color==='#ffd28e'&&d.points.length>10));
assert.ok(!document.getElementById('action-contract-detail').textContent.includes('result_status'));
seek(6);
assert.equal(document.getElementById('preview-select').children.length,3);
assert.match(document.getElementById('preview-detail').textContent,/preview-2/);
const selector=document.getElementById('preview-select');selector.value='preview-1';selector.handlers.change();
assert.match(document.getElementById('preview-detail').textContent,/历史预测，窗口已过/);
''')
