"""Test disclosed fixed actions against real Mission safety, not guessed state."""
from copy import deepcopy
import pytest
from embodiedguard.execution import Mission
from embodiedguard.mcp_server import MCPServer
from test_mcp import initialize, call

def setup(scenario='normal',compact=True):
    m=Mission(scenario);s=MCPServer(m,response_mode='compact' if compact else 'full');initialize(s)
    route=call(s,'plan_route',dict(target='panel_a',route='direct'))['structuredContent']['route_id']
    return m,s,route

def test_disclosed_profile_and_explicit_constants_have_identical_motion_and_evidence():
    runs=[]
    for constants in ({},{'max_sim_seconds':15,'inspect_on_arrival':True}):
        m,s,route=setup()
        args=dict(route_id=route,operation_id='one',**constants)
        result=call(s,'execute_route',args)['structuredContent']
        assert result['status']=='ARRIVED' and result['inspection']['performed']
        start=next(e for e in m._events if e['event']=='tool_call_started' and e['tool']=='execute_route')
        assert start['arguments']==args
        assert start['effective_arguments']['max_sim_seconds']==15
        assert start['effective_arguments']['inspect_on_arrival'] is True
        assert start['execution_profile']=='arrival-inspection-15s-v1'
        runs.append((m._trace,m.verify()))
    assert runs[0]==runs[1]

@pytest.mark.parametrize('override',[{'max_sim_seconds':16},{'max_sim_seconds':5},{'inspect_on_arrival':False},{'disable_safety':True}])
def test_fixed_profile_cannot_be_overridden_and_rejections_have_no_motion(override):
    m,s,route=setup();before=deepcopy(m._trace)
    result=s.handle(dict(jsonrpc='2.0',id=3,method='tools/call',params=dict(name='execute_route',arguments=dict(route_id=route,operation_id='bad',**override))))
    assert result['error']['code']==-32602
    assert m._trace==before and m.verify()['inspections']=={}

def test_stale_recovery_omitted_constants_never_refresh_or_recover_automatically():
    m,s,route=setup('stale')
    result=call(s,'execute_route',dict(route_id=route,operation_id='initial'))['structuredContent']
    assert result['reason']=='STALE_OBSERVATION' and not result['inspection']['performed']
    assert m.verify()['inspections']=={}
    call(s,'reobserve')
    resumed=call(s,'execute_route',dict(route_id=route,operation_id='resumed'))['structuredContent']
    assert resumed['status']=='ARRIVED' and resumed['inspection']['performed']
    assert resumed['recovery']['consumed_actions']==1

def test_full_mode_keeps_explicit_duration_requirement():
    m,s,route=setup(compact=False)
    result=s.handle(dict(jsonrpc='2.0',id=3,method='tools/call',params=dict(name='execute_route',arguments=dict(route_id=route,operation_id='missing'))))
    assert result['error']['code']==-32602
    assert m.verify()['inspections']=={}
