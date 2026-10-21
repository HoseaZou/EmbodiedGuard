"""Standalone evidence viewer contract; no browser/network dependency."""
from html.parser import HTMLParser
import json
import shutil
import subprocess

import pytest

from embodiedguard.replay import write_replay


class Scripts(HTMLParser):
    def __init__(self):
        super().__init__()
        self.active = None
        self.scripts = {}
        self.external_resources = []

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == 'script':
            self.active = attrs.get('id', 'application')
            self.scripts[self.active] = ''
        if tag in ('script', 'link', 'img', 'iframe'):
            self.external_resources.extend(v for k, v in attrs.items() if k in ('src', 'href'))

    def handle_endtag(self, tag):
        if tag == 'script':
            self.active = None

    def handle_data(self, data):
        if self.active:
            self.scripts[self.active] += data


def sample_evidence():
    return dict(summary=dict(task_success=False, safe_termination=True,
        decision_source='external_mcp_client', model_call_verified=False,
        sim_time_sec=1., completed_targets=[], missing_targets=['panel_a'],
        termination_reason='NO_SAFE_ROUTE', metrics=dict(collisions=0, boundary_violations=0,
            speed_violations=0, path_length_m=.1, cte_rmse_m=None, safety_interventions=1)),
        scenario='blocked', seed=7, bounds=(-2, -6, 40, 6),
        checkpoints={'panel_a': (12, 0)}, plant_config={'radius_m': .45},
        events=[dict(event_id=1, event='stopped', sim_time_sec=1.,
            reason='</script><script>alert("untrusted")</script>')],
        trace=[dict(sim_time_sec=1., state=dict(x=.1, y=0., yaw_rad=.2, speed_mps=0.),
            action=dict(throttle=0., brake=1., steer_left_normalized=0.),
            obstacles=[dict(id='box', x=18., y=0., radius=.65, vx=0., vy=0.)],
            geometric_cte_m=None)])


def test_replay_preserves_evidence_and_cannot_break_out_of_json_script(tmp_path):
    evidence = sample_evidence()
    target = tmp_path / 'nested' / 'replay.html'
    write_replay(evidence, target)
    html = target.read_text(encoding='utf-8')
    parsed = Scripts()
    parsed.feed(html)
    assert len(parsed.scripts) == 2
    assert not parsed.external_resources
    assert json.loads(parsed.scripts['evidence-data']) == json.loads(json.dumps(evidence))
    assert '二维运动学仿真' in html
    assert '模型身份未核验' in html
    assert 'id="play"' in html and 'id="timeline"' in html
    assert '<script>alert("untrusted")</script>' not in html


def test_empty_trace_is_renderable_and_javascript_has_valid_syntax(tmp_path):
    evidence = sample_evidence()
    evidence['trace'] = []
    evidence['events'] = []
    target = tmp_path / 'empty.html'
    write_replay(evidence, target)
    parsed = Scripts()
    parsed.feed(target.read_text(encoding='utf-8'))
    assert json.loads(parsed.scripts['evidence-data'])['trace'] == []
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node unavailable for JavaScript syntax validation')
    script = tmp_path / 'viewer.js'
    script.write_text(parsed.scripts['application'], encoding='utf-8')
    result = subprocess.run([node, '--check', str(script)], capture_output=True, text=True, encoding='utf-8')
    assert result.returncode == 0, result.stderr


def test_generated_viewer_starts_and_seeks_without_network(tmp_path):
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node unavailable for generated viewer startup validation')
    target = tmp_path / 'replay.html'
    write_replay(sample_evidence(), target)
    parsed = Scripts()
    parsed.feed(target.read_text(encoding='utf-8'))
    # Minimal DOM/canvas host for executing the generated public artifact.
    # It intentionally does not emulate layout or claim visual/browser QA.
    host = r'''
const assert = require('node:assert/strict');
const nodes = new Map();
function element(){return {textContent:'',value:'',children:[],handlers:{},
  classList:{add(){},toggle(){}},append(...xs){this.children.push(...xs);},
  replaceChildren(){this.children=[];},addEventListener(name,f){this.handlers[name]=f;},
  getBoundingClientRect(){return {width:800,height:350};},
  getContext(){return new Proxy({}, {get:(o,k)=>o[k]||(()=>{})});}};}
global.document={getElementById(id){if(!nodes.has(id))nodes.set(id,element());return nodes.get(id);},createElement:element};
global.window={devicePixelRatio:1,addEventListener(){}};
global.requestAnimationFrame=()=>{};
document.getElementById('event-filter').value='all';
document.getElementById('rate').value='4';
'''
    host += "document.getElementById('evidence-data').textContent=" + json.dumps(parsed.scripts['evidence-data']) + ';\n'
    checks = r'''
assert.equal(document.getElementById('status').textContent,'安全终止');
assert.match(document.getElementById('scope').textContent,/外部 MCP 调用.*模型身份未核验/);
const range=document.getElementById('timeline');range.value='1';range.handlers.input();
assert.match(document.getElementById('pose').textContent,/0\.10 \/ 0\.00/);
assert.equal(document.getElementById('events').children.length,1);
document.getElementById('search').value='no-match';document.getElementById('search').handlers.input();
assert.equal(document.getElementById('events').children[0].textContent,'当前时刻没有匹配事件');
'''
    script = tmp_path / 'start-viewer.cjs'
    script.write_text(host + parsed.scripts['application'] + checks, encoding='utf-8')
    result = subprocess.run([node, str(script)], capture_output=True, text=True, encoding='utf-8')
    assert result.returncode == 0, result.stderr


def run_artifact(tmp_path, evidence, checks):
    """Run the actual portable HTML's application script in an offline host.

    Canvas commands are retained to check the plotted evidence; this is not a
    browser layout emulator and makes no pixel-level visual QA claim.
    """
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node unavailable for generated artifact behavior validation')
    target = tmp_path / 'artifact.html'
    write_replay(evidence, target)
    parsed = Scripts()
    parsed.feed(target.read_text(encoding='utf-8'))
    host = r'''
const assert = require('node:assert/strict');
const nodes = new Map();
function element(tag='div') {
  const el={tagName:tag,textContent:'',value:'',children:[],handlers:{},draws:[],
    classList:{add(){},toggle(){}},append(...xs){this.children.push(...xs);},
    replaceChildren(...xs){this.children=[...xs];},
    addEventListener(name,f){this.handlers[name]=f;},
    getBoundingClientRect(){return {width:800,height:this.id==='map'?350:140};}};
  let points=[],dash=[];
  el.context=new Proxy({
    beginPath(){points=[];},moveTo(x,y){points.push(['M',x,y]);},
    lineTo(x,y){points.push(['L',x,y]);},setLineDash(xs){dash=[...xs];},
    stroke(){el.draws.push({type:'stroke',points:[...points],color:this.strokeStyle,dash:[...dash]});},
    fillText(text,x,y){el.draws.push({type:'text',text,x,y});},
    clearRect(){el.draws=[];}
  },{get:(o,k)=>k in o?o[k]:(()=>{})});
  el.getContext=()=>el.context;
  return el;
}
global.document={getElementById(id){if(!nodes.has(id)){const el=element();el.id=id;nodes.set(id,el);}return nodes.get(id);},createElement:element};
global.window={devicePixelRatio:1,addEventListener(){}};
global.requestAnimationFrame=()=>{};
document.getElementById('event-filter').value='all';
document.getElementById('rate').value='4';
function seek(t){const el=document.getElementById('timeline');el.value=String(t);el.handlers.input();}
function selectPlan(id){const el=document.getElementById('plan-select');el.value=id;el.handlers.change();}
function bodyText(el){return el.textContent+' '+el.children.map(bodyText).join(' ');}
'''
    host += "document.getElementById('evidence-data').textContent=" + json.dumps(parsed.scripts['evidence-data']) + ';\n'
    script = tmp_path / 'artifact-check.cjs'
    script.write_text(host + parsed.scripts['application'] + checks, encoding='utf-8')
    result = subprocess.run([node, str(script)], capture_output=True, text=True, encoding='utf-8')
    assert result.returncode == 0, result.stderr


def planned_evidence():
    evidence = sample_evidence()
    evidence['summary']['sim_time_sec'] = 5.
    evidence['plans'] = [
        dict(schema_version=1, plan_id='p1', route_id='r1', target='panel_a',
             route='upper', parameterization='spatial', coordinate_frame='world',
             sim_time_sec=1., observation_frame=1,
             source_observation=dict(source='simulated_sensor', observed_at=1.),
             anchor_pose=dict(x=.1, y=0., yaw_rad=.2),
             points_xy=[[.1, 0.], [4., 2.], [12., 0.]], status='PLANNED',
             reason=None, blocked_by=[], rationale='先巡检 panel_a', event_id=2),
        dict(schema_version=1, plan_id='p2', route_id=None, target='pump_b',
             route='direct', parameterization='spatial', coordinate_frame='world',
             sim_time_sec=3., observation_frame=3,
             source_observation=dict(source='simulated_sensor', observed_at=3.),
             anchor_pose=dict(x=12., y=0., yaw_rad=0.),
             points_xy=[[12., 0.], [24., 0.]], status='REJECTED',
             reason='PATH_BLOCKED', blocked_by=['blocked_aisle'],
             rationale='</script><img src=x onerror=alert(1)>', event_id=3),
        dict(schema_version=1, plan_id='p3', route_id='r3', target='pump_b',
             route='upper', parameterization='spatial', coordinate_frame='world',
             sim_time_sec=4., observation_frame=4, source_observation=None,
             anchor_pose=None, points_xy=None, status='PLANNED', reason=None,
             blocked_by=[], rationale='改走 upper', event_id=4),
    ]
    evidence['events'] = [dict(event_id=p['event_id'], event='planning_decision',
        plan_id=p['plan_id'], sim_time_sec=p['sim_time_sec'], target=p['target'],
        rationale=p['rationale']) for p in evidence['plans']]
    evidence['trace'] = []
    for t, speed, steer, cte in [(1., .2, .1, .3), (2., None, None, None),
                                (3., .4, -.2, .1), (5., 0., 0., 0.)]:
        evidence['trace'].append(dict(sim_time_sec=t, plan_id='p1', route_id='r1',
            state=dict(x=t, y=0., yaw_rad=0., speed_mps=speed),
            action=dict(throttle=0., brake=1., steer_left_normalized=steer),
            geometric_cte_m=cte, obstacles=[]))
    return evidence


def test_plans_are_time_gated_and_rejected_path_is_never_shown_as_executed(tmp_path):
    run_artifact(tmp_path, planned_evidence(), r'''
assert.match(document.getElementById('plan-detail').textContent,/当前时刻尚无规划/);
assert.equal(document.getElementById('plan-select').children.length,1);
assert.equal(document.getElementById('events').children[0].textContent,'当前时刻没有匹配事件');
seek(1);
assert.equal(document.getElementById('plan-select').children.length,2);
assert.match(document.getElementById('plan-detail').textContent,/p1.*spatial/s);
assert.ok(!document.getElementById('plan-detail').textContent.includes('PATH_BLOCKED'));
assert.ok(document.getElementById('map').draws.some(x=>x.type==='stroke'&&x.color==='#79b7ff'&&x.dash.length));
assert.ok(!document.getElementById('map').draws.some(x=>x.type==='stroke'&&x.color==='#ff9299'));
seek(3);
assert.equal(document.getElementById('plan-select').children.length,3);
assert.match(document.getElementById('plan-detail').textContent,/拒绝.*未执行.*PATH_BLOCKED/s);
assert.match(document.getElementById('plan-detail').textContent,/blocked_aisle/);
assert.ok(document.getElementById('map').draws.some(x=>x.type==='stroke'&&x.color==='#ff9299'&&x.dash.length));
assert.ok(document.getElementById('map').draws.some(x=>x.type==='stroke'&&x.color==='#61dcc3'&&x.dash.length===0));
selectPlan('p1');
assert.match(document.getElementById('plan-detail').textContent,/历史计划/);
assert.match(document.getElementById('plan-detail').textContent,/simulated_sensor/);
seek(4);
assert.equal(document.getElementById('plan-select').value,'p1');
document.getElementById('plan-jump').handlers.click();
assert.match(document.getElementById('clock').textContent,/1\.00 \/ 5\.00/);
assert.equal(document.getElementById('plan-select').children.length,2);
seek(0);
assert.equal(document.getElementById('plan-select').children.length,1);
assert.ok(!document.getElementById('plan-detail').textContent.includes('p1'));
assert.ok(!document.getElementById('map').draws.some(x=>x.type==='stroke'&&x.color==='#79b7ff'));
''')


def test_charts_follow_cursor_and_preserve_missing_samples_as_gaps(tmp_path):
    run_artifact(tmp_path, planned_evidence(), r'''
seek(3);
for(const id of ['speed-chart','steer-chart','cte-chart']) {
  const draws=document.getElementById(id).draws;
  const series=draws.find(x=>x.type==='stroke'&&x.color==='#61dcc3');
  assert.equal(series.points.filter(x=>x[0]==='M').length,2);
  assert.equal(series.points.filter(x=>x[0]==='L').length,0);
  const cursor=draws.find(x=>x.type==='stroke'&&x.color==='#ffd28e');
  assert.ok(cursor.points[0][1]>400&&cursor.points[0][1]<600);
}
seek(2);
assert.equal(document.getElementById('motion').textContent,'— m/s · 0.0°');
assert.equal(document.getElementById('cte').textContent,'— m');
assert.match(document.getElementById('recorded-at').textContent,/2\.00/);
seek(4);selectPlan('p3');
assert.match(document.getElementById('plan-detail').textContent,/未记录参考路径/);
assert.match(document.getElementById('plan-detail').textContent,/观测来源.*未记录/s);
assert.match(document.getElementById('recorded-at').textContent,/3\.00/);
''')


@pytest.mark.parametrize('normalized_by_loader', [False, True])
def test_legacy_evidence_does_not_infer_reference_paths_or_model_identity(tmp_path, normalized_by_loader):
    evidence = sample_evidence()
    evidence['trace'] = []
    if normalized_by_loader:
        evidence['plans'] = []
        evidence['plan_evidence_available'] = False
    run_artifact(tmp_path, evidence, r'''
assert.match(document.getElementById('plan-detail').textContent,/未记录规划快照/);
assert.equal(document.getElementById('plan-select').disabled,true);
assert.match(document.getElementById('scope').textContent,/模型身份未核验/);
assert.match(document.getElementById('verification-detail').textContent,/未提供外部核验/);
assert.ok(!document.getElementById('map').draws.some(x=>x.type==='stroke'&&x.color==='#79b7ff'));
assert.equal(document.getElementById('play').disabled,true);
assert.match(document.getElementById('receipt-detail').textContent,/未记录完成时刻回执.*不为旧记录补造/);
assert.match(document.getElementById('budget-detail').textContent,/未记录预算快照.*不.*反推预算/);
''')


def test_external_verification_is_labeled_offline_and_does_not_change_runtime_flag(tmp_path):
    evidence = planned_evidence()
    evidence['external_verification'] = dict(status='matched', model='agnes-3.0-flash',
        tool_pairs=14, report_path='reports/核验.md', run_sha256='a'*64,
        cross_check_sha256='b'*64)
    run_artifact(tmp_path, evidence, r'''
assert.match(document.getElementById('scope').textContent,/附外部离线核验说明/);
assert.match(document.getElementById('verification-detail').textContent,/agnes-3\.0-flash/);
assert.match(document.getElementById('verification-detail').textContent,/14/);
assert.match(document.getElementById('verification-detail').textContent,/model_call_verified = false/);
assert.match(document.getElementById('verification-detail').textContent,/不构成运行时身份认证/);
assert.equal(JSON.parse(document.getElementById('evidence-data').textContent).summary.model_call_verified,false);
seek(3);
assert.match(document.getElementById('plan-detail').textContent,/<img src=x onerror=alert\(1\)>/);
assert.equal(document.getElementById('plan-detail').children.length,0);
''')


def test_historical_completion_receipt_cannot_override_final_failure_or_create_early_event(tmp_path):
    evidence = sample_evidence()
    evidence['summary'].update(sim_time_sec=300., termination_reason='TIME_BUDGET_EXHAUSTED',
        budget=dict(sim_time_sec=300., remaining_sim_sec=0., deadline_sim_sec=300.,
                    remaining_execution_operations=77, max_execution_operations=80,
                    clock_policy='fixed_step_motion_plus_measured_idle_wall_after_first_plan',
                    snapshot_phase='tool_return'),
        tool_budget=dict(counted_requests=14, max_requests=120, remaining_requests=106))
    receipt = dict(receipt_id='completion-1', sim_time_sec=258.676, frame_id=6000,
                   criteria_met_at_completion=True, deadline_sim_sec=300.,
                   inspection_references=[dict(target=target) for target in ('panel_a', 'pump_b', 'exit_c')])
    evidence['completion_receipt'] = receipt
    evidence['summary']['completion_receipt'] = receipt
    evidence['events'] = [dict(event_id=1, event='mission_completion_evaluated',
                               sim_time_sec=258.676, receipt=receipt)]
    run_artifact(tmp_path, evidence, r'''
assert.equal(document.getElementById('status').textContent,'安全终止');
assert.match(document.getElementById('outcome-detail').textContent,/TIME_BUDGET_EXHAUSTED/);
assert.match(document.getElementById('receipt-detail').textContent,/历史.*258\.68.*当时条件满足/s);
assert.match(document.getElementById('receipt-detail').textContent,/不覆盖后续超时.*最终判定仍为：安全终止/s);
assert.match(document.getElementById('budget-detail').textContent,/最终预算快照.*300\.00.*剩余 0\.00/s);
assert.match(document.getElementById('budget-detail').textContent,/工具预算（独立于执行操作）/);
assert.ok(!bodyText(document.getElementById('events')).includes('完成时刻评估'));
seek(258.675);
assert.ok(!bodyText(document.getElementById('events')).includes('完成时刻评估'));
seek(258.676);
assert.match(bodyText(document.getElementById('events')),/完成时刻评估（历史）.*历史回执不覆盖最终判定/s);
assert.equal(document.getElementById('status').textContent,'安全终止');
''')
