"""Offline, dependency-free HTML replay of actual Mission evidence."""
from __future__ import annotations

import json
from pathlib import Path


def write_replay(evidence: dict, path: Path) -> None:
    """Write one portable HTML file; embedded evidence is inert JSON, never HTML.

    The viewer replays recorded samples. It does not simulate missing samples,
    infer model authorization, or promote safe termination to mission success.
    Invalid JSON numbers fail before a file is written.
    """
    if not isinstance(evidence, dict):
        raise TypeError('evidence must be a dictionary')
    payload = json.dumps(evidence, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
    payload = (payload.replace('&', '\\u0026').replace('<', '\\u003c')
               .replace('>', '\\u003e').replace('\u2028', '\\u2028').replace('\u2029', '\\u2029'))
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(_HTML.replace('EVIDENCE_JSON_TOKEN', payload), encoding='utf-8')


_HTML = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>EmbodiedGuard · 巡检执行回放</title>
<style>
:root{color-scheme:dark;--bg:#101620;--card:#192330;--ink:#e9f0f5;--muted:#9eafc0;--line:#304053;--teal:#61dcc3;--gold:#ffd28e;--red:#ff9299}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:system-ui,"Microsoft YaHei",sans-serif;font-size:14px;line-height:1.6}
main{max-width:1440px;margin:auto;padding:28px}header{display:flex;justify-content:space-between;gap:24px;align-items:center}h1{font-size:25px;letter-spacing:.01em;margin:0;font-weight:650}h2{font-size:15px;margin:0;font-weight:600}.sub{color:var(--muted);margin:3px 0 0;font-size:12px}.status{border:1px solid var(--line);border-radius:30px;padding:7px 16px;font-weight:650;white-space:nowrap}.success{color:var(--teal)}.terminated{color:var(--gold)}.incomplete{color:var(--red)}
.scope{margin:18px 0;border:1px solid #65503b;background:#302920;color:#ffd9a3;border-radius:10px;padding:10px 14px}.scope small{color:#c7bcae;display:block;margin-top:2px}
.metrics{display:grid;grid-template-columns:repeat(5,1fr);gap:10px;margin:16px 0}.metric{border:1px solid var(--line);border-radius:12px;background:var(--card);padding:13px 16px}.metric .label{color:var(--muted);font-size:12px}.metric strong{display:block;font-size:24px;font-weight:600;margin-top:2px}.metric .note{font-size:11px;color:var(--muted)}
.layout{display:grid;grid-template-columns:minmax(0,1fr) 355px;gap:16px}.panel{background:var(--card);border:1px solid var(--line);border-radius:14px;overflow:hidden}.panel-head{padding:14px 18px;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid var(--line);gap:12px}.map-wrap{height:370px;min-height:280px;padding:12px;position:relative}canvas{display:block;width:100%;height:100%}.legend{display:flex;flex-wrap:wrap;gap:18px;color:var(--muted);font-size:12px;padding:0 18px 12px}.legend span:before{content:"";display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px;background:var(--teal)}.legend .obstacle:before{background:var(--red)}.legend .point:before{background:#b3a3fc}
.controls{padding:14px 18px;border-top:1px solid var(--line)}.control-row{display:flex;align-items:center;gap:12px;margin-bottom:12px}button,input,select{font:inherit;color:var(--ink)}button,select{border:1px solid #42566b;background:#253446;border-radius:7px;padding:6px 12px;cursor:pointer}button:hover{border-color:var(--teal)}button:disabled{opacity:.4;cursor:default}#play{background:var(--teal);color:#0b302b;border-color:var(--teal);min-width:74px;font-weight:600}input[type=range]{width:100%;accent-color:var(--teal);margin:0}#clock{font-variant-numeric:tabular-nums;flex:1;color:var(--muted)}.live{padding:12px 18px;display:grid;grid-template-columns:repeat(4,1fr);gap:12px;border-top:1px solid var(--line)}.live label{font-size:11px;color:var(--muted);display:block}.live output{font-family:ui-monospace,monospace;font-size:13px}.details{padding:12px 18px;font-size:12px;color:var(--muted);border-top:1px solid var(--line)}
.filters{padding:12px;display:flex;gap:8px;border-bottom:1px solid var(--line)}.filters select{width:132px;font-size:12px}.filters input{width:100%;min-width:0;background:#111b28;border:1px solid var(--line);border-radius:7px;padding:7px 9px;font-size:12px}.events{height:670px;overflow:auto;padding:8px 12px}.event{padding:11px 10px;border-bottom:1px solid var(--line);border-radius:6px}.event.current{background:#253b43}.event-head{display:flex;align-items:center;gap:8px;justify-content:space-between}.event-head button{font-size:11px;padding:2px 6px;white-space:nowrap}.event-title{font-size:12px;font-weight:600}.event-summary{color:var(--muted);font-size:12px;overflow-wrap:anywhere;margin:6px 0 0}.event details{font-size:11px;margin-top:6px;color:var(--muted)}pre{white-space:pre-wrap;overflow-wrap:anywhere;margin:6px 0;max-height:200px;overflow:auto}.empty{color:var(--muted);padding:24px 10px}.foot{font-size:11px;color:var(--muted);margin:15px 0}.mono{font-family:ui-monospace,monospace}#outcome-detail{max-width:900px;overflow-wrap:anywhere}
.legend .reference:before{background:#79b7ff;border-radius:0;width:18px;height:0;border-top:2px dashed #79b7ff}.legend .rejected:before{background:transparent;border-radius:0;width:18px;height:0;border-top:2px dashed var(--red)}.plans{padding:12px 18px;border-top:1px solid var(--line)}.plan-row{display:flex;gap:10px;align-items:center}.plan-row select{min-width:0;flex:1}.plan-row button{white-space:nowrap}.plan-detail{white-space:pre-wrap;overflow-wrap:anywhere;color:var(--muted);font-size:12px;margin:9px 0 0;max-height:205px;overflow:auto}.charts{padding:12px 18px;border-top:1px solid var(--line)}.chart-wrap{height:112px;margin-top:5px}.chart-title{font-size:12px;color:var(--muted)}#verification-detail{white-space:pre-wrap;overflow-wrap:anywhere;color:#c7bcae;font-size:11px}#recorded-at{padding:0 18px 10px}.scope details{margin-top:6px}.scope summary{font-size:12px;cursor:pointer}
@media(max-width:1000px){.layout{grid-template-columns:1fr}.events{height:300px}.metrics{grid-template-columns:repeat(3,1fr)}}@media(max-width:580px){main{padding:16px}header{align-items:flex-start;gap:10px}h1{font-size:20px}.status{padding:5px 9px;font-size:12px}.metrics{grid-template-columns:repeat(2,1fr)}.map-wrap{height:300px}.live{grid-template-columns:repeat(2,1fr)}.control-row{gap:7px}.scope{font-size:12px}}
</style></head><body><main>
<header><div><h1>EmbodiedGuard <span style="color:var(--muted);font-weight:400">/ 执行回放</span></h1><p class="sub" id="run-meta"></p></div><div id="status" class="status">读取证据</div></header>
<div class="scope"><strong id="scope">二维运动学仿真 · 模型身份未核验</strong><small>仅回放记录；温度与异常为 synthetic_fixture 合成数据，不声明检测准确率。仿真运动学不代表实机验证；模型费用未核实。</small><details><summary>模型来源与证据范围</summary><div id="verification-detail"></div></details></div>
<section class="metrics" id="metrics" aria-label="本次运行指标"></section>
<section class="panel" style="margin-bottom:16px" aria-label="最终预算与历史完成回执"><div class="details" id="budget-detail"></div><div class="details" id="receipt-detail"></div></section>
<div class="layout"><section class="panel"><div class="panel-head"><h2>巡检场景</h2><span class="sub" id="sample-count"></span></div>
<div class="map-wrap"><canvas id="map" aria-label="二维轨迹、障碍、车体朝向和巡检点"></canvas></div>
<div class="legend"><span>已执行轨迹 / 车体</span><span class="reference">规划参考（空间路径）</span><span style="color:#ffd28e">橙线：只读短期预测</span><span class="rejected">被拒候选（未执行）</span><span class="obstacle">合成障碍</span><span class="point">巡检点</span><span style="color:#8da0b7">坐标单位 m</span></div>
<div class="controls"><div class="control-row"><button id="play" type="button">播放</button><button id="reset" type="button">回到起点</button><span id="clock"></span><label class="sub" for="rate">速度</label><select id="rate"><option value="1">1×</option><option value="2">2×</option><option value="4" selected>4×</option><option value="8">8×</option></select></div><input id="timeline" type="range" min="0" max="0" value="0" step="0.001" aria-label="回放时间"></div>
<div class="live"><div><label>当前位置 x / y</label><output id="pose"></output></div><div><label>速度 / 航向</label><output id="motion"></output></div><div><label>油门 / 制动 / 转向</label><output id="action"></output></div><div><label>几何横向误差</label><output id="cte"></output></div></div>
<p class="sub" id="recorded-at"></p>
<div class="plans"><div class="plan-row"><label class="sub" for="plan-select">规划快照</label><select id="plan-select" aria-label="选择当前时刻已产生的规划"><option value="">自动选择最近规划</option></select><button id="plan-jump" type="button">跳到规划时刻</button></div><div id="plan-detail" class="plan-detail"></div></div>
<div class="plans"><div class="plan-row"><label class="sub" for="preview-select">短期轨迹预测</label><select id="preview-select" aria-label="选择已有的只读预测"></select><button id="preview-jump" type="button">跳到预测起点</button></div><div id="preview-detail" class="plan-detail"></div><details><summary>意图、动作与恢复证据</summary><pre id="action-contract-detail"></pre></details></div>
<div class="charts"><div class="chart-title">实际速度 · m/s</div><div class="chart-wrap"><canvas id="speed-chart" aria-label="记录的实际速度与时间游标"></canvas></div><div class="chart-title">实际转向 · 归一化，左为正</div><div class="chart-wrap"><canvas id="steer-chart" aria-label="记录的归一化转向与时间游标"></canvas></div><div class="chart-title">几何横向误差 · m</div><div class="chart-wrap"><canvas id="cte-chart" aria-label="记录的横向误差与时间游标"></canvas></div><p class="sub">横轴为仿真秒；黄线为回放游标。只画游标之前的实际记录，缺失值断开，不补零。</p></div>
<div class="details" id="outcome-detail"></div></section>
<aside class="panel"><div class="panel-head"><h2>决策与执行证据</h2><span class="sub" id="event-count"></span></div><div class="filters"><select id="event-filter" aria-label="事件类型"><option value="all">全部事件</option><option value="decision">规划 / 决策</option><option value="execution">执行 / 巡检</option><option value="safety">异常 / 停止</option><option value="recovery">恢复 / 等待</option></select><input id="search" type="search" placeholder="搜索原因、目标、操作 ID" aria-label="搜索事件"></div><div id="events" class="events"></div></aside></div>
<p class="foot">单文件离线回放 · 画面按记录时间选择实际样本，不插补缺失状态。顶部指标为整轮最终结果；规划、事件和曲线仅显示游标之前的记录。空间路径没有预设时间或速度，不等同于时序轨迹。安全终止不计为任务成功。“—”表示未记录。</p>
</main><script type="application/json" id="evidence-data">EVIDENCE_JSON_TOKEN</script>
<script>
'use strict';
const evidence = JSON.parse(document.getElementById('evidence-data').textContent);
const summary = evidence.summary || {}, metrics = summary.metrics || {};
const trace = Array.isArray(evidence.trace) ? evidence.trace : [];
const events = Array.isArray(evidence.events) ? evidence.events : [];
const plansRecorded = Array.isArray(evidence.plans) && evidence.plan_evidence_available !== false;
const plans = plansRecorded ? evidence.plans : [];
const previews = Array.isArray(evidence.trajectory_previews) ? evidence.trajectory_previews : [];
const previewChecks = Array.isArray(evidence.preview_checks) ? evidence.preview_checks : [];
const contracts = Array.isArray(evidence.action_contracts) ? evidence.action_contracts : [];
const recoveryDecisions = Array.isArray(evidence.recovery_decisions) ? evidence.recovery_decisions : [];
const $ = id => document.getElementById(id);
const finite = x => typeof x === 'number' && Number.isFinite(x);
const fmt = (x,n=2) => finite(x) ? x.toFixed(n) : '—';
const number = (x,fallback=0) => finite(x) ? x : fallback;
const lastTime = trace.length ? number(trace[trace.length-1].sim_time_sec) : 0;
const duration = Math.max(lastTime,number(summary.sim_time_sec),...events.map(e=>number(e.sim_time_sec)),...plans.map(p=>number(p.sim_time_sec)));
let time = 0, frame = -1, playing = false, previousTick = null;
let selectedPlanId = '', knownPlans = [], activePlan = null;
let selectedPreviewId = '', activePreview = null;
const source = String(summary.decision_source || 'unknown');
const external = source === 'external_mcp_client';
const sourceLabel = external ? '外部 MCP 调用' : /fixed|rule|baseline|recovery/.test(source) ? '固定规则演示' : '未识别决策来源';
const offlineVerification=evidence.external_verification;
const attachedVerification=offlineVerification?.status==='matched' && typeof offlineVerification.model==='string'
  && finite(offlineVerification.tool_pairs) && offlineVerification.tool_pairs>=0
  && typeof offlineVerification.report_path==='string'
  && /^[a-f0-9]{64}$/i.test(offlineVerification.run_sha256||'')
  && /^[a-f0-9]{64}$/i.test(offlineVerification.cross_check_sha256||'');
$('scope').textContent = '二维运动学仿真 · ' + sourceLabel + ' · ' + (attachedVerification?'附外部离线核验说明':'模型身份未核验');
$('verification-detail').textContent=attachedVerification
  ? '由离线核验加载器提供：'+offlineVerification.model+'；匹配工具调用 '+offlineVerification.tool_pairs+' 对。\n报告：'+offlineVerification.report_path+'\nrun SHA-256：'+offlineVerification.run_sha256+'\n交叉核验 SHA-256：'+offlineVerification.cross_check_sha256+'\n此说明不构成运行时身份认证；工具内 model_call_verified = '+String(summary.model_call_verified??'未记录')+'。费用未核实。'
  : '未提供外部核验，或核验说明缺少绑定哈希。MCP 调用来源本身不能证明模型身份。工具内 model_call_verified = '+String(summary.model_call_verified??'未记录')+'。费用未核实。';
$('run-meta').textContent = '场景 ' + String(evidence.scenario ?? '未记录') + ' · seed ' + String(evidence.seed ?? '—') + ' · ' + source;
const outcome = summary.task_success === true ? '任务成功' : summary.safe_termination === true ? '安全终止' : '未完成';
$('status').textContent = outcome;
$('status').classList.add(summary.task_success === true ? 'success' : summary.safe_termination === true ? 'terminated' : 'incomplete');
function card(label,value,note) { const box=document.createElement('div');box.className='metric';const a=document.createElement('div');a.className='label';a.textContent=label;const b=document.createElement('strong');b.textContent=value;const c=document.createElement('div');c.className='note';c.textContent=note;box.append(a,b,c);$('metrics').append(box); }
const completed=Array.isArray(summary.completed_targets)?summary.completed_targets:[];
const missing=Array.isArray(summary.missing_targets)?summary.missing_targets:[];
const violationKeys=['collisions','boundary_violations','speed_violations'];
const violationValue=violationKeys.every(k=>finite(metrics[k]))?String(violationKeys.reduce((s,k)=>s+metrics[k],0)):'—';
card('巡检完成',completed.length+' / '+Object.keys(evidence.checkpoints||{}).length,'独立巡检证据');
card('安全违规',violationValue,'碰撞 / 越界 / 超速记录');
card('仿真时间',fmt(summary.sim_time_sec,1)+' s','仿真时钟，非模型时延');
card('跟踪 RMSE',fmt(metrics.cte_rmse_m,3)+' m','有记录的几何误差样本');
card('执行路程',fmt(metrics.path_length_m,1)+' m','安全干预 '+String(metrics.safety_interventions??'—')+' 次');
$('outcome-detail').textContent='整轮终态：'+outcome+'；未完成：'+(missing.length?missing.join('、'):'无')+'；终止原因：'+String(summary.termination_reason??'无')+'。模型证据：'+(attachedVerification?'已附外部离线核验说明':'模型身份未核验')+'；费用：未核实。';
const budget=summary.budget, toolBudget=summary.tool_budget;
$('budget-detail').textContent=budget
  ? '最终预算快照 @ '+fmt(budget.sim_time_sec,2)+' s：剩余 '+fmt(budget.remaining_sim_sec,2)+' s / 截止 '+fmt(budget.deadline_sim_sec,2)+' s；剩余执行操作 '+String(budget.remaining_execution_operations??'未记录')+' / '+String(budget.max_execution_operations??'未记录')+'。\n时钟策略：'+String(budget.clock_policy??'未记录')+'；快照阶段：'+String(budget.snapshot_phase??'未记录')+'。'+(toolBudget?'\n工具预算（独立于执行操作）：'+JSON.stringify(toolBudget):' 工具调用预算未记录。')
  : '该证据未记录预算快照；不从时间或执行次数反推预算。';
const completionReceipt=evidence.completion_receipt??summary.completion_receipt;
$('receipt-detail').textContent=completionReceipt
  ? '完成时刻回执（历史）：'+String(completionReceipt.receipt_id??'未记录')+' @ '+fmt(completionReceipt.sim_time_sec,2)+' s，帧 '+String(completionReceipt.frame_id??'未记录')+'；当时条件'+(completionReceipt.criteria_met_at_completion===true?'满足':'未满足')+'；截止 '+fmt(completionReceipt.deadline_sim_sec,2)+' s；关联巡检证据 '+String(completionReceipt.inspection_references?.length??'未记录')+' 项。\n此回执只记录当时的判定，不覆盖后续超时、停止或安全失败。整轮最终判定仍为：'+outcome+'。回执的本地一致性不构成模型身份认证或实机验证。'
  : '未记录完成时刻回执；不为旧记录补造历史判定。';
$('sample-count').textContent=trace.length+' 个状态样本';
$('timeline').max=String(duration);$('play').disabled=trace.length===0||duration===0;
const labels={mission_started:'任务开始',planning_decision:'规划决策',execution_started:'开始执行',execution_finished:'执行反馈',inspection_verified:'巡检验证',mission_completion_evaluated:'完成时刻评估（历史）',safety_rejection:'安全拒绝',safety_violation:'安全违规',sensor_reobservation:'重新观测',waited:'等待',stopped:'停止'};
function category(event){const name=String(event.event||'');if(/safety|stop/.test(name))return 'safety';if(/observ|wait/.test(name))return 'recovery';if(/decision|plan/.test(name))return 'decision';return 'execution';}
let eventRows=[], renderedEventKey='';
function renderEvents(){const filter=$('event-filter').value, query=$('search').value.toLowerCase();const selected=events.filter(e=>finite(e.sim_time_sec)&&e.sim_time_sec<=time+1e-9&&(filter==='all'||category(e)===filter)&&JSON.stringify(e).toLowerCase().includes(query));const key=JSON.stringify([filter,query,selected.length]);if(key===renderedEventKey){highlightEvents();return;}renderedEventKey=key;$('events').replaceChildren();eventRows=[];$('event-count').textContent=selected.length+' / '+events.length;
  for(const event of selected){const row=document.createElement('div');row.className='event';const head=document.createElement('div');head.className='event-head';const title=document.createElement('span');title.className='event-title';title.textContent=labels[event.event]||String(event.event||'事件');const jump=document.createElement('button');jump.type='button';jump.textContent=fmt(event.sim_time_sec,2)+' s';jump.addEventListener('click',()=>{pause();setTime(number(event.sim_time_sec));});head.append(title,jump);
    const desc=document.createElement('div');desc.className='event-summary';desc.textContent=event.event==='mission_completion_evaluated'?'当时条件'+(event.receipt?.criteria_met_at_completion===true?'满足':'未满足')+'；历史回执不覆盖最终判定。':String(event.rationale||event.reason||event.target||event.status||event.operation_id||event.verdict?.reason||'');const detail=document.createElement('details');const heading=document.createElement('summary');heading.textContent='查看原始证据';const raw=document.createElement('pre');raw.textContent=JSON.stringify(event,null,2);detail.append(heading,raw);row.append(head,desc,detail);$('events').append(row);eventRows.push({row,time:number(event.sim_time_sec)});}
  if(!selected.length){const empty=document.createElement('div');empty.className='empty';empty.textContent='当前时刻没有匹配事件';$('events').append(empty);}highlightEvents();}
function highlightEvents(){let newest=-Infinity;for(const entry of eventRows)if(entry.time<=time)newest=Math.max(newest,entry.time);for(const entry of eventRows)entry.row.classList.toggle('current',entry.time===newest);}
function planHasPoints(plan){return Array.isArray(plan?.points_xy)&&plan.points_xy.length>1;}
function renderPlans(){
  knownPlans=plans.filter(p=>finite(p.sim_time_sec)&&p.sim_time_sec<=time+1e-9).sort((a,b)=>a.sim_time_sec-b.sim_time_sec);
  if(selectedPlanId&&!knownPlans.some(p=>p.plan_id===selectedPlanId))selectedPlanId='';
  const selector=$('plan-select');selector.replaceChildren();const automatic=document.createElement('option');automatic.value='';automatic.textContent='自动选择最近规划';selector.append(automatic);
  for(const plan of knownPlans){const option=document.createElement('option');option.value=String(plan.plan_id);option.textContent=fmt(plan.sim_time_sec,2)+' s · '+String(plan.target??'—')+' / '+String(plan.route??'—')+' · '+(plan.status==='REJECTED'?'拒绝，未执行':String(plan.status??'—'));selector.append(option);}
  selector.value=selectedPlanId;selector.disabled=knownPlans.length===0;
  activePlan=selectedPlanId?knownPlans.find(p=>p.plan_id===selectedPlanId):knownPlans[knownPlans.length-1]||null;
  $('plan-jump').disabled=!activePlan;
  if(!activePlan){$('plan-detail').textContent=plansRecorded?'当前时刻尚无规划。移动时间游标查看后续已记录规划。':'该旧证据未记录规划快照；仅显示实际执行轨迹，不反推参考路径。';return;}
  const p=activePlan, historical=p!==knownPlans[knownPlans.length-1];
  const mode=historical?'已选历史计划（不会改变执行记录）':selectedPlanId?'已选当前最近计划':'自动：当前最近计划';
  const status=p.status==='REJECTED'?'拒绝，未执行':p.status==='ARRIVED'?'规划时已到达目标':String(p.status??'未记录');
  $('plan-detail').textContent=mode+'\n'+String(p.plan_id??'—')+' · '+String(p.target??'—')+' / '+String(p.route??'—')+' · '+status+'\n时间 '+fmt(p.sim_time_sec,2)+' s；表示 '+String(p.parameterization??'未记录')+'；坐标系 '+String(p.coordinate_frame??'未记录')+'；route_id '+String(p.route_id??'无')+'\n原因 '+String(p.reason??'无')+'；阻塞对象 '+(Array.isArray(p.blocked_by)&&p.blocked_by.length?p.blocked_by.join('、'):'无记录')+'\n观测帧 '+String(p.observation_frame??'未记录')+'；观测来源 '+(p.source_observation?JSON.stringify(p.source_observation):'未记录')+'\n起始位姿 '+(p.anchor_pose?JSON.stringify(p.anchor_pose):'未记录')+'\n'+(planHasPoints(p)?p.points_xy.length+' 个参考点；空间路径没有预设时间或速度。':'未记录参考路径，不从实际轨迹重建。')+'\n公开决策理由：'+String(p.rationale??'未记录');
}
function frameAt(t){let low=0,high=trace.length-1,found=-1;while(low<=high){const mid=(low+high)>>1;if(number(trace[mid].sim_time_sec)<=t+1e-9){found=mid;low=mid+1;}else high=mid-1;}return found;}
function renderShadow(){
  const available=previews.filter(p=>finite(p.requested_sim_time_sec)&&p.requested_sim_time_sec<=time+1e-9);
  if(selectedPreviewId&&!available.some(p=>p.preview_id===selectedPreviewId))selectedPreviewId='';
  const selector=$('preview-select');selector.replaceChildren();const auto=document.createElement('option');auto.value='';auto.textContent='自动选择最近预测';selector.append(auto);
  for(const p of available){const item=document.createElement('option');item.value=p.preview_id;item.textContent=fmt(p.requested_sim_time_sec,2)+' s · '+String(p.operation_id)+' · '+(p.status==='available'?'条件预测':String(p.reason));selector.append(item);}
  selector.value=selectedPreviewId;selector.disabled=!available.length;
  activePreview=(selectedPreviewId?available.find(p=>p.preview_id===selectedPreviewId):available[available.length-1])||null;
  $('preview-jump').disabled=!activePreview;
  if(!activePreview){$('preview-detail').textContent=previews.length?'当前游标之前没有预测。':'未附只读预测证据；不从实际轨迹补造预测。';}
  else if(activePreview.status!=='available'){$('preview-detail').textContent=String(activePreview.preview_id)+'：预测不可用 · '+String(activePreview.reason)+'。无猜测轨迹，不授权运动。';}
  else{const p=activePreview,c=previewChecks.find(c=>c.preview_id===p.preview_id),errs=(c?.errors||[]).filter(e=>e.sim_time_sec<=time+1e-9);const end=p.samples?.[p.samples.length-1]?.sim_time_sec;
    $('preview-detail').textContent=String(p.preview_id)+' · '+String(p.operation_id)+' · '+(finite(end)&&time>end?'历史预测，窗口已过':'预测窗口')+'\n来源观测 '+JSON.stringify(p.source_observation)+'\n真实测量锚点 '+JSON.stringify(p.anchor_state)+'\n世界坐标 / 锚点局部坐标 · t,x,y,yaw,v；'+fmt(p.sampled_horizon_sec,2)+' s 条件窗口\n截至游标匹配 '+errs.length+' 个实际样本；位置偏差最大 '+(errs.length?fmt(Math.max(...errs.map(e=>e.position_error_m)),4):'—')+' m\n橙线：控制器和运动学的只读预测；绿线：实际运动。安全中断可能使两者分离。\n不预测未来障碍、传感器故障和执行段末制动；不授权运动，不声明实机预测准确率。';}
  const known=contracts.filter(c=>finite(c.request_sim_time_sec)&&c.request_sim_time_sec<=time+1e-9);
  const latest=known[known.length-1];
  const decision=recoveryDecisions.filter(d=>d.evidence_refs?.some(r=>events.some(e=>e.event_id===r.event_id&&e.event==='tool_call_started'&&e.sim_time_sec<=time+1e-9))).slice(-1)[0];
  const visibleContract=latest?{...latest}:null;
  if(visibleContract&&!events.some(e=>e.event_id===visibleContract.result_event_id&&e.sim_time_sec<=time+1e-9)){delete visibleContract.result_event_id;delete visibleContract.result_status;delete visibleContract.result_reason;delete visibleContract.result_ok;delete visibleContract.stopping_verified;}
  $('action-contract-detail').textContent=latest?JSON.stringify({action_contract:visibleContract,recovery_decision:decision??null},null,2):'未附实际工具请求契约；不补造 Agent 意图或解释。';
}
function setTime(t){time=Math.max(0,Math.min(duration,t));frame=frameAt(time);$('timeline').value=String(time);$('clock').textContent=fmt(time,2)+' / '+fmt(duration,2)+' s';const row=frame>=0?trace[frame]:null,state=row?.state||{},action=row?.action||{};
  $('pose').textContent=fmt(state.x)+' / '+fmt(state.y)+' m';$('motion').textContent=fmt(state.speed_mps)+' m/s · '+(finite(state.yaw_rad)?fmt(state.yaw_rad*180/Math.PI,1):'—')+'°';$('action').textContent=[action.throttle,action.brake,action.steer_left_normalized].map(x=>fmt(x)).join(' / ');$('cte').textContent=fmt(row?.geometric_cte_m,3)+' m';
  $('recorded-at').textContent=row?'显示已记录样本 @ '+fmt(row.sim_time_sec,2)+' s；plan_id '+String(row.plan_id??'未记录')+'；route_id '+String(row.route_id??'未记录')+'。回放游标不表示新观测。':'当前时刻没有状态样本。';renderPlans();renderShadow();renderEvents();draw();}
function pause(){playing=false;previousTick=null;$('play').textContent='播放';}
$('play').addEventListener('click',()=>{if(playing){pause();return;}if(time>=duration)setTime(0);playing=true;previousTick=null;$('play').textContent='暂停';});
$('reset').addEventListener('click',()=>{pause();setTime(0);});$('timeline').addEventListener('input',()=>{pause();setTime(Number($('timeline').value));});$('event-filter').addEventListener('change',renderEvents);$('search').addEventListener('input',renderEvents);
$('plan-select').addEventListener('change',()=>{selectedPlanId=$('plan-select').value;renderPlans();draw();});
$('plan-jump').addEventListener('click',()=>{if(activePlan){pause();setTime(activePlan.sim_time_sec);}});
$('preview-select').addEventListener('change',()=>{selectedPreviewId=$('preview-select').value;renderShadow();draw();});
$('preview-jump').addEventListener('click',()=>{if(activePreview){pause();setTime(activePreview.requested_sim_time_sec);}});
const canvas=$('map'), ctx=canvas.getContext('2d');
function draw(){const rect=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1,w=Math.max(1,rect.width),h=Math.max(1,rect.height);if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
  const b=Array.isArray(evidence.bounds)&&evidence.bounds.length===4?evidence.bounds:[-2,-6,40,6];const scale=Math.min((w-68)/Math.max(1,b[2]-b[0]),(h-56)/Math.max(1,b[3]-b[1]));const ox=(w-(b[2]-b[0])*scale)/2,oy=(h-(b[3]-b[1])*scale)/2;const X=x=>ox+(x-b[0])*scale,Y=y=>h-oy-(y-b[1])*scale;
  ctx.fillStyle='#111b28';ctx.fillRect(X(b[0]),Y(b[3]),(b[2]-b[0])*scale,(b[3]-b[1])*scale);ctx.lineWidth=1;ctx.strokeStyle='#273547';ctx.fillStyle='#76889c';ctx.font='10px system-ui';ctx.textAlign='center';
  for(let x=Math.ceil(b[0]/5)*5;x<=b[2];x+=5){ctx.beginPath();ctx.moveTo(X(x),Y(b[1]));ctx.lineTo(X(x),Y(b[3]));ctx.stroke();ctx.fillText(String(x),X(x),Y(b[1])+16);}
  for(let y=Math.ceil(b[1]/2)*2;y<=b[3];y+=2){ctx.beginPath();ctx.moveTo(X(b[0]),Y(y));ctx.lineTo(X(b[2]),Y(y));ctx.stroke();ctx.fillText(String(y),X(b[0])-16,Y(y)+3);}
  ctx.strokeStyle='#698094';ctx.strokeRect(X(b[0]),Y(b[3]),(b[2]-b[0])*scale,(b[3]-b[1])*scale);
  for(const plan of knownPlans){
    if(!planHasPoints(plan)||plan.coordinate_frame!=='world'||plan.parameterization!=='spatial')continue;
    ctx.save();ctx.globalAlpha=plan===activePlan?1:.22;ctx.strokeStyle=plan.status==='REJECTED'?'#ff9299':'#79b7ff';ctx.lineWidth=plan===activePlan?2.5:1.5;ctx.setLineDash(plan.status==='REJECTED'?[4,5]:[8,5]);ctx.beginPath();let open=false;
    for(const p of plan.points_xy){if(!Array.isArray(p)||!finite(p[0])||!finite(p[1])){open=false;continue;}if(open)ctx.lineTo(X(p[0]),Y(p[1]));else ctx.moveTo(X(p[0]),Y(p[1]));open=true;}ctx.stroke();ctx.setLineDash([]);ctx.restore();
  }
  if(activePreview?.status==='available'&&activePreview.role==='read_only_shadow'){
    ctx.save();ctx.strokeStyle='#ffd28e';ctx.lineWidth=3;ctx.setLineDash([3,3]);ctx.beginPath();let open=false;
    for(const s of activePreview.samples||[]){if(!finite(s.x)||!finite(s.y)){open=false;continue;}if(open)ctx.lineTo(X(s.x),Y(s.y));else ctx.moveTo(X(s.x),Y(s.y));open=true;}ctx.stroke();ctx.restore();
  }
  const done=new Set(events.filter(e=>e.event==='inspection_verified'&&number(e.sim_time_sec)<=time).map(e=>e.evidence?.target));
  for(const [name,p] of Object.entries(evidence.checkpoints||{})){if(!Array.isArray(p)||p.length<2)continue;ctx.fillStyle=done.has(name)?'#61dcc3':'#b3a3fc';ctx.beginPath();ctx.arc(X(p[0]),Y(p[1]),5,0,Math.PI*2);ctx.fill();ctx.font='11px system-ui';ctx.fillText(name+(done.has(name)?' ✓':''),X(p[0]),Y(p[1])+21);}
  if(frame>=0){ctx.strokeStyle='#61dcc3';ctx.lineWidth=2.5;ctx.setLineDash([]);ctx.beginPath();let open=false;for(let i=0;i<=frame;i++){const s=trace[i].state;if(!s||!finite(s.x)||!finite(s.y)){open=false;continue;}if(open)ctx.lineTo(X(s.x),Y(s.y));else ctx.moveTo(X(s.x),Y(s.y));open=true;}ctx.stroke();
    for(const ob of trace[frame].obstacles||[]){if(!finite(ob.x)||!finite(ob.y)||!finite(ob.radius))continue;ctx.fillStyle='rgba(255,146,153,.24)';ctx.strokeStyle='#ff9299';ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(X(ob.x),Y(ob.y),Math.max(2,ob.radius*scale),0,Math.PI*2);ctx.fill();ctx.stroke();ctx.fillStyle='#ffb8bc';ctx.font='10px system-ui';ctx.fillText(String(ob.id||'障碍'),X(ob.x),Y(ob.y)-ob.radius*scale-7);if(number(ob.vx)||number(ob.vy)){ctx.beginPath();ctx.moveTo(X(ob.x),Y(ob.y));ctx.lineTo(X(ob.x+number(ob.vx)),Y(ob.y+number(ob.vy)));ctx.stroke();}}
    const s=trace[frame].state;if(s&&finite(s.x)&&finite(s.y)&&finite(s.yaw_rad)){const r=number(evidence.plant_config?.radius_m,.45)*scale;ctx.save();ctx.translate(X(s.x),Y(s.y));ctx.rotate(-s.yaw_rad);ctx.fillStyle='rgba(97,220,195,.16)';ctx.strokeStyle='#61dcc3';ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(0,0,Math.max(4,r),0,Math.PI*2);ctx.fill();ctx.stroke();ctx.fillStyle='#8ff5df';ctx.beginPath();ctx.moveTo(Math.max(8,r+4),0);ctx.lineTo(-5,-4);ctx.lineTo(-2,0);ctx.lineTo(-5,4);ctx.closePath();ctx.fill();ctx.restore();}
  }else{ctx.fillStyle='#9eafc0';ctx.font='13px system-ui';ctx.fillText(trace.length?'等待首个记录样本':'本次没有状态轨迹',w/2,h/2);}
  drawChart('speed-chart',r=>r.state?.speed_mps,0,number(evidence.plant_config?.max_speed_mps,1.5));
  drawChart('steer-chart',r=>r.action?.steer_left_normalized,-1,1);
  drawChart('cte-chart',r=>r.geometric_cte_m,0,.1);
}
function drawChart(id,valueOf,defaultMin,defaultMax){
  const chart=$(id),c=chart.getContext('2d'),rect=chart.getBoundingClientRect(),dpr=window.devicePixelRatio||1,w=Math.max(1,rect.width),h=Math.max(1,rect.height);
  if(chart.width!==Math.round(w*dpr)||chart.height!==Math.round(h*dpr)){chart.width=Math.round(w*dpr);chart.height=Math.round(h*dpr);}c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);
  let lo=defaultMin,hi=defaultMax;for(let i=0;i<=frame;i++){const value=valueOf(trace[i]);if(finite(value)){lo=Math.min(lo,value);hi=Math.max(hi,value);}}
  const left=44,right=Math.max(left+1,w-15),top=10,bottom=Math.max(top+1,h-23),X=t=>left+Math.max(0,Math.min(duration,t))/Math.max(duration,.001)*(right-left),Y=v=>bottom-(v-lo)/Math.max(.001,hi-lo)*(bottom-top);
  c.strokeStyle='#304053';c.lineWidth=1;c.setLineDash([]);c.beginPath();c.moveTo(left,top);c.lineTo(left,bottom);c.lineTo(right,bottom);c.stroke();c.font='10px system-ui';c.fillStyle='#9eafc0';c.textAlign='right';c.fillText(fmt(hi,2),left-5,top+3);c.fillText(fmt(lo,2),left-5,bottom+3);c.textAlign='left';c.fillText('0 s',left,bottom+16);c.textAlign='right';c.fillText(fmt(duration,1)+' s',right,bottom+16);
  c.strokeStyle='#61dcc3';c.lineWidth=1.8;c.beginPath();let open=false,points=0;for(let i=0;i<=frame;i++){const row=trace[i],v=valueOf(row);if(!finite(row.sim_time_sec)||!finite(v)){open=false;continue;}if(open)c.lineTo(X(row.sim_time_sec),Y(v));else c.moveTo(X(row.sim_time_sec),Y(v));open=true;points++;}c.stroke();
  if(points===0){c.fillStyle='#9eafc0';c.textAlign='center';c.fillText('当前时刻无有效记录',(left+right)/2,(top+bottom)/2);}
  c.strokeStyle='#ffd28e';c.lineWidth=1;c.setLineDash([3,3]);c.beginPath();c.moveTo(X(time),top);c.lineTo(X(time),bottom);c.stroke();c.setLineDash([]);
}
function tick(timestamp){if(playing){if(previousTick!==null)setTime(time+Math.min(.25,(timestamp-previousTick)/1000)*Number($('rate').value));previousTick=timestamp;if(time>=duration)pause();}requestAnimationFrame(tick);}
window.addEventListener('resize',draw);setTime(0);requestAnimationFrame(tick);
</script></body></html>'''
