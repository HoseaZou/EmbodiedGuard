"""Reproducible paired engineering checks and existing-action replay; no model calls."""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import hashlib
from html import escape
import json
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from embodiedguard.evidence import load_run
from embodiedguard.execution import Mission
from embodiedguard.mcp_server import MCPServer
from embodiedguard.replay import write_replay
from embodiedguard.shadow import ShadowMission, attach_shadow


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def write(path, value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def rows(path):
    return [json.loads(row) for row in path.read_text(encoding='utf-8').splitlines() if row.strip()]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_server(mission, mode):
    result = MCPServer(mission,response_mode=mode,charge_idle_wall=False)
    result.handle(dict(jsonrpc='2.0',id=1,method='initialize',params={'protocolVersion':'2025-11-25'}))
    result.handle(dict(jsonrpc='2.0',method='notifications/initialized'))
    return result


class Pair:
    def __init__(self,scenario,seed=0,mode='compact',idle=0.):
        self.missions = [Mission(scenario,seed=seed),ShadowMission(scenario,seed=seed)]
        self.servers = [make_server(m,mode) for m in self.missions]
        self.requests = []
        self.idle = idle
        self.feedback_equal = True
        self.wall_seconds = [0.,0.]

    def elapse(self,seconds):
        values = [m.elapse_idle_wall(seconds) for m in self.missions]
        assert values[0] == values[1], 'Idle schedules diverged'

    def call(self,tool,args=None,identifier=None,apply_idle=True):
        if apply_idle and self.idle:
            self.elapse(self.idle)
        request = dict(jsonrpc='2.0',id=identifier or len(self.requests)+2,
                       method='tools/call',params=dict(name=tool,arguments=args or {}))
        values=[]
        for index,server in enumerate(self.servers):
            started=time.perf_counter()
            values.append(server.handle(deepcopy(request)))
            self.wall_seconds[index]+=time.perf_counter()-started
        self.requests.append(request)
        assert values[0] == values[1], 'Tool feedback changed: '+tool
        if 'error' in values[0]:
            raise ValueError(values[0]['error'])
        return values[0]['result']['structuredContent']

    def finish(self,path):
        path.mkdir(parents=True,exist_ok=False)
        left,right=self.missions
        assert left._trace == right._trace, 'Actual motion trace changed'
        assert left._plans == right._plans, 'Spatial planning evidence changed'
        assert left.verify() == right.verify(), 'Independent verification changed'
        for kind,mission in zip(('baseline','candidate'),self.missions):
            target=path/kind
            mission.save(target,tool_context=self.servers[0]._tool_context())
            evidence=load_run(target)
            if kind=='candidate':
                evidence=attach_shadow(evidence,target)
                assert not evidence['shadow_errors'], evidence['shadow_errors']
            write_replay(evidence,target/'replay.html')
        write(path/'requests.json',self.requests)
        shadow=right.shadow_evidence()
        failures=[e['result']['reason'] for e in right._events
                  if e['event']=='tool_call_finished' and e['result'].get('ok') is False]
        motion_failures=[e for e in right._events if e['event']=='execution_finished' and not e['ok']]
        trace=right._trace
        exposure=[]
        for e in motion_failures:
            operation=[r for r in trace if r.get('operation_id')==e['operation_id']]
            exposure.append(dict(reason=e['reason'],operation_id=e['operation_id'],
                movement_recorded_in_operation=any(r['state']['speed_mps']>0 for r in operation),
                stopped=e['stopping_verified']))
        summary=right.verify()
        result=dict(case_id=path.name,scenario=right._world.scenario,seed=right._world.seed,
            task_success=summary['task_success'],safe_termination=summary['safe_termination'],
            feedback_equal=True,trace_equal=True,plans_equal=True,verify_equal=True,
            tool_requests=len(self.requests),failures=dict(Counter(failures)),
            execution_failure_exposure=exposure,metrics=summary['metrics'],
            previews=len(shadow['trajectory_previews']),
            available_previews=sum(p['status']=='available' for p in shadow['trajectory_previews']),
            action_contracts=len(shadow['action_contracts']),
            recovery_decisions=len(shadow['recovery_decisions']),
            matched_prediction_samples=sum(c['matched_samples'] for c in shadow['preview_checks']),
            preview_calculation_wall_sec=sum(p.get('calculation_wall_sec',0) for p in shadow['trajectory_previews']),
            measured_tool_wall_sec=dict(baseline=self.wall_seconds[0],candidate=self.wall_seconds[1]),
            new_model_requests=0,model_usage=None,model_cost=None,
            validation_scope='synthetic_2d_kinematics_only')
        write(path/'comparison.json',result)
        return result


def engineering(pair):
    """One transparent fixed policy, only for feature parity and fault exposure.

    This is not a new performance baseline for claims against Agnes. The fixed
    policy chooses route order and recovery requests; the original MCP recovery
    budget and every original safety check still enforce the execution.
    """
    pair.call('observe')
    operation=0
    for target in ('panel_a','pump_b','exit_c'):
        result=None
        for route in ('direct','upper','lower'):
            plan=pair.call('plan_route',dict(target=target,route=route))
            if plan.get('ok'):
                break
        if not plan.get('ok'):
            pair.call('stop',dict(terminate=True,reason='Diagnostic fixed policy: no feasible admitted route'))
            pair.call('verify')
            return
        for _ in range(20):
            operation+=1
            result=pair.call('execute_route',dict(route_id=plan['route_id'],operation_id=f'diagnostic-{operation}'))
            if result.get('status')=='ARRIVED':
                break
            if result.get('ok'):
                continue
            reason=result.get('reason')
            if reason in ('STALE_OBSERVATION','UNKNOWN_OBSERVATION'):
                recovery=pair.call('reobserve')
            elif reason=='PATH_BLOCKED' and pair.missions[0]._world.scenario=='dynamic':
                recovery=pair.call('wait',dict(seconds=5.))
            else:
                break
            if not recovery.get('ok'):
                break
        if not result or result.get('status')!='ARRIVED':
            pair.call('stop',dict(terminate=True,reason='Diagnostic fixed policy: execution did not arrive'))
            pair.call('verify')
            return
    pair.call('verify')


def replay_native(trial,path):
    review=trial/'evidence'
    original=load_run(review/'frozen-run',verification_directory=review)
    events=original['events']
    mode=next(e['response_mode'] for e in events if e['event']=='tool_call_finished')
    pair=Pair(original['scenario'],original['seed'],mode)
    for event in events:
        if event['event']=='decision_idle_elapsed':
            pair.elapse(event['requested_wall_seconds'])
        elif event['event']=='tool_call_started':
            pair.call(event['tool'],event['arguments'],identifier=event['tool_call_id'],apply_idle=False)
    result=pair.finish(path)
    candidate=pair.missions[1]
    actual=candidate.verify()
    assert candidate._trace==original['trace'], 'Replay differs from original physical trace'
    for key in ('task_success','safe_termination','terminated','termination_reason','completed_targets',
                'missing_targets','final_state','stopping_verified','sim_time_sec','metrics','inspections'):
        assert actual[key]==original['summary'][key], 'Original outcome changed: '+key
    result.update(kind='previous_agnes_action_replay_not_new_model_trial',
        original_trace_equal=True,original_physical_verdict_equal=True,
        original_model_evidence=str(review/'exact-cross-check.json'),
        original_evidence_manifest_sha256=digest(review/'evidence-manifest.json'),
        source_run_sha256=digest(review/'frozen-run/run.json'),
        note='Replays frozen observed model tool arguments and idle durations. New replay event IDs/clock annotations are not original native evidence; no old model proof is attached to the new run.')
    write(path/'comparison.json',result)
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--native-root',type=Path,required=True)
    args=parser.parse_args()
    assert not args.output.exists(), 'Refusing to overwrite an existing validation'
    args.output.mkdir(parents=True)
    protocol=dict(status='frozen_before_execution',source='EmbodiedGuard-v4 via copied unchanged protected modules',
        candidate=str(ROOT),new_model_calls=0,independent_physical_validation=False,
        diagnostic_clocks=[0.,30.],diagnostic_delay='Explicit identical simulation idle before each tool request, after runtime starts; not deployment wall time.',
        primary_checks=['exact paired tool feedback','exact paired actual trace','same independent verify','replay matches previously frozen physical trace'],
        scenarios=[['normal',0],['blocked',0],['stale',0],['unreachable',0],['dynamic',0],['dynamic',1],['dynamic',2],['execution_fault',0]],
        interpretation='Engineering parity and fault-exposure diagnostics only. No ranking or new model recovery rate.',
        source_files={p.relative_to(ROOT).as_posix():digest(p) for p in sorted(ROOT.rglob('*.py')) if 'outputs' not in p.parts})
    write(args.output/'protocol.frozen.json',protocol)
    results=[]
    for delay in (0.,30.):
        for scenario,seed in protocol['scenarios']:
            case=f'diagnostic-{scenario}-seed-{seed}-idle-{int(delay)}'
            pair=Pair(scenario,seed,idle=delay)
            engineering(pair)
            result=pair.finish(args.output/case)
            result.update(kind='paired_fixed_policy_diagnostic',synthetic_idle_per_request_sec=delay)
            write(args.output/case/'comparison.json',result)
            results.append(result)
            print(json.dumps({'recorded':case,'task_success':result['task_success'],'safe_termination':result['safe_termination']},ensure_ascii=False),flush=True)
            write(args.output/'progress.json',{'recorded':results,'errors':[]})
    for trial in sorted(args.native_root.glob('trial-*')):
        result=replay_native(trial,args.output/('replay-'+trial.name))
        results.append(result)
        print(json.dumps({'recorded':result['case_id'],'original_trace_equal':True},ensure_ascii=False),flush=True)
        write(args.output/'progress.json',{'recorded':results,'errors':[]})
    assert len(results)==23, 'Expected 16 engineering pairs and seven native-action replays'
    previews=[p for row in results for p in rows(args.output/row['case_id']/'candidate/trajectory_previews.jsonl')]
    durations=[p['calculation_wall_sec'] for p in previews if 'calculation_wall_sec' in p]
    report=dict(status='completed',engineering_pairs=16,previous_native_replays=7,
        recorded_cases=len(results),all_paired_feedback_and_traces_equal=True,
        all_previous_native_physical_traces_equal=True,new_model_calls=0,
        per_preview_calculation_median_sec=statistics.median(durations) if durations else None,
        per_preview_calculation_max_sec=max(durations) if durations else None,
        extra_model_round_trips=0,model_usage=None,model_cost=None,
        claims=['read-only evidence added','original physical behavior preserved under recorded identical inputs'],
        unsupported_claims=['higher mission success','Agent superiority','real robot prediction accuracy','new live Agnes validation'],cases=results)
    write(args.output/'comparison.json',report)
    table=''.join('<tr><td>'+escape(r['case_id'])+'</td><td>'+('完成' if r['task_success'] else '安全终止' if r['safe_termination'] else '未完成')+'</td><td>'+escape(json.dumps(r['failures'],ensure_ascii=False))+'</td><td>'+str(r['available_previews'])+'/'+str(r['previews'])+'</td><td><a href="'+escape(r['case_id'])+'/candidate/replay.html">轨迹与动作回放</a></td></tr>' for r in results)
    html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>EmbodiedGuard v5 · 动作与预测验证</title><style>body{margin:0;background:#101620;color:#e9f0f5;font:15px/1.65 system-ui,"Microsoft YaHei",sans-serif}main{max-width:1260px;margin:auto;padding:32px}h1{font-size:28px}p{max-width:1000px}a{color:#79b7ff}table{width:100%;border-collapse:collapse;font-size:13px}td,th{text-align:left;padding:12px;border-bottom:1px solid #304053}strong{color:#61dcc3}.note{background:#302920;color:#ffd9a3;padding:16px;border-radius:10px}.cards{display:flex;gap:20px;flex-wrap:wrap}.card{padding:18px;background:#192330;border:1px solid #304053;border-radius:12px}code{font-size:13px}</style><main><h1>EmbodiedGuard v5 · 可检查的动作与短期轨迹</h1><p>Agent 选路线与恢复动作 → 原执行器复用规划 / 控制 → 独立逐步安全检查 → 实际运动与巡检 → 最终 verify。新增只读预测和动作契约只记录证据，不接管选择或安全授权。</p><div class="cards"><div class="card"><strong>16 / 16</strong> 工程配对一致</div><div class="card"><strong>7 / 7</strong> 旧 Agnes 决策重放轨迹一致</div><div class="card"><strong>0</strong> 新模型调用与新增往返</div></div><p class="note">验证范围：合成二维 Ackermann 运动学。7 项重放采用既有真实模型的调用参数和记录等待，不是 7 次新模型实验；未证明任务成功率提高或 Agent 优于固定策略。预测和仿真使用同一模型，吻合不代表真实世界准确率。30 秒延迟诊断可能导致预算耗尽，失败均保留。</p><p>轨迹表示同时给出世界坐标 <code>(t,x,y,yaw,v)</code> 与实际测量锚点下的局部位移 / 转角。借鉴 AutoVLA 的局部运动表达；没有引入其权重、动作 token、码本或训练。橙线为条件性预测，绿线为真实执行记录；原路径仍是空间参考。<a href="comparison.json">完整测量结果</a> · <a href="protocol.frozen.json">执行前冻结协议</a> · <a href="../../README-v5.txt">版本使用说明</a></p><table><thead><tr><th>条件 / 时钟</th><th>独立结果</th><th>真实失败反馈</th><th>有效预测 / 请求</th><th>证据</th></tr></thead><tbody>'''+table+'</tbody></table><p>动作解释仅保留实际提交的短理由；未提交则为 null。恢复记录绑定真实失败事件和后续调用，不填补隐藏推理。完整预测数组留在本地，MCP 的八个工具和返回保持相同。</p></main></html>'
    (args.output/'index.html').write_text(html,encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='cases'},ensure_ascii=False),flush=True)


if __name__=='__main__':
    main()
