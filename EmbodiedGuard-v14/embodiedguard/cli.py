"""Deterministic engineering demos; these policies are explicitly not an LLM."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from .execution import Mission
from .sim import SCENARIOS, finite_number


def run_demo(scenario: str, *, policy='fixed_recovery', seed=0,
             segment_seconds=8.0, decision_delay_sec=0.0) -> Mission:
    if policy not in ('no_recovery', 'fixed_recovery'):
        raise ValueError('unknown engineering policy')
    segment_seconds = finite_number(segment_seconds, 'segment_seconds')
    decision_delay_sec = finite_number(decision_delay_sec, 'decision_delay_sec')
    if not .1 <= segment_seconds <= 15 or decision_delay_sec < 0:
        raise ValueError('segment_seconds must be in [0.1, 15]; decision delay must be nonnegative')
    mission = Mission(scenario, seed=seed, decision_source=policy)

    def call(name, **arguments):
        if decision_delay_sec:
            mission.elapse_idle_wall(decision_delay_sec)
        return getattr(mission, name)(**arguments)

    def finish():
        call('verify')
        return mission

    operation = 0
    recoveries = 0
    for target in ('panel_a', 'pump_b', 'exit_c'):
        choices = ('direct', 'upper', 'lower') if policy == 'fixed_recovery' else ('direct',)
        plan = None
        for route in choices:
            plan = call('plan_route', target=target, route=route,
                                      rationale='Fixed engineering policy: try configured aisle order; no model call.')
            if plan['ok']:
                break
        if not plan or not plan['ok']:
            call('stop', terminate=True, reason=plan['reason'] if plan else 'NO_PLAN')
            return finish()
        if plan['status'] == 'ARRIVED':
            result = plan
        else:
            result = None
            for _ in range(16):
                operation += 1
                result = call('execute_route', route_id=plan['route_id'], operation_id=f'demo-{operation}',
                              max_sim_seconds=segment_seconds, rationale='Fixed engineering policy: execute bounded segment.')
                if result['status'] == 'ARRIVED':
                    break
                if result['ok']:
                    continue
                if policy == 'no_recovery' or recoveries >= 3:
                    break
                if result['reason'] in ('STALE_OBSERVATION', 'UNKNOWN_OBSERVATION'):
                    recovered = call('reobserve', rationale='Fixed rule for stale/unknown state.')
                elif result['reason'] == 'PATH_BLOCKED':
                    recovered = call('wait', seconds=5., rationale='Fixed rule: wait for crossing object.')
                else:
                    break
                recoveries += 1
                if not recovered['ok']:
                    break
        if result is None or result['status'] != 'ARRIVED':
            call('stop', terminate=True, reason=result['reason'] if result else 'NO_PROGRESS')
            return finish()
        inspection = call('inspect', target=target)
        if not inspection['ok']:
            call('stop', terminate=True, reason=inspection['reason'])
            return finish()
    return finish()


def save_demo(mission: Mission, path: Path):
    mission.save(path)
    from .replay import write_replay
    write_replay(mission.evidence(), path / 'replay.html')


def main():
    parser = argparse.ArgumentParser(description='EmbodiedGuard 2D simulation, no runtime model required.')
    sub = parser.add_subparsers(dest='command', required=True)
    demo = sub.add_parser('demo', help='Run a labelled deterministic engineering policy.')
    demo.add_argument('--scenario', choices=SCENARIOS, default='blocked')
    demo.add_argument('--policy', choices=('no_recovery','fixed_recovery'), default='fixed_recovery')
    demo.add_argument('--seed', type=int, default=0)
    demo.add_argument('--segment-seconds', type=float, default=8.)
    demo.add_argument('--decision-delay-sec', type=float, default=0.)
    demo.add_argument('--output', type=Path)
    evaluate = sub.add_parser('evaluate', help='Paired engineering smoke matrix, not an Agnes A/B experiment.')
    evaluate.add_argument('--output', type=Path)
    experiment = sub.add_parser('experiment', help='Repeat paired engineering trials with a frozen protocol; no model calls.')
    experiment.add_argument('--scenarios', nargs='+', choices=SCENARIOS, default=list(SCENARIOS))
    experiment.add_argument('--seeds', nargs='+', type=int, default=[0])
    experiment.add_argument('--repeats', type=int, default=1)
    experiment.add_argument('--segment-seconds', type=float, default=8.)
    experiment.add_argument('--decision-delay-sec', type=float, default=0.)
    experiment.add_argument('--output', type=Path)
    replay = sub.add_parser('replay', help='Render recorded evidence without rerunning a mission or a model.')
    replay.add_argument('--run', type=Path, required=True)
    replay.add_argument('--verification', type=Path, help='Optional directory containing matched AGH tool projection and ledger.')
    replay.add_argument('--output', type=Path, required=True, help='New HTML file; existing files are not overwritten.')
    args = parser.parse_args()
    if args.command == 'replay':
        from .evidence import load_run
        from .replay import write_replay
        if args.output.exists():
            parser.error('replay output already exists; choose a new file')
        evidence = load_run(args.run, verification_directory=args.verification)
        write_replay(evidence, args.output)
        print(json.dumps(dict(output=str(args.output.resolve()),
                              external_verification=evidence.get('external_verification')), ensure_ascii=False))
        return
    base = args.output or Path(__file__).resolve().parents[1] / 'outputs' / f'{args.command}-{time.strftime("%Y%m%d-%H%M%S")}'
    if args.command == 'experiment':
        from .experiments import run_experiments
        run_experiments(base, scenarios=args.scenarios, seeds=args.seeds, repeats=args.repeats,
                        segment_seconds=args.segment_seconds, decision_delay_sec=args.decision_delay_sec)
        print(json.dumps(dict(output=str(base.resolve()), scope='deterministic_engineering_trials_no_model_calls'), ensure_ascii=False))
    elif args.command == 'demo':
        mission = run_demo(args.scenario, policy=args.policy, seed=args.seed,
                           segment_seconds=args.segment_seconds, decision_delay_sec=args.decision_delay_sec)
        save_demo(mission, base)
        print(json.dumps(dict(output=str(base.resolve()), summary=mission.verify()), ensure_ascii=False, indent=2))
    else:
        results = []
        for scenario in SCENARIOS:
            for policy in ('no_recovery', 'fixed_recovery'):
                mission = run_demo(scenario, policy=policy)
                save_demo(mission, base / scenario / policy)
                results.append(dict(scenario=scenario, policy=policy, seed=0, **mission.verify()))
        payload = dict(scope='engineering_smoke_matrix_not_model_or_generalization_evidence',
                       decisions='deterministic policies', clock='fixed-step, policy computation not charged',
                       same_scenario_seed_controller_and_safety=True, results=results)
        (base / 'matrix.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        print(json.dumps(dict(output=str(base.resolve()), results=[{k:row[k] for k in
              ('scenario','policy','task_success','safe_termination','termination_reason')} for row in results]), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
