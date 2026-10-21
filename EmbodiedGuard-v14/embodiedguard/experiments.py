"""Frozen, paired engineering trials; never substitutes scripts for Agnes evidence."""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import inspect
import json
from pathlib import Path
import platform
import statistics
import time

from .execution import Mission
from .sim import InspectionWorld, SCENARIOS, finite_number


POLICIES = ('no_recovery', 'fixed_recovery')
SCOPE = 'paired_deterministic_engineering_trials_not_agnes_or_generalization_evidence'


def _write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                    encoding='utf-8')


def _inputs(scenarios, seeds, repeats, segment_seconds, decision_delay_sec):
    scenarios, seeds = tuple(scenarios), tuple(seeds)
    if not scenarios or any(value not in SCENARIOS for value in scenarios):
        raise ValueError('scenarios must be a nonempty selection of known scenarios')
    if len(set(scenarios)) != len(scenarios):
        raise ValueError('duplicate scenarios are ambiguous; use repeats')
    if not seeds or any(type(seed) is not int or seed < 0 for seed in seeds):
        raise ValueError('seeds must be nonnegative integers')
    if len(set(seeds)) != len(seeds):
        raise ValueError('duplicate seeds are ambiguous; use repeats')
    if type(repeats) is not int or repeats < 1:
        raise ValueError('repeats must be a positive integer')
    segment_seconds = finite_number(segment_seconds, 'segment_seconds')
    decision_delay_sec = finite_number(decision_delay_sec, 'decision_delay_sec')
    if not .1 <= segment_seconds <= 15 or decision_delay_sec < 0:
        raise ValueError('segment_seconds must be in [0.1,15]; decision_delay_sec must be nonnegative')
    return scenarios, seeds, repeats, segment_seconds, decision_delay_sec


def _source_manifest() -> dict:
    root = Path(__file__).resolve().parents[1]
    files = sorted((root / 'embodiedguard').rglob('*.py'))
    files.extend(path for path in (root / 'run_mcp.py', root / 'pyproject.toml',
                                   root / 'prompts' / 'inspection.md') if path.is_file())
    hashes = {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in sorted(files)}
    encoded = json.dumps(hashes, sort_keys=True, separators=(',', ':')).encode('utf-8')
    return dict(algorithm='sha256', aggregate_sha256=hashlib.sha256(encoded).hexdigest(),
                files=hashes, scope='local_runtime_python_and_entrypoint_project_prompt')


def _fixtures(scenarios, seeds) -> list[dict]:
    fixtures = []
    for scenario in scenarios:
        for seed in seeds:
            world = InspectionWorld(scenario, seed)
            fixtures.append(dict(
                scenario=scenario, seed=seed,
                fixture_id=f'dynamic-phase-{seed % 3}' if scenario == 'dynamic' else f'{scenario}-fixed',
                seed_changes_fixture=scenario == 'dynamic',
                seed_rule='initial_y=-10.0-(seed%3)*0.3' if scenario == 'dynamic' else 'seed does not change fixture',
                initial_obstacles=world.obstacles_at(),
                fault=(dict(kind='sensor_freeze', trigger_sim_seconds=4.0) if scenario == 'stale' else
                       dict(kind='execution_fault', trigger_sim_seconds=4.0) if scenario == 'execution_fault' else None)))
    return fixtures


def prepare_agent_trials(scenarios=SCENARIOS, seeds=(0,), repeats=1, *,
                         segment_seconds=8.0, decision_delay_sec=0.0) -> dict:
    """Return an unexecuted checklist. This function neither starts AGH nor invokes a model."""
    scenarios, seeds, repeats, segment_seconds, decision_delay_sec = _inputs(
        scenarios, seeds, repeats, segment_seconds, decision_delay_sec)
    fixtures = {(row['scenario'], row['seed']): row['fixture_id'] for row in _fixtures(scenarios, seeds)}
    return dict(
        status='prepared_not_executed', model_calls_made=False,
        model_usage=None, model_cost=None,
        prerequisites=[
            'Freeze the exact runtime source manifest used by the paired engineering trials.',
            'Start a fresh Mission process AND a new AGH conversation for every trial; a new conversation alone does not reset the shared MCP Mission.',
            'Confirm the requested scenario/seed, frame 0, simulation time 0, initial pose and no completed targets before starting.',
            'Use identical plant, safety, mission time/operation budgets and per-execution segment limit.',
            'Predeclare recovery-attempt accounting before formal comparison: the current fixed policy counts execution recovery calls separately from its up-to-three route planning choices.',
            'Keep all attempts, failures, approvals and tool results; export a redacted AGH ledger and match it to the simulator events.',
            'Record runtime model identity, measured per-call wall delays and actual token/cost metadata; leave unavailable values null.',
        ],
        clock_comparison=dict(
            engineering_delay_sec=decision_delay_sec,
            engineering_delay_kind='synthetic_fixed_delay_per_tool_call_not_measured_agnes_latency',
            matched_clock_claimed=False,
            requirement='For strict clock-matched comparison, replay recorded Agent per-call delays or use a separately specified frozen virtual-clock protocol. Equal fixed delays alone do not match actual Agent latency.'),
        common_task=dict(targets=['panel_a', 'pump_b', 'exit_c'], require_inspection=True,
                         execution_segment_seconds=segment_seconds, max_recovery_attempts=3,
                         route_policy='Agent chooses from tool feedback; do not force the fixed engineering route order.',
                         completion='Call verify and preserve success or safe termination without discarding failures.'),
        trials=[dict(trial_id=f'{scenario}--seed-{seed}--repeat-{repeat:03d}--agnes',
                     scenario=scenario, seed=seed, repeat=repeat, fixture_id=fixtures[(scenario, seed)],
                     status='not_run', agh_session_id=None, mission_run_id=None, evidence_directory=None)
                for scenario in scenarios for seed in seeds for repeat in range(1, repeats + 1)])


def _aggregate(rows: list[dict]) -> list[dict]:
    groups = []
    for scenario, policy in sorted({(row['scenario'], row['policy']) for row in rows}):
        selected = [row for row in rows if row['scenario'] == scenario and row['policy'] == policy]
        completed = [row for row in selected if row['status'] == 'recorded']
        successes = sum(row['task_success'] for row in completed)
        encountered_failures = [row for row in completed if row['encountered_failure']]
        successful_after_failure = sum(row['task_success'] for row in encountered_failures)
        def mean(key):
            values = [row[key] for row in completed if row[key] is not None]
            return statistics.mean(values) if values else None
        groups.append(dict(
            scenario=scenario, policy=policy, attempted=len(selected), recorded=len(completed),
            runner_errors=len(selected) - len(completed), task_successes=successes,
            task_success_rate=successes / len(selected),
            success_rate_denominator='all attempted trials, including runner errors',
            safe_terminations=sum(row['safe_termination'] for row in completed),
            termination_reasons=dict(Counter(row['termination_reason'] or 'not_terminated' for row in completed)),
            total_collisions=sum(row['metrics']['collisions'] for row in completed),
            total_boundary_violations=sum(row['metrics']['boundary_violations'] for row in completed),
            total_speed_violations=sum(row['metrics']['speed_violations'] for row in completed),
            total_safety_interventions=sum(row['metrics']['safety_interventions'] for row in completed),
            safety_counts_exclude_unknown_runner_errors=True,
            mean_cte_rmse_m=mean('cte_rmse_m'), mean_cte_p95_m=mean('cte_p95_m'),
            cte_aggregation='arithmetic mean of per-trial metrics; not pooled trajectory RMSE or P95',
            mean_path_length_m=mean('path_length_m'), mean_sim_time_sec=mean('sim_time_sec'),
            mean_wall_runtime_sec=mean('wall_runtime_sec'),
            encountered_failure_trials=len(encountered_failures),
            encountered_failure_then_task_success_count=successful_after_failure,
            encountered_failure_then_task_success_rate=(successful_after_failure / len(encountered_failures)
                                                        if encountered_failures else None),
            recovery_rate_note='Descriptive task-level rate among trials with a logged rejected plan or failed execution. Faults are not preclassified as recoverable; this is not a formal recoverable-fault recovery rate.',
            model_usage=None, model_cost=None))
    return groups


def run_experiments(output: Path, scenarios=SCENARIOS, seeds=(0,), repeats=1,
                    segment_seconds=8.0, decision_delay_sec=0.0) -> dict:
    """Run both fixed policies on fresh Missions, freeze protocol and retain every outcome."""
    scenarios, seeds, repeats, segment_seconds, decision_delay_sec = _inputs(
        scenarios, seeds, repeats, segment_seconds, decision_delay_sec)
    output = Path(output)
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise FileExistsError('experiment output must be absent or empty; historical results are never overwritten')
    output.mkdir(parents=True, exist_ok=True)
    sources = _source_manifest()
    world = InspectionWorld()
    mission_defaults = inspect.signature(Mission).parameters
    protocol = dict(
        schema_version=1, scope=SCOPE, created_at_utc=datetime.now(timezone.utc).isoformat(),
        runtime=dict(python=platform.python_version(), platform=platform.platform()),
        source=sources,
        config=dict(scenarios=list(scenarios), seeds=list(seeds), repeats=repeats,
                    policies=list(POLICIES), segment_seconds=segment_seconds,
                    decision_delay_sec=decision_delay_sec,
                    mission_max_sim_seconds=mission_defaults['max_sim_seconds'].default,
                    mission_max_operations=mission_defaults['max_operations'].default,
                    plant=asdict(world.plant.config), bounds=world.bounds, checkpoints=world.checkpoints),
        clock=dict(kind='fixed_step_with_optional_synthetic_tool_delay',
                   step_sec=world.plant.config.dt_sec,
                   synthetic_delay_sec=decision_delay_sec,
                   charge='Before each policy tool call via Mission.elapse_idle_wall; pre-first-plan idle is ignored by Mission. Final verification is charged once.',
                   wall_runtime='Measured policy execution and final delay, excluding artifact serialization; computation wall time is not charged to simulation.',
                   actual_agnes_latency=False),
        design=dict(pairing='same scenario, seed, repeat, controller, safety and configured clock; separate new Mission per policy',
                    order=list(POLICIES), random_order=False, model_calls_made=False,
                    policies=dict(no_recovery='direct route only; terminate on plan or execution failure',
                                  fixed_recovery='direct/upper/lower planning order per target; at most 16 execution segments per target; at most 3 execution recovery calls per mission (reobserve stale/unknown, wait 5 seconds for blocked); planning alternatives are not counted as execution recovery calls'),
                    repeat_interpretation='deterministic repeatability, not independent samples or generalization evidence',
                    seed_interpretation='Only dynamic changes with seed modulo 3; other scenario fixtures are seed-invariant.',
                    fixture_mapping=_fixtures(scenarios, seeds),
                    unexpected_errors='Persist an error row and continue; never omit failed attempts.'),
        model_usage=None, model_cost=None)
    _write(output / 'protocol.json', protocol)
    prepared = prepare_agent_trials(scenarios, seeds, repeats, segment_seconds=segment_seconds,
                                   decision_delay_sec=decision_delay_sec)
    prepared['required_source_sha256'] = sources['aggregate_sha256']
    _write(output / 'agent-trials.json', prepared)
    fixture_ids = {(row['scenario'], row['seed']): row['fixture_id']
                   for row in protocol['design']['fixture_mapping']}
    from .cli import run_demo
    results = []
    for scenario in scenarios:
        for seed in seeds:
            for repeat in range(1, repeats + 1):
                pair_id = f'{scenario}--seed-{seed}--repeat-{repeat:03d}'
                for policy in POLICIES:
                    trial_id = f'{pair_id}--{policy}'
                    trial_dir = output / 'trials' / trial_id
                    trial_dir.mkdir(parents=True)
                    row = dict(trial_id=trial_id, pair_id=pair_id, scenario=scenario, seed=seed,
                               repeat=repeat, policy=policy, fixture_id=fixture_ids[(scenario, seed)],
                               evidence_directory=trial_dir.relative_to(output).as_posix(),
                               model_usage=None, model_cost=None, model_call_verified=False)
                    started = time.perf_counter()
                    try:
                        mission = run_demo(scenario, policy=policy, seed=seed,
                                           segment_seconds=segment_seconds, decision_delay_sec=decision_delay_sec)
                        # run_demo charges its terminal verification once; serialization is an evaluator read.
                        evidence = mission.evidence()
                        wall_runtime = time.perf_counter() - started
                        mission.save(trial_dir)
                        summary = evidence['summary']
                        failures = [event for event in evidence['events']
                                    if (event['event'] == 'planning_decision' and event.get('status') == 'REJECTED')
                                    or (event['event'] == 'execution_finished' and event.get('ok') is False)]
                        row.update(status='recorded', **summary,
                                   wall_runtime_sec=wall_runtime,
                                   encountered_failure=bool(failures),
                                   failure_event_ids=[event['event_id'] for event in failures],
                                   failure_reasons=[event.get('reason') for event in failures],
                                   path_length_m=summary['metrics']['path_length_m'],
                                   cte_rmse_m=summary['metrics']['cte_rmse_m'],
                                   cte_p95_m=summary['metrics']['cte_p95_m'])
                    except Exception as exc:
                        row.update(status='runner_error', error_type=type(exc).__name__, error=str(exc),
                                   task_success=False, safe_termination=None, termination_reason=None,
                                   encountered_failure=None, failure_event_ids=None, failure_reasons=None,
                                   metrics=None, sim_time_sec=None,
                                   wall_runtime_sec=time.perf_counter() - started,
                                   path_length_m=None, cte_rmse_m=None, cte_p95_m=None)
                    results.append(row)
                    _write(trial_dir / 'trial.json', row)
                    # A checkpoint survives interruption without pretending the remaining trials ran.
                    _write(output / 'summary.json', dict(scope=SCOPE, status='running', results=results,
                                                         planned_trials=len(scenarios)*len(seeds)*repeats*len(POLICIES)))
    source_after = _source_manifest()
    payload = dict(
        scope=SCOPE,
        status=('invalid_source_changed_during_run' if source_after != sources else
                'complete_with_runner_errors' if any(row['status'] == 'runner_error' for row in results) else 'complete'),
        planned_trials=len(scenarios)*len(seeds)*repeats*len(POLICIES),
        attempted_trials=len(results), source_unchanged=source_after == sources,
        source_sha256=sources['aggregate_sha256'], source_after_sha256=source_after['aggregate_sha256'],
        model_usage=None, model_cost=None, results=results, aggregate=_aggregate(results),
        interpretation='Descriptive engineering outcomes only; deterministic repeats and repeated fixtures do not estimate generalization or Agnes improvement.')
    _write(output / 'summary.json', payload)
    return payload
