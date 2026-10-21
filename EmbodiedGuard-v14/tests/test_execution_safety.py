import pytest

from embodiedguard.execution import Mission


def test_expired_observation_never_leaks_current_evaluator_pose():
    mission = Mission('stale')
    plan = mission.plan_route(target='panel_a')
    result = mission.execute_route(route_id=plan['route_id'], operation_id='freeze')
    assert result['reason'] == 'STALE_OBSERVATION'
    assert result['state'] is None
    assert result['stopping_verified'] is True
    assert result['verification_source'] == 'simulator_evaluator'
    mission.wait(seconds=.1)
    feedback = mission.observe()
    assert feedback['reason'] == 'STALE_OBSERVATION'
    assert feedback['state'] is None
    report = mission.verify()
    assert report['final_state'] is None
    assert report['stopping_verified'] is True
    assert mission.reobserve()['ok']
    assert mission.verify()['final_state']['speed_mps'] == 0.


def test_evidence_cannot_rewrite_mission_targets_or_later_missions():
    mission = Mission('normal')
    evidence = mission.evidence()
    evidence['checkpoints']['panel_a'] = (0., 0.)
    evidence['checkpoints']['exit_c'] = (0., 0.)
    assert mission.inspect(target='panel_a')['reason'] == 'INSPECTION_POSE_NOT_REACHED'
    assert Mission('normal').evidence()['checkpoints']['panel_a'] == (12., 0.)
    assert mission.evidence()['evaluation_only_fields'] == ['evaluation_final_state', 'trace']


@pytest.mark.parametrize('operation', ['reobserve', 'wait'])
@pytest.mark.parametrize('budget', [.1, .12])
def test_recovery_cannot_step_past_budget_and_claim_to_be_available(operation, budget):
    mission = Mission('normal', max_sim_seconds=budget)
    result = getattr(mission, operation)()
    assert not result['ok']
    assert result['reason'] == 'TIME_BUDGET_EXHAUSTED'
    report = mission.verify()
    assert report['terminated']
    assert report['safe_termination']
    assert report['sim_time_sec'] <= budget + 1e-9
    assert not mission.inspect(target='panel_a')['ok']
    assert not report['task_success']


@pytest.mark.parametrize('kwargs', [
    {'max_sim_seconds': float('nan')}, {'max_sim_seconds': float('inf')},
    {'max_sim_seconds': True}, {'max_sim_seconds': 0},
    {'max_operations': 1.5}, {'max_operations': True}, {'max_operations': 0},
])
def test_invalid_budgets_cannot_disable_limits(kwargs):
    with pytest.raises(ValueError):
        Mission(**kwargs)


def test_extreme_integer_tool_argument_is_rejected_without_crashing():
    mission = Mission('normal')
    assert mission.wait(seconds=10**1000)['reason'] == 'INVALID_ARGUMENT'


def test_controller_exception_after_motion_returns_a_verified_stop(monkeypatch):
    from embodiedguard import execution

    # Inject an unforeseen REAL execution fault, not a predictor fault. A
    # predictable cloned-controller failure is now rejected before any motion.
    original_forecast = execution.forecast
    def fault_free_prediction(*args, **kwargs):
        kwargs.pop('route_tracker', None)
        return original_forecast(*args, **kwargs)
    monkeypatch.setattr(execution, 'forecast', fault_free_prediction)

    class FailingTracker:
        def __init__(self, *args, **kwargs):
            self.calls = 0

        def command(self, **kwargs):
            self.calls += 1
            if self.calls > 1:
                raise ValueError('injected command-generation failure')
            return {'throttle': .3, 'brake': 0., 'steer_left_normalized': 0.}

        def reset_controller(self):
            pass

    monkeypatch.setattr(execution, 'LegacyTracker', FailingTracker)
    mission = Mission('normal')
    plan = mission.plan_route(target='panel_a')
    result = mission.execute_route(route_id=plan['route_id'], operation_id='controller-failure')
    assert not result['ok']
    assert result['reason'] == 'EXECUTION_ERROR'
    assert result['stopping_verified']
    assert mission.verify()['metrics']['path_length_m'] > 0.
    before = mission.verify()['sim_time_sec']
    assert mission.execute_route(route_id=plan['route_id'], operation_id='controller-failure') == result
    assert mission.verify()['sim_time_sec'] == before


def test_model_idle_time_advances_fault_clock_and_budget_only_after_planning_starts():
    mission = Mission('normal', max_sim_seconds=.2)
    mission.elapse_idle_wall(60.)
    assert mission.verify()['sim_time_sec'] == 0.
    assert mission.plan_route(target='panel_a')['ok']
    result = mission.elapse_idle_wall(.12)
    assert result['advanced_sec'] == pytest.approx(.12)
    assert mission.verify()['sim_time_sec'] == pytest.approx(.12)
    result = mission.elapse_idle_wall(1000.)
    assert result['advanced_sec'] == pytest.approx(.08)
    assert result['reason'] == 'TIME_BUDGET_EXHAUSTED'
    summary = mission.verify()
    assert summary['sim_time_sec'] == pytest.approx(.2)
    assert summary['safe_termination']
    assert not summary['task_success']
    assert summary['metrics']['collisions'] == 0
    for invalid in (-1., True, '1', float('inf'), float('nan')):
        with pytest.raises(ValueError):
            mission.elapse_idle_wall(invalid)
