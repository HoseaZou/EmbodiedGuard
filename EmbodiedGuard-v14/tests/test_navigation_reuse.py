"""Real MCP/plant checks for exact selected-route reuse and rejection boundaries."""
from copy import deepcopy

from embodiedguard.execution import Mission
from embodiedguard.motion_faults import MotionFaultMission
from embodiedguard.task_contract import TaskMission
from test_efficiency_profile import call, connect


def test_preplanned_selected_route_is_not_planned_again():
    mission = TaskMission('restricted_unreachable')
    server = connect(mission)
    plan = call(server, 'plan_route', target='panel_a', route='direct')
    result = call(server, 'navigate', target='panel_a', route='direct', operation_id='handoff')
    assert result['status'] == 'ARRIVED' and result['inspection']['performed']
    assert len(mission._plans) == 1
    assert result['route_id'] == plan['route_id']
    assert result['navigation']['route_reused'] is True
    assert result['navigation']['planning_dispatched'] is False
    assert result['navigation']['source_plan_id'] == plan['plan_id']
    assert len([e for e in mission._events if e['event']=='navigation_route_reused']) == 1


def test_partial_selected_route_keeps_geometry_and_tracker_cursor():
    mission = TaskMission('selective_targets')
    server = connect(mission)
    partial = call(server, 'navigate', target='pump_b', route='direct', operation_id='first')
    assert partial['reason'] == 'SEGMENT_COMPLETE'
    route = mission._routes[partial['route_id']]
    tracker = route['tracker']
    continued = call(server, 'navigate', target='pump_b', route='direct', operation_id='continue')
    assert continued['status'] == 'ARRIVED' and continued['inspection']['performed']
    assert continued['route_id'] == partial['route_id']
    assert mission._routes[continued['route_id']]['tracker'] is tracker
    assert len(mission._plans) == 1 and continued['navigation']['route_reused']


def test_sensor_recovery_reuses_selected_route_without_spending_another_recovery():
    mission = MotionFaultMission('motion_sensor_freeze')
    server = connect(mission)
    failed = call(server, 'navigate', target='panel_a', route='direct', operation_id='first')
    assert failed['reason'] == 'STALE_OBSERVATION'
    call(server, 'reobserve')
    restored = call(server, 'navigate', target='panel_a', route='direct', operation_id='restored')
    assert restored['status'] == 'ARRIVED' and restored['inspection']['performed']
    assert restored['route_id'] == failed['route_id'] and len(mission._plans) == 1
    assert restored['recovery']['consumed_actions'] == 1
    assert restored['navigation']['planning_dispatched'] is False


def test_unknown_sensor_cannot_be_healed_by_reusing_navigation():
    mission = MotionFaultMission('motion_sensor_unavailable')
    server = connect(mission)
    failed = call(server, 'navigate', target='panel_a', route='direct', operation_id='first')
    assert failed['reason'] == 'UNKNOWN_OBSERVATION'
    pose = deepcopy(mission._world.plant.state)
    failed_again = call(server, 'navigate', target='panel_a', route='direct', operation_id='try-existing')
    assert failed_again['reason'] == 'UNKNOWN_OBSERVATION'
    assert not failed_again['inspection']['performed'] and failed_again['stopping_verified']
    assert mission._world.plant.state == pose and len(mission._plans) == 1
    assert mission._world.recovered is False


def test_task_revision_change_rejects_selected_old_handle_without_replanning():
    mission = TaskMission('selective_targets')
    server = connect(mission)
    call(server, 'plan_route', target='pump_b', route='direct')
    # Supervisor version race at this boundary; retain old pointer to prove executor rejection.
    mission._task_revision += 1
    denied = call(server, 'navigate', target='pump_b', route='direct', operation_id='old-version')
    assert denied['reason'] == 'TASK_REVISION_CHANGED'
    assert len(mission._plans) == 1 and mission._metrics['executions'] == 0
    assert not denied['inspection']['performed']


def test_route_permission_change_rejects_selected_old_handle_without_replanning():
    mission = TaskMission('selective_targets')
    server = connect(mission)
    call(server, 'plan_route', target='pump_b', route='direct')
    mission._permitted_routes = ('lower',)
    denied = call(server, 'navigate', target='pump_b', route='direct', operation_id='closed-route')
    assert denied['reason'] == 'TASK_CONSTRAINT_REJECTED'
    assert len(mission._plans) == 1 and mission._metrics['executions'] == 0
    assert mission._world.plant.state.speed_mps == 0


def test_different_agent_route_requires_a_new_plan():
    mission = Mission('normal')
    server = connect(mission)
    old = call(server, 'plan_route', target='panel_a', route='upper')
    chosen = call(server, 'navigate', target='panel_a', route='lower', operation_id='different-route')
    assert chosen['status'] == 'ARRIVED'
    assert chosen['route_id'] != old['route_id'] and len(mission._plans) == 2
    assert chosen['navigation']['planning_dispatched'] and not chosen['navigation']['route_reused']
    assert mission._plans[-1]['route'] == 'lower'


def test_next_target_requires_a_new_plan_not_the_previous_target_handle():
    mission = Mission('normal')
    server = connect(mission)
    first = call(server, 'navigate', target='panel_a', route='direct', operation_id='panel')
    second = call(server, 'navigate', target='pump_b', route='direct', operation_id='pump')
    assert second['status'] == 'ARRIVED'
    assert second['route_id'] != first['route_id'] and len(mission._plans) == 2
    assert second['navigation']['planning_dispatched'] and not second['navigation']['route_reused']


def test_superseded_handle_is_not_reused_from_route_history():
    mission = Mission('normal')
    server = connect(mission)
    first = call(server, 'plan_route', target='panel_a', route='direct')
    call(server, 'plan_route', target='panel_a', route='upper')
    chosen = call(server, 'navigate', target='panel_a', route='direct', operation_id='new-current')
    assert chosen['route_id'] != first['route_id'] and len(mission._plans) == 3
    assert not chosen['navigation']['route_reused']


def test_reused_navigation_retry_is_historical_and_rechecks_no_controller():
    mission = TaskMission('selective_targets')
    server = connect(mission)
    call(server, 'plan_route', target='pump_b', route='direct')
    args = dict(target='pump_b', route='direct', operation_id='reuse')
    first = call(server, 'navigate', **args)
    before = deepcopy((mission._trace, mission._world.time_sec, mission._metrics,
                      mission._plans, mission._inspections))
    historical = call(server, 'navigate', **args)
    assert historical['navigation']['replayed'] and historical['navigation']['route_reused']
    assert historical['route_id'] == first['route_id']
    assert (mission._trace, mission._world.time_sec, mission._metrics,
            mission._plans, mission._inspections) == before
    assert call(server, 'navigate', target='pump_b', route='lower', operation_id='reuse')['reason'] == 'OPERATION_CONFLICT'
    assert len(mission._plans) == 1


def test_reused_handle_does_not_bypass_deadline_or_budget_admission():
    mission = Mission('normal', max_sim_seconds=20)
    server = connect(mission)
    call(server, 'plan_route', target='panel_a', route='direct')
    mission.elapse_idle_wall(10)
    denied = call(server, 'navigate', target='panel_a', route='direct', operation_id='late')
    assert denied['reason'] == 'INSUFFICIENT_COMPLETION_BUDGET'
    assert len(mission._plans) == 1 and mission._metrics['executions'] == 0
    assert denied['stopping_verified'] and not denied['inspection']['performed']


def test_reused_handle_does_not_bypass_execution_operation_cap():
    mission = Mission('normal', max_operations=1)
    server = connect(mission)
    first = call(server, 'navigate', target='panel_a', route='direct', operation_id='panel')
    # Use a current unfinished handle at a target without changing the exhausted cap.
    plan = call(server, 'plan_route', target='pump_b', route='direct')
    denied = call(server, 'navigate', target='pump_b', route='direct', operation_id='over-cap')
    assert denied['reason'] == 'OPERATION_BUDGET_EXHAUSTED'
    assert len(mission._plans) == 2 and mission._metrics['executions'] == 1
    assert denied['navigation']['route_reused']
    assert call(server, 'verify')['safe_termination']


def test_recovery_cap_blocks_new_plan_but_not_the_existing_guarded_executor():
    mission = MotionFaultMission('motion_sensor_freeze')
    server = connect(mission)
    failed = call(server, 'navigate', target='panel_a', route='direct', operation_id='first')
    assert failed['reason'] == 'STALE_OBSERVATION'
    for _ in range(3):
        result = call(server, 'reobserve')
    assert result['recovery']['consumed_actions'] == 3
    restored = call(server, 'navigate', target='panel_a', route='direct', operation_id='resume')
    assert restored['status'] == 'ARRIVED' and restored['recovery']['consumed_actions'] == 3
    assert len(mission._plans) == 1


def test_same_selected_route_after_termination_does_not_replan_or_move():
    mission = Mission('normal')
    server = connect(mission)
    call(server, 'plan_route', target='panel_a', route='direct')
    call(server, 'stop', terminate=True, reason='explicit operator termination')
    denied = call(server, 'navigate', target='panel_a', route='direct', operation_id='after-stop')
    assert denied['ok'] is False and mission._terminated
    assert len(mission._plans) == 1 and mission._metrics['executions'] == 0
    assert denied['stopping_verified']
