from copy import deepcopy
import importlib.util
from pathlib import Path
import unittest

from embodiedguard.mcp_server import MCPServer
from embodiedguard.task_contract import TaskMission, TASK_FIXTURES
from embodiedguard.evidence import load_run

POLICY_PATH = Path(__file__).resolve().parents[2] / 'reports/agent-validation/20261011-task-contract-r1/task_policy.py'
spec = importlib.util.spec_from_file_location('task_fixed_policy', POLICY_PATH)
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


class Client:
    def __init__(self, fixture):
        self.mission = TaskMission(fixture)
        self.server = MCPServer(self.mission, response_mode='compact')
        self.server.handle(dict(jsonrpc='2.0', id=0, method='initialize', params={}))
        self.server.handle(dict(jsonrpc='2.0', method='notifications/initialized'))
        self.calls = []

    def call(self, name, **arguments):
        response = self.server.handle(dict(jsonrpc='2.0', id=len(self.calls) + 1, method='tools/call',
            params=dict(name=name, arguments=arguments)))
        result = response['result']['structuredContent']
        self.calls.append(dict(tool=name, arguments=arguments, result=deepcopy(result)))
        return result


class TaskContractTests(unittest.TestCase):
    def test_unknown_fixture_rejected(self):
        with self.assertRaises(ValueError):
            TaskMission('unknown')

    def test_selective_goals_reject_unrequested_inspection_and_plan(self):
        client = Client('selective_targets')
        initial = client.call('observe')
        self.assertEqual(initial['remaining_targets'], ['pump_b', 'exit_c'])
        self.assertEqual(client.call('plan_route', target='panel_a')['reason'], 'TARGET_NOT_REQUIRED')
        self.assertEqual(client.call('inspect', target='panel_a')['reason'], 'TARGET_NOT_REQUIRED')
        self.assertEqual(client.mission._world.plant.state.x, 0.)
        self.assertEqual(client.mission._inspections, {})

    def test_task_snapshot_cannot_modify_authoritative_contract(self):
        mission = TaskMission('selective_targets')
        packet = mission.observe()
        packet['task']['required_targets'].append('panel_a')
        packet['task']['permitted_routes'].clear()
        self.assertEqual(mission.task_status()['required_targets'], ['pump_b', 'exit_c'])
        self.assertEqual(mission.task_status()['permitted_routes'], ['direct', 'upper', 'lower'])

    def test_condition_cannot_resolve_before_actual_inspection(self):
        client = Client('conditional_skip')
        client.call('observe')
        self.assertEqual(client.call('inspect', target='panel_a')['reason'], 'INSPECTION_POSE_NOT_REACHED')
        self.assertEqual(client.mission.task_status()['revision'], 1)
        self.assertFalse(client.mission.task_status()['conditional_rule']['resolved'])

    def test_required_inspection_order_enforced(self):
        client = Client('conditional_skip')
        self.assertEqual(client.call('inspect', target='pump_b')['reason'], 'TARGET_OUT_OF_ORDER')
        self.assertEqual(client.mission._inspections, {})

    def test_complete_comparator_handles_each_contract(self):
        for fixture in TASK_FIXTURES:
            with self.subTest(fixture=fixture):
                client = Client(fixture)
                verdict = policy.run_policy(client.call)
                reachable = fixture != 'restricted_unreachable'
                self.assertEqual(verdict['task_success'], reachable)
                self.assertEqual(verdict['safe_termination'], not reachable)
                self.assertTrue(verdict['stopping_verified'])
                self.assertEqual(client.calls[-1]['tool'], 'verify')
                for key in ('collisions', 'boundary_violations', 'speed_violations', 'task_constraint_rejections'):
                    self.assertEqual(verdict['metrics'][key], 0)

    def test_conditional_update_has_actual_evidence_and_no_pump_inspection(self):
        client = Client('conditional_skip')
        verdict = policy.run_policy(client.call)
        self.assertEqual(verdict['completed_targets'], ['panel_a', 'exit_c'])
        self.assertNotIn('pump_b', verdict['inspections'])
        event = next(e for e in client.mission._events if e['event'] == 'task_contract_updated')
        witness = next(e for e in client.mission._events if e['event_id'] == event['evidence_event_id'])
        self.assertEqual(witness['event'], 'inspection_verified')
        self.assertIs(witness['evidence']['anomaly'], False)
        self.assertEqual(event['source'], 'actual_inspection_condition')

    def test_upper_route_refused_after_real_task_update_without_motion(self):
        client = Client('restricted_detour')
        plan = client.call('plan_route', target='panel_a')
        arrival = client.call('execute_route', route_id=plan['route_id'], operation_id='a')
        self.assertEqual(arrival['status'], 'ARRIVED')
        self.assertEqual(arrival['task']['revision'], 2)
        before = (client.mission._world.time_sec, client.mission._world.plant.state)
        rejection = client.call('plan_route', target='pump_b', route='upper')
        self.assertEqual(rejection['reason'], 'ROUTE_NOT_PERMITTED')
        self.assertEqual(before, (client.mission._world.time_sec, client.mission._world.plant.state))
        self.assertNotIn('upper', rejection['recovery']['untried_routes'])

    def test_old_route_cannot_be_new_execution_permission(self):
        client = Client('conditional_skip')
        plan = client.call('plan_route', target='panel_a')
        client.call('execute_route', route_id=plan['route_id'], operation_id='a')
        before = (client.mission._world.time_sec, len(client.mission._trace))
        result = client.call('execute_route', route_id=plan['route_id'], operation_id='b')
        self.assertEqual(result['reason'], 'TASK_REVISION_CHANGED')
        self.assertEqual(before, (client.mission._world.time_sec, len(client.mission._trace)))

    def test_cached_bundle_is_historical_but_current_task_envelope_is_new(self):
        client = Client('conditional_skip')
        plan = client.call('plan_route', target='panel_a')
        args = dict(route_id=plan['route_id'], operation_id='a')
        first = client.call('execute_route', **args)
        before = (client.mission._world.time_sec, len(client.mission._trace))
        repeated = client.call('execute_route', **args)
        # A cached execution receipt is immutable; transport counters and the
        # explicit replay flag describe this new request and must change.
        self.assertEqual({k:v for k,v in first.items() if k not in ('tool_budget', 'recovery')},
                         {k:v for k,v in repeated.items() if k not in ('tool_budget', 'recovery')})
        self.assertEqual(repeated['tool_budget']['counted_requests'], first['tool_budget']['counted_requests'] + 1)
        self.assertTrue(repeated['recovery']['last_execution_result_replayed'])
        self.assertEqual(repeated['task']['revision'], 2)
        self.assertEqual(before, (client.mission._world.time_sec, len(client.mission._trace)))

    def test_legal_lower_route_is_used_for_static_blockage(self):
        client = Client('restricted_detour')
        verdict = policy.run_policy(client.call)
        plans = [c for c in client.calls if c['tool'] == 'plan_route' and c['arguments']['target'] == 'pump_b']
        self.assertEqual([c['arguments']['route'] for c in plans], ['direct', 'lower'])
        self.assertEqual(plans[0]['result']['reason'], 'PATH_BLOCKED')
        self.assertTrue(verdict['task_contract_verified'])

    def test_no_recovery_is_not_strong_comparator(self):
        client = Client('restricted_detour')
        verdict = policy.run_policy(client.call, recovery=False)
        self.assertFalse(verdict['task_success'])
        self.assertTrue(verdict['safe_termination'])

    def test_model_cannot_set_goals_or_permissions_through_tool_arguments(self):
        client = Client('selective_targets')
        reply = client.server.handle(dict(jsonrpc='2.0', id=900, method='tools/call',
            params=dict(name='plan_route', arguments=dict(target='panel_a', required_targets=['panel_a']))))
        self.assertEqual(reply['error']['code'], -32602)
        self.assertEqual(client.mission.task_status()['required_targets'], ['pump_b', 'exit_c'])

    def test_initialization_discloses_dynamic_requirements_in_both_modes(self):
        for mode in ('full', 'compact'):
            with self.subTest(mode=mode):
                server = MCPServer(TaskMission('selective_targets'), response_mode=mode)
                response = server.handle(dict(jsonrpc='2.0', id=0, method='initialize', params={}))
                instructions = response['result']['instructions']
                self.assertIn('task envelope', instructions)
                self.assertNotIn('panel_a,pump_b,exit_c in order', instructions)

    def test_new_task_receipt_remains_valid_in_existing_evidence_loader(self):
        import tempfile
        client = Client('conditional_skip')
        policy.run_policy(client.call)
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]) as directory:
            client.mission.save(directory)
            run = load_run(Path(directory))
            self.assertTrue(run['completion_receipt']['criteria_met_at_completion'])
            self.assertEqual(run['summary']['completed_targets'], ['panel_a', 'exit_c'])


if __name__ == '__main__':
    unittest.main()
