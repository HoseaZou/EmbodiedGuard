"""Symptom-level regressions; they are red on the frozen implementation."""
import json
import math
from pathlib import Path
import tempfile
import unittest

from embodiedguard.public_maps import PublicMapMission
from embodiedguard.mcp_server import MCPServer
from embodiedguard.sim import PlantConfig, PoseState
from embodiedguard.map_geometry import Grid

class DefectRegression(unittest.TestCase):
    def test_saved_task_matches_actual_tool_envelope(self):
        mission = PublicMapMission('maze512-16-0', 102, .5)
        with tempfile.TemporaryDirectory() as folder:
            server = MCPServer(mission, Path(folder), response_mode='compact', interface_profile='focused')
            server.handle({'jsonrpc':'2.0', 'id':0, 'method':'initialize'})
            server.handle({'jsonrpc':'2.0', 'method':'notifications/initialized'})
            result = server.handle({'jsonrpc':'2.0', 'id':1, 'method':'tools/call',
                                   'params':{'name':'verify', 'arguments':{}}})['result']['structuredContent']
            summary = json.loads((Path(folder) / 'summary.json').read_text(encoding='utf-8'))
            self.assertEqual(summary.get('task'), result['task'], 'Current task envelope must survive evidence persistence')

    def test_multiturn_plan_exists_without_changing_vehicle(self):
        from embodiedguard.global_planning import GlobalPlanner, PlannerConfig
        rows = [['.'] * 40 for _ in range(30)]
        for row in range(12, 30):
            rows[row][12] = '@'
        grid = Grid('synthetic-wall-fixture', tuple(''.join(row) for row in rows))
        planner = GlobalPlanner(grid, 1., PlantConfig(), PlannerConfig())
        result = planner.plan(PoseState(4., 8., 0., 0.), (28., 8.))
        self.assertTrue(result.ok, result.diagnostics)
        self.assertGreater(max(y for x,y in result.points), 18.)
        self.assertLessEqual(result.diagnostics['maximum_curvature_per_m'],
                             math.tan(PlantConfig().max_steer_rad * .5) / PlantConfig().wheelbase_m + 1e-6)

    def test_bounded_search_failure_does_not_claim_global_impossibility(self):
        from embodiedguard.global_planning import GlobalPlanner, PlannerConfig
        grid = Grid('open', tuple('.' * 40 for _ in range(30)))
        planner = GlobalPlanner(grid, 1., PlantConfig(), PlannerConfig(max_expansions=1))
        result = planner.plan(PoseState(4., 8., math.pi, 0.), (28., 8.))
        self.assertFalse(result.ok)
        self.assertFalse(result.diagnostics['global_infeasibility_proven'])
        self.assertIn(result.reason, ('SEARCH_BUDGET_EXHAUSTED', 'NO_ROUTE_IN_SEARCH_RESOLUTION'))

    def test_geometric_multiturn_route_is_trackable_before_admission(self):
        mission = PublicMapMission('maze512-16-0', 143, .5)
        plan = mission.plan_route(target='exit_c',route='global')
        self.assertTrue(plan['ok'])
        self.assertIsNotNone(plan['execution_estimate']['estimated_arrival_and_stop_sec'],
                             'A geometrically feasible route must pass the isolated actual-controller forecast')

if __name__ == '__main__':
    unittest.main()
