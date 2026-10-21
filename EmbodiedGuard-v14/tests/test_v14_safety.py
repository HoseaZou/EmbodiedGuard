import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np

from embodiedguard.map_geometry import Grid
from embodiedguard.public_maps import PublicMapMission,checked_grid_path
from embodiedguard.reachability import certify_disconnection
from embodiedguard.mcp_server import MCPServer
from embodiedguard.control import LegacyTracker
from embodiedguard.sim import PoseState,PlantConfig
from embodiedguard.timing import forecast

class NewSafetyContracts(unittest.TestCase):
    def server(self,mission):
        server=MCPServer(mission,response_mode='compact',interface_profile='focused')
        server.handle({'jsonrpc':'2.0','id':0,'method':'initialize'})
        server.handle({'jsonrpc':'2.0','method':'notifications/initialized'})
        return server

    def call(self,server,tool,**args):
        return server.handle({'jsonrpc':'2.0','id':len(server.mission._events)+1,'method':'tools/call',
                              'params':{'name':tool,'arguments':args}})

    def test_macro_retries_do_not_move_or_execute_again(self):
        mission=PublicMapMission('maze512-16-0',102,.5);server=self.server(mission)
        args=dict(target='exit_c',route='global',route_mode='plan',operation_id='macro-idempotent')
        first=self.call(server,'navigate',**args)['result']['structuredContent']
        self.assertEqual(first['navigation']['original_windows_dispatched'],2)
        frames=mission._world.frame_id;executions=mission._metrics['executions']
        retry=self.call(server,'navigate',**args)['result']['structuredContent']
        self.assertTrue(retry['navigation']['replayed'])
        self.assertEqual(mission._world.frame_id,frames)
        self.assertEqual(mission._metrics['executions'],executions)

    def test_macro_returns_immediately_at_stale_fault(self):
        mission=PublicMapMission('maze512-16-0',142,.5,'freeze_once');server=self.server(mission)
        result=self.call(server,'navigate',target='exit_c',route='global',route_mode='plan',operation_id='stale')
        actual=result['result']['structuredContent']
        self.assertFalse(actual['ok']);self.assertEqual(actual['reason'],'STALE_OBSERVATION')
        self.assertEqual(actual['navigation']['original_windows_dispatched'],1)
        self.assertTrue(actual['stopping_verified'])
        self.assertEqual(mission._world.refresh_attempts,0)

    def test_macro_counts_original_operations_and_honors_cap(self):
        mission=PublicMapMission('maze512-16-0',102,.5);mission._max_operations=1;server=self.server(mission)
        result=self.call(server,'navigate',target='exit_c',route='global',route_mode='plan',operation_id='cap')['result']['structuredContent']
        self.assertEqual(mission._metrics['executions'],1)
        self.assertEqual(len(mission._operations),1)
        self.assertEqual(result['reason'],'OPERATION_BUDGET_EXHAUSTED')
        self.assertTrue(result['stopping_verified'])

    def test_tools_cannot_request_unknown_routes_or_larger_windows(self):
        mission=PublicMapMission('maze512-16-0',102,.5);server=self.server(mission)
        for overrides in ({'max_sim_seconds':60},{'original_window_limit':100},{'route':'teleport'},{'safety_margin_m':0}):
            args=dict(target='exit_c',route='global',route_mode='plan',operation_id='bad');args.update(overrides)
            self.assertIn('error',self.call(server,'navigate',**args))
        self.assertEqual(mission._world.frame_id,0)

    def test_global_search_failures_consume_recovery(self):
        mission=PublicMapMission('maze512-16-0',63,.5);server=self.server(mission)
        for number in range(5):
            result=self.call(server,'navigate',target='exit_c',route='global',route_mode='plan',operation_id=f'fail-{number}')['result']['structuredContent']
        self.assertEqual(result['reason'],'RECOVERY_BUDGET_EXHAUSTED')
        self.assertEqual(result['recovery']['consumed_actions'],3)
        self.assertEqual(mission._world.frame_id,0)

    def test_micro_segments_never_bypass_geometry(self):
        grid=Grid('wall',tuple('.@....' for _ in range(6)))
        # Tiny consecutive increments accumulate past the safe wall margin;
        # comparing each pair would erase this whole motion incorrectly.
        x=.429999995
        points=[(x+i*1e-10,3.) for i in range(100)]
        result=checked_grid_path(grid,points,1.,.57)
        self.assertFalse(result['safe'])

    def test_narrow_door_is_certified_without_treating_grid_failure_as_proof(self):
        rows=[list('.'*20) for _ in range(12)]
        for y in range(12):
            rows[y][10]='@'
        rows[6][10]='.'
        grid=Grid('one-metre-door',tuple(''.join(row) for row in rows))
        result=certify_disconnection(grid,1.,.57,(4.,6.5),(16.,6.5))
        self.assertTrue(result['global_infeasibility_proven'])
        self.assertFalse(result['ackermann_feasibility_proven'])

    def test_wide_door_is_not_falsely_certified_unreachable(self):
        rows=[list('.'*20) for _ in range(12)]
        for y in range(12):rows[y][10]='@'
        rows[5][10]=rows[6][10]='.'
        grid=Grid('two-metre-door',tuple(''.join(row) for row in rows))
        result=certify_disconnection(grid,1.,.57,(4.,6.),(16.,6.))
        self.assertFalse(result['global_infeasibility_proven'])
        self.assertTrue(result['connected'])

    def test_cover_budget_exhaustion_is_unknown(self):
        grid=Grid('open',tuple('.'*20 for _ in range(12)))
        result=certify_disconnection(grid,1.,.57,(4.,6.),(16.,6.),max_cells=1)
        self.assertIsNone(result['connected'])
        self.assertFalse(result['global_infeasibility_proven'])

    def test_signed_curve_tracking_works_for_both_turn_directions(self):
        for sign in (-1,1):
            points=[(4*math.sin(i*math.pi/200),sign*4*(1-math.cos(i*math.pi/200))) for i in range(201)]
            tracker=LegacyTracker(points,global_curve_profile=True)
            estimate=forecast(points,PoseState(),points[-1],PlantConfig(),route_tracker=tracker)
            self.assertIsNotNone(estimate['estimated_arrival_and_stop_sec'])

if __name__=='__main__':unittest.main()
