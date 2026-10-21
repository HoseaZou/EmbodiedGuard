"""Absolute-path MCP entry, independent of the caller's working directory."""
import argparse
from pathlib import Path
import sys

from embodiedguard.execution import Mission
from embodiedguard.mcp_server import serve
from embodiedguard.sim import SCENARIOS
from embodiedguard.shadow import ShadowMission
from embodiedguard.motion_faults import FIXTURES, MotionFaultMission
from embodiedguard.task_contract import TASK_FIXTURES, TaskMission
from embodiedguard.efficiency import PROFILES


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=SCENARIOS, default='blocked')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--response-mode', choices=['full', 'compact'], default='full')
    parser.add_argument('--interface-profile', choices=PROFILES, default='baseline',
                        help='Optional audited transport layer; baseline keeps the original eight-tool interface.')
    parser.add_argument('--disable-shadow', action='store_true', help='Use the frozen executor without local prediction sidecars.')
    parser.add_argument('--fixture', choices=FIXTURES, help='Explicit motion-triggered diagnostic fixture, evaluated separately from legacy scenarios.')
    parser.add_argument('--task-fixture', choices=TASK_FIXTURES, help='Explicit revisioned task diagnostic; no safety/model changes.')
    parser.add_argument('--clock-policy',choices=['measured_idle_wall_deployment','execution_only_diagnostic'],
                        default='measured_idle_wall_deployment')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent / 'outputs' / 'mcp')
    args = parser.parse_args()
    if hasattr(sys.stdin, 'reconfigure'):
        sys.stdin.reconfigure(encoding='utf-8')
        sys.stdout.reconfigure(encoding='utf-8')
    mission_type = Mission if args.disable_shadow else ShadowMission
    if args.fixture and args.task_fixture:
        parser.error('select one motion or task fixture')
    if (args.fixture or args.task_fixture) and args.disable_shadow:
        parser.error('motion fixtures include local shadow evidence; --disable-shadow is for legacy scenarios')
    if args.clock_policy=='execution_only_diagnostic' and not (args.fixture or args.task_fixture):
        parser.error('diagnostic clock requires an explicitly selected motion fixture')
    mission = (TaskMission(args.task_fixture, seed=args.seed) if args.task_fixture else
               MotionFaultMission(args.fixture,seed=args.seed) if args.fixture else mission_type(args.scenario, seed=args.seed))
    serve(mission,args.output,response_mode=args.response_mode,
          interface_profile=args.interface_profile,
          charge_idle_wall=args.clock_policy=='measured_idle_wall_deployment')


if __name__ == '__main__':
    main()

