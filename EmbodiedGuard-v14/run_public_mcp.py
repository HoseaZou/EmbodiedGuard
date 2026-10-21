"""Known public-map MCP entry. All coordinates/map/faults are supervisor owned."""
import argparse
from pathlib import Path
import sys

from embodiedguard.public_maps import PublicMapMission
from embodiedguard.mcp_server import serve

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',required=True,choices=('16room_000','32room_000','maze512-16-0'))
    parser.add_argument('--line',required=True,type=int)
    parser.add_argument('--scale',required=True,type=float,choices=(.5,1.))
    parser.add_argument('--fault',choices=('none','freeze_once','persistent_freeze'),default='none')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parent.parent
    output=args.output.resolve()
    if not output.is_relative_to(root) or len(str(output/'run-20261013-235959-xxxxxxxx/trace.jsonl'))>235:
        raise ValueError('Output must remain in hackathon with a bounded Windows path')
    mission=PublicMapMission(args.map,args.line,args.scale,args.fault,decision_source='external_mcp_client')
    if hasattr(sys.stdin,'reconfigure'):
        sys.stdin.reconfigure(encoding='utf-8');sys.stdout.reconfigure(encoding='utf-8')
    serve(mission,output,response_mode='compact',interface_profile='focused',charge_idle_wall=True)

if __name__=='__main__':main()
