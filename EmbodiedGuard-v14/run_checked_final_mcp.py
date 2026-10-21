"""Portable checked entry for v14 inspection or public-map simulation."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy
import sys

PRODUCT=Path(__file__).resolve().parent
ROOT=PRODUCT.parent

def preflight(output):
    manifest_path=PRODUCT/'final-launch-manifest.json'
    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    for name,digest in manifest['implementation_files'].items():
        if hashlib.sha256((PRODUCT/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Final source changed: '+name)
    for name,digest in manifest['public_data_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Registered public data changed: '+name)
    target=Path(output).resolve()
    if not target.is_relative_to(ROOT):
        raise ValueError('Output must remain under the project root')
    longest=target/'run-20261013-235959-xxxxxxxx/trajectory_previews.jsonl'
    if len(str(longest).encode('utf-16-le'))//2>240:
        raise ValueError('Output exceeds Windows path budget')
    return target

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=('inspection','public'),required=True)
    parser.add_argument('--output',type=Path,required=True)
    args,remaining=parser.parse_known_args()
    if '--output' in remaining:parser.error('Duplicate output')
    target=preflight(args.output)
    print(json.dumps({'startup_checked':True,'mode':args.mode,'model_calls':0}),file=sys.stderr)
    if args.mode=='inspection':
        sys.argv=[str(PRODUCT/'run_mcp.py'),'--output',str(target),*remaining]
        runpy.run_path(str(PRODUCT/'run_mcp.py'),run_name='__main__')
        return
    public=argparse.ArgumentParser()
    public.add_argument('--map',required=True,choices=('16room_000','32room_000','maze512-16-0'))
    public.add_argument('--line',required=True,type=int)
    public.add_argument('--scale',required=True,type=float,choices=(.5,1.))
    public.add_argument('--fault',choices=('none','freeze_once','persistent_freeze'),default='none')
    spec=public.parse_args(remaining)
    from embodiedguard.runtime_final import PublicMapMissionFinal
    from embodiedguard.mcp_server import serve
    mission=PublicMapMissionFinal(spec.map,spec.line,spec.scale,spec.fault,decision_source='external_mcp_client')
    if hasattr(sys.stdin,'reconfigure'):
        sys.stdin.reconfigure(encoding='utf-8');sys.stdout.reconfigure(encoding='utf-8')
    serve(mission,target,response_mode='compact',interface_profile='focused',charge_idle_wall=True)

if __name__=='__main__':main()
