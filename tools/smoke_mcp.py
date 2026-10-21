"""Zero-motion local MCP transport check; no AGH account/model involved."""
from pathlib import Path
import json
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
requests = [
    {'jsonrpc': '2.0', 'id': 1, 'method': 'initialize', 'params': {}},
    {'jsonrpc': '2.0', 'method': 'notifications/initialized'},
    {'jsonrpc': '2.0', 'id': 2, 'method': 'tools/list'},
    {'jsonrpc': '2.0', 'id': 3, 'method': 'tools/call', 'params': {'name': 'observe', 'arguments': {}}},
    {'jsonrpc': '2.0', 'id': 4, 'method': 'tools/call', 'params': {'name': 'stop', 'arguments': {'terminate': True, 'reason': 'PUBLIC_RELEASE_PROTOCOL_SMOKE'}}},
    {'jsonrpc': '2.0', 'id': 5, 'method': 'tools/call', 'params': {'name': 'verify', 'arguments': {}}},
]
command = [sys.executable, str(root / 'EmbodiedGuard-v14/run_checked_final_mcp.py'),
           '--mode', 'public', '--map', 'maze512-16-0', '--line', '142', '--scale', '0.5',
           '--fault', 'freeze_once', '--output', str(root / 'runtime_data/protocol-smoke')]
run = subprocess.run(command, input='\n'.join(json.dumps(r) for r in requests) + '\n',
                     capture_output=True, text=True, encoding='utf-8', timeout=60, cwd=root)
if run.returncode:
    print(run.stderr, file=sys.stderr)
    raise SystemExit(run.returncode)
replies = {r['id']: r for r in (json.loads(line) for line in run.stdout.splitlines())}
tools = {t['name'] for t in replies[2]['result']['tools']}
observation = replies[3]['result']['structuredContent']
verification = replies[5]['result']['structuredContent']
assert tools == {'observe', 'navigate', 'reobserve', 'wait', 'stop', 'verify'}, tools
assert observation['observation']['frame_id'] == 0
assert verification['safe_termination'] is True and verification['task_success'] is False
assert verification['metrics']['path_length_m'] == 0
print(json.dumps({'passed': True, 'tools': sorted(tools), 'frame': 0,
                  'safe_termination': True, 'task_success': False,
                  'model_calls': 0, 'motion_actions': 0}, ensure_ascii=False))
