"""Print machine-local stdio parameters; does not edit AGH or call a model."""
from pathlib import Path
import json
import sys

root = Path(__file__).resolve().parents[1]
config = {'mcpServers': {'embodiedguard-competition-v14': {
    'command': sys.executable,
    'args': [str(root / 'EmbodiedGuard-v14/run_checked_final_mcp.py'),
             '--mode', 'public', '--map', 'maze512-16-0', '--line', '142',
             '--scale', '0.5', '--fault', 'freeze_once',
             '--output', str(root / 'runtime_data/agh-public')]}}}
print(json.dumps(config, ensure_ascii=False, indent=2))
