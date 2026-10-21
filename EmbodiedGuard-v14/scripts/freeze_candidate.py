"""Freeze local candidate code and baseline identity; no runtime/model actions."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
HACK = ROOT.parent
REPORT = HACK / 'reports/implementation/20261010-budget-admission'
BASELINE = HACK / 'EmbodiedGuard'
RELEASE = HACK / 'reports/implementation/20261010-budget-receipt'
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    REPORT.mkdir(parents=True, exist_ok=True)
    original = json.loads((RELEASE / 'implementation-source-manifest.json').read_text(encoding='utf-8'))
    unchanged = all(sha(BASELINE / p) == digest for p, digest in original['implementation_files'].items())
    if not unchanged:
        raise ValueError('Baseline source changed; refusing release freeze')
    files = {p.relative_to(ROOT).as_posix(): sha(p) for p in ROOT.rglob('*') if p.is_file()
        and 'outputs' not in p.relative_to(ROOT).parts and '__pycache__' not in p.relative_to(ROOT).parts}
    mapping = hashlib.sha256(json.dumps(files, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    manifest = dict(schema_version=1, candidate_root=str(ROOT), implementation_files=files,
        mapping_sha256=mapping, baseline_source_unchanged=unchanged, baseline_files_checked=len(original['implementation_files']),
        current_agh_configuration_changed=False, official_harness_modified=False, model_trials_run=0)
    if (REPORT / 'candidate-source-manifest.json').exists():
        raise FileExistsError('Release already frozen')
    for rel in files:
        target = REPORT / 'frozen-source' / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / rel, target)
    (REPORT / 'candidate-source-manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n',encoding='utf-8')
    shutil.copy2(ROOT / 'prompts/inspection.md', REPORT / 'prompt.frozen.md')
    script = """import json
from embodiedguard.execution import Mission
m=Mission('normal'); p=m.plan_route(target='panel_a'); m.elapse_idle_wall(291.57)
r=m.execute_route(route_id=p['route_id'],operation_id='late',max_sim_seconds=15)
print(json.dumps(dict(reason=r['reason'],x=m._world.plant.state.x,time=m._world.time_sec,executions=m._metrics['executions'],stopped=r['stopping_verified'])))
"""
    comparison = {}
    for label, cwd in [('baseline',BASELINE),('candidate',ROOT)]:
        result = subprocess.run([sys.executable,'-c',script], cwd=cwd, text=True, encoding='utf-8',capture_output=True,check=True)
        comparison[label] = json.loads(result.stdout)
    assert comparison['baseline']['x'] > 0 and comparison['baseline']['reason'] == 'TIME_BUDGET_EXHAUSTED'
    assert comparison['candidate']['x'] == 0 and comparison['candidate']['reason'] == 'INSUFFICIENT_COMPLETION_BUDGET'
    (REPORT / 'late-execution-comparison.json').write_text(json.dumps(comparison,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(source_files=len(files), baseline_unchanged=unchanged, comparison=comparison), ensure_ascii=False))

if __name__ == '__main__':
    main()
