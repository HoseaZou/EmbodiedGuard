"""Verify frozen origins and record this implementation's reproducible source hash."""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--git', default='git')
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    hackathon = project.parent
    old = hackathon / 'bev_occupancy_pilot_20260911'
    baseline = json.loads((hackathon / 'reports/evidence/baseline-files.sha256.json').read_text(encoding='utf-8-sig'))
    failures = []
    for name, expected in baseline['files'].items():
        expected_hash = expected if isinstance(expected, str) else expected['sha256']
        if digest(old / name) != expected_hash:
            failures.append(f'old:{name}')
    control = json.loads((project/'provenance_control.json').read_text(encoding='utf-8'))
    for row in control['files']:
        if digest(old/row['source_path']) != row['source_sha256']:
            failures.append(row['source_path'])
        if digest(project/row['derived_path']) != row['derived_sha256']:
            failures.append(row['derived_path'])
    if digest(project/control['adapter']) != control['adapter_sha256']:
        failures.append('control adapter')
    state = json.loads((project/'provenance_state.json').read_text(encoding='utf-8'))
    for key in ('source', 'derived', 'tests'):
        row = state[key]
        if digest(hackathon/row['path_relative_to_hackathon']) != row['sha256']:
            failures.append(f'state:{key}')
    sources = {}
    for path in sorted(project.rglob('*')):
        if not path.is_file() or any(part in ('outputs','.venv','__pycache__','.git') for part in path.relative_to(project).parts):
            continue
        if path.suffix in ('.py','.md','.json','.toml','.ps1','.sh'):
            sources[path.relative_to(project).as_posix()] = digest(path)
            if path.suffix == '.py':
                ast.parse(path.read_text(encoding='utf-8-sig'), filename=str(path))
    def git(*command):
        return subprocess.check_output([args.git, '-C', str(old), *command], text=True, encoding='utf-8').strip()
    old_head, old_status = git('rev-parse','HEAD'), git('status','--porcelain')
    if old_head != baseline['commit'] or old_status:
        failures.append('old git identity or cleanliness')
    code_hash = hashlib.sha256(json.dumps(sources,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    result = dict(passed=not failures, failures=failures, old_tracked_files_checked=len(baseline['files']),
                  old_head=old_head, old_git_clean=not old_status, legacy_copied_files_checked=len(control['files']),
                  state_provenance_checks=3, implementation_files=sources, implementation_manifest_sha256=code_hash)
    destination = hackathon / 'reports/implementation/source-verification.json'
    destination.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k != 'implementation_files'},ensure_ascii=False))
    if failures:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
