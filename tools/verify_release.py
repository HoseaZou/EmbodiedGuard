"""Read-only integrity check for the competition-source public copy."""
from pathlib import Path
import hashlib
import json
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
IGNORED_PARTS = {'.git', '.venv', '__pycache__', '.pytest_cache', 'runtime_data', 'outputs', 'build', 'dist'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def included(path):
    return not any(part in IGNORED_PARTS or part.endswith('.egg-info') for part in path.relative_to(ROOT).parts)


def main():
    public = json.loads((ROOT / 'provenance/public-files.json').read_text(encoding='utf-8'))
    submitted = json.loads((ROOT / 'provenance/submitted-files.json').read_text(encoding='utf-8'))
    changes = json.loads((ROOT / 'provenance/publication-changes.json').read_text(encoding='utf-8'))
    identity = json.loads((ROOT / 'COMPETITION_VERSION.json').read_text(encoding='utf-8'))
    errors = []
    actual = {p.relative_to(ROOT).as_posix() for p in ROOT.rglob('*') if p.is_file() and included(p)}
    expected = set(public['files']) | {'provenance/public-files.json'}
    if actual != expected:
        errors.append({'file_set': {'missing': sorted(expected - actual), 'unexpected': sorted(actual - expected)}})
    for name, record in public['files'].items():
        path = ROOT / name
        if not path.is_file() or digest(path) != record['sha256'] or path.stat().st_size != record['bytes']:
            errors.append({'public_file_changed': name})
    allowed = {row['path']: row for row in changes['allowed_modifications']}
    changed = set()
    original_py = 0
    for name, record in submitted['files'].items():
        path = ROOT / name
        if not path.is_file():
            errors.append({'submitted_file_missing': name})
            continue
        current = digest(path)
        if current != record['sha256']:
            changed.add(name)
            rule = allowed.get(name)
            if not rule or rule['original_sha256'] != record['sha256'] or rule['public_sha256'] != current or name.endswith('.py'):
                errors.append({'unauthorized_submission_change': name})
        if name.endswith('.py'):
            original_py += 1
    if changed != set(allowed):
        errors.append({'allowed_change_set_mismatch': sorted(changed ^ set(allowed))})
    original_manifest = ROOT / 'provenance/competition-final-launch-manifest.json'
    if digest(original_manifest) != submitted['files']['EmbodiedGuard-v14/final-launch-manifest.json']['sha256']:
        errors.append('Original launch manifest identity changed')
    original = json.loads(original_manifest.read_text(encoding='utf-8'))
    for name, value in original['implementation_files'].items():
        if submitted['files']['EmbodiedGuard-v14/' + name]['sha256'] != value:
            errors.append({'original_pin_mismatch': name})
    launch = json.loads((ROOT / 'EmbodiedGuard-v14/final-launch-manifest.json').read_text(encoding='utf-8'))
    if set(launch['implementation_files']) != set(original['implementation_files']):
        errors.append('Launch implementation file set changed')
    for name, value in launch['implementation_files'].items():
        if digest(ROOT / 'EmbodiedGuard-v14' / name) != value:
            errors.append({'current_launch_pin_mismatch': name})
    if launch['public_data_sha256'] != original['public_data_sha256']:
        errors.append('Original map pin set changed')
    for name, value in original['public_data_sha256'].items():
        if digest(ROOT / name) != value:
            errors.append({'map_changed': name})
    fixture = submitted['supplemental_test_fixture']
    if digest(ROOT / fixture['path']) != fixture['sha256']:
        errors.append('Supplemental fixture changed')
    if identity['archive_sha256'] != submitted['archive_sha256'] or identity['release_identity'] != 'competition-v14':
        errors.append('Competition identity mismatch')
    sensitive_patterns = {
        'private_key': r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',
        'github_token': r'\b(?:ghp_|github_pat_)[A-Za-z0-9_]{30,}',
        'api_key_literal': r'\bsk-[A-Za-z0-9_-]{24,}',
        'user_absolute_path': r'[A-Za-z]:[/\\]+Users[/\\]+|/home/[A-Za-z0-9_.-]+',
        'private_wechat_path': r'wxid_[A-Za-z0-9_]{5,}',
    }
    for name in public['files']:
        path = ROOT / name
        if path.suffix in {'.map', '.scen'}:
            continue
        text = path.read_text(encoding='utf-8', errors='replace')
        for label, pattern in sensitive_patterns.items():
            if re.search(pattern, text):
                errors.append({'sensitive_pattern': label, 'file': name})
    result = {'passed': not errors, 'release': 'competition-v14', 'public_files_checked': len(public['files']),
              'submission_files_checked': len(submitted['files']), 'original_python_files_byte_identical': original_py,
              'allowed_non_python_modifications': len(allowed), 'map_files_checked': len(original['public_data_sha256']),
              'errors': errors, 'model_calls': 0, 'simulation_runs': 0,
              'scope': 'consistency plus scoped text patterns; not a cryptographic authenticity signature'}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
