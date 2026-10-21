"""Preflight frozen source and short local output storage, then run the MCP entry."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy
import sys
import uuid

ROOT = Path(__file__).resolve().parent
HACK = ROOT.parent.resolve()
MANIFEST = HACK / 'reports/implementation/20261012-route-validity-v13/source-candidate.json'


def preflight(output):
    target = output.resolve()
    longest = target / 'run-20261011-235959-xxxxxxxx' / 'trajectory_previews.jsonl'
    if not target.is_relative_to(HACK):
        raise ValueError('Output must remain inside the hackathon directory')
    if len(str(longest).encode('utf-16-le')) // 2 > 240:
        raise ValueError('Output path exceeds conservative Windows path budget')
    source = json.loads(MANIFEST.read_text(encoding='utf-8'))
    for name, digest in source['implementation_files'].items():
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != digest:
            raise ValueError('Frozen source changed')
    target.mkdir(parents=True, exist_ok=True)
    probe = target / ('preflight-' + uuid.uuid4().hex[:8] + '.tmp')
    if not probe.resolve().is_relative_to(HACK):
        raise ValueError('Storage probe resolved outside hackathon directory')
    try:
        with probe.open('xb') as stream:
            stream.write(b'EmbodiedGuard output probe\n')
        if probe.read_bytes() != b'EmbodiedGuard output probe\n':
            raise ValueError('Storage probe did not round-trip')
    finally:
        if probe.exists():
            probe.unlink()
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args, remaining = parser.parse_known_args()
    if '--output' in remaining:
        parser.error('Duplicate output argument')
    try:
        target = preflight(args.output)
    except (ValueError, OSError) as error:
        print(json.dumps(dict(status='startup_rejected', error_type=type(error).__name__,
            reason=str(error) if isinstance(error, ValueError) else 'Storage unavailable')), file=sys.stderr)
        return 2
    print(json.dumps(dict(status='startup_checked', output=str(target), model_calls=0)), file=sys.stderr)
    sys.argv = [str(ROOT / 'run_mcp.py'), '--output', str(target), *remaining]
    runpy.run_path(str(ROOT / 'run_mcp.py'), run_name='__main__')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
