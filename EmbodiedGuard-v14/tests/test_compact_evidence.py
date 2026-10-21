"""Keep compact native feedback and full independent verify separately bound."""
import hashlib
import json

import pytest

from embodiedguard.evidence import load_run
from embodiedguard.feedback import compact_feedback
from test_evidence import make_recording, write_json


def refresh_manifest(review):
    manifest=json.loads((review/'evidence-manifest.json').read_text(encoding='utf-8'))
    for name in manifest['files']:
        manifest['files'][name]['sha256']=hashlib.sha256((review/name).read_bytes()).hexdigest()
    write_json(review/'evidence-manifest.json',manifest)


def compact_recording(tmp_path):
    run,review=make_recording(tmp_path)
    summary=json.loads((run/'summary.json').read_text(encoding='utf-8'))
    summary['budget']=dict(sim_time_sec=20.,remaining_sim_sec=280.,schema_version=1,
        snapshot_phase='tool_return',idle_wall_charged_between_tools=True,max_execution_operations=80)
    summary['tool_budget']=dict(counted_requests=1,max_tool_requests=120,
                               remaining_requests=119,exempt_tools=['stop','verify'])
    write_json(run/'summary.json',summary)
    recording=json.loads((run/'run.json').read_text(encoding='utf-8'))
    recording['summary']=summary
    write_json(run/'run.json',recording)
    events=[json.loads(row) for row in (run/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    events[-1].update(result=summary,model_result=compact_feedback(summary),response_mode='compact')
    (run/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events),encoding='utf-8')
    projection=[json.loads(row) for row in (review/'tool-evidence.exact.jsonl').read_text(encoding='utf-8').splitlines()]
    projection[-1]['result']=compact_feedback(summary)
    (review/'tool-evidence.exact.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in projection),encoding='utf-8')
    refresh_manifest(review)
    return run,review,events,projection


def test_compact_native_result_preserves_full_verify_and_honest_identity(tmp_path):
    run,review,events,projection=compact_recording(tmp_path)
    assert events[-1]['result'] != projection[-1]['result']
    evidence=load_run(run,verification_directory=review)
    assert evidence['external_verification']['status']=='matched'
    assert evidence['summary']['budget']['schema_version']==1
    assert evidence['summary']['model_call_verified'] is False
    assert evidence['summary']['model_cost'] is None


@pytest.mark.parametrize('damage',['model_projection','full_verdict','unsupported_mode'])
def test_matching_hashes_cannot_hide_a_changed_compact_projection_or_full_verify(tmp_path,damage):
    run,review,events,projection=compact_recording(tmp_path)
    if damage=='model_projection':
        events[-1]['model_result']['task_success']=False
        projection[-1]['result']['task_success']=False
    elif damage=='full_verdict':
        events[-1]['result']['task_success']=False
        events[-1]['model_result']['task_success']=False
        projection[-1]['result']['task_success']=False
    else:
        events[-1]['response_mode']='unrecognized'
    (run/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events),encoding='utf-8')
    (review/'tool-evidence.exact.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in projection),encoding='utf-8')
    refresh_manifest(review)
    with pytest.raises(ValueError):
        load_run(run,verification_directory=review)
