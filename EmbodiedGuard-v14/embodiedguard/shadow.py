"""A read-only evidence seam around the unchanged deterministic Mission."""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time

from .action_contract import ActionReceipts
from .execution import Mission
from .state import ObservationError
from .trajectory_preview import compare_preview, preview_trajectory


class ShadowMission(Mission):
    """Preserve every tool result; predictions and semantic receipts stay local.

    No new sensor calls, action dispatches or model feedback are introduced.
    Protected modules, safety ownership and the deployment clock are unchanged.
    """
    def __init__(self, *args, **kwargs):
        self._shadow_receipts = ActionReceipts()
        self._previews = []
        self._preview_checks = []
        self._shadow_errors = []
        self._capture = None
        super().__init__(*args, **kwargs)

    def _shadow_error(self, stage, exc):
        self._shadow_errors.append(dict(stage=stage, exception_type=type(exc).__name__,
            detail=str(exc)[:300], sim_time_sec=self._world.time_sec))

    def _log(self, event, **data):
        event_id = super()._log(event, **data)
        if event in ('tool_call_started','tool_call_finished'):
            try:
                self._shadow_receipts.accept_event(self._events[-1])
            except Exception as exc:
                self._shadow_error('action_receipt', exc)
        return event_id

    def _observe(self):
        try:
            measured = super()._observe()
        except ObservationError as exc:
            if self._capture and self._capture['pending']:
                self._capture['pending'] = False
                self._capture['observation_error'] = exc.code
            raise
        capture = self._capture
        if capture and capture['pending']:
            capture['pending'] = False
            started = time.perf_counter()
            try:
                plan = self._routes[capture['route_id']]
                preview = preview_trajectory(observation=measured,
                    episode_id=self._world.episode_id, current_frame_id=self._world.frame_id,
                    now=self._world.time_sec, tracker=plan['tracker'],
                    target_xy=self._world.checkpoints[plan['target']], config=self._world.plant.config,
                    segment_seconds=capture['duration'], preview_id=f'preview-{len(self._previews)+1}',
                    route_id=capture['route_id'], operation_id=capture['operation_id'])
                preview.update(plan_id=plan['plan_id'], target=plan['target'], route=plan['route'],
                    calculation_wall_sec=time.perf_counter()-started)
                capture['preview'] = preview
            except Exception as exc:
                self._shadow_error('trajectory_preview', exc)
        return measured

    def execute_route(self, *, route_id, operation_id, max_sim_seconds=8., rationale=''):
        # An idempotent retry is a historical receipt, never a new prediction.
        cached = isinstance(operation_id, str) and operation_id in self._operations
        if cached:
            return super().execute_route(route_id=route_id, operation_id=operation_id,
                                        max_sim_seconds=max_sim_seconds, rationale=rationale)
        capture = dict(pending=True, route_id=route_id, operation_id=operation_id,
                       duration=max_sim_seconds, trace_start=len(self._trace),
                       requested_sim_time_sec=self._world.time_sec)
        self._capture = capture
        try:
            result = super().execute_route(route_id=route_id, operation_id=operation_id,
                                          max_sim_seconds=max_sim_seconds, rationale=rationale)
        finally:
            self._capture = None
        try:
            preview = capture.get('preview')
            if preview is None:
                preview = dict(schema_version=1, preview_id=f'preview-{len(self._previews)+1}',
                    route_id=route_id, operation_id=operation_id, status='unavailable',
                    reason=capture.get('observation_error', result.get('reason') or 'NO_EXECUTION_ANCHOR'),
                    role='read_only_shadow', motion_authorized=False, samples=[])
            preview.update(execution_admitted=operation_id in self._operations,
                           requested_sim_time_sec=capture['requested_sim_time_sec'],
                           execution_result_reason=result.get('reason'))
            check = compare_preview(preview, self._trace[capture['trace_start']:])
            check['execution_result_reason'] = result.get('reason')
            self._previews.append(preview)
            self._preview_checks.append(check)
        except Exception as exc:
            self._shadow_error('prediction_comparison', exc)
        return result

    def shadow_evidence(self):
        return deepcopy(dict(schema_version=1, mode='read_only_shadow',
            trajectory_previews=self._previews, preview_checks=self._preview_checks,
            action_contracts=self._shadow_receipts.contracts,
            recovery_decisions=self._shadow_receipts.recovery_decisions,
            shadow_errors=self._shadow_errors, extra_model_calls=0,
            tool_feedback_changed=False, additional_sensor_calls=0,
            validation_scope='synthetic_2d_kinematics_only'))

    def save(self, directory, *, tool_context=None):
        super().save(directory, tool_context=tool_context)
        path = Path(directory)
        evidence = self.shadow_evidence()
        hashes = {}
        for key in ('trajectory_previews','preview_checks','action_contracts','recovery_decisions','shadow_errors'):
            target = path / f'{key}.jsonl'
            target.write_text(''.join(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n'
                                      for row in evidence[key]), encoding='utf-8')
            hashes[target.name] = hashlib.sha256(target.read_bytes()).hexdigest()
        metadata = {k:v for k,v in evidence.items() if k not in
                    ('trajectory_previews','preview_checks','action_contracts','recovery_decisions','shadow_errors')}
        metadata['files'] = hashes
        metadata['base_run_sha256'] = hashlib.sha256((path/'run.json').read_bytes()).hexdigest()
        (path/'shadow.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def attach_shadow(evidence, directory):
    """Attach only a sidecar whose hashes match this existing base run."""
    path = Path(directory)
    metadata = json.loads((path/'shadow.json').read_text(encoding='utf-8'))
    if metadata.get('mode') != 'read_only_shadow':
        raise ValueError('Unsupported shadow mode')
    if metadata['base_run_sha256'] != hashlib.sha256((path/'run.json').read_bytes()).hexdigest():
        raise ValueError('Shadow sidecar does not bind to the base run')
    result = deepcopy(evidence)
    for key in ('trajectory_previews','preview_checks','action_contracts','recovery_decisions','shadow_errors'):
        name = key+'.jsonl'
        data = (path/name).read_bytes()
        if metadata['files'].get(name) != hashlib.sha256(data).hexdigest():
            raise ValueError('Shadow sidecar hash mismatch: '+name)
        result[key] = [json.loads(row) for row in data.decode('utf-8').splitlines() if row.strip()]
    result['shadow_metadata'] = metadata
    return result
