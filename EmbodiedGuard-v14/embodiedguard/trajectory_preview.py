"""Conditional, read-only controller predictions; never a motion permission."""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict
import math

from .sim import Action, PoseState, finite_number, integrate
from .state import ObservationError, ObservationStore


def preview_trajectory(*, observation, episode_id, current_frame_id, now,
                       tracker, target_xy, config, segment_seconds,
                       preview_id, route_id, operation_id, horizon_sec=3.):
    """Clone the *current* tracker, not a newly reset route or sensor.

    Sampling uses the frozen controller and plant step. Obstacle evolution,
    safety interventions, future sensor faults and mandatory terminal braking
    are deliberately not predicted. The live executor checks every action.
    An unavailable prediction contains no guessed pose or trajectory samples.
    """
    result = dict(schema_version=1, preview_id=preview_id, route_id=route_id,
        operation_id=operation_id, status='unavailable', reason=None,
        role='read_only_shadow', motion_authorized=False,
        method='cloned_live_tracker_ackermann_rollout', samples=[])
    try:
        now = finite_number(now, 'now')
        duration = finite_number(segment_seconds, 'segment_seconds')
        horizon = finite_number(horizon_sec, 'horizon_sec')
        if not .1 <= duration <= 15 or not 0 < horizon <= 3:
            raise ValueError('bounded segment/preview horizon required')
        if type(current_frame_id) is not int or current_frame_id < 0:
            raise ValueError('current frame must be a nonnegative integer')
        if tracker is None or not callable(getattr(tracker, 'command', None)):
            raise ValueError('an existing route tracker is required')
        if not isinstance(target_xy, (tuple, list)) or len(target_xy) != 2:
            raise ValueError('target must contain two finite coordinates')
        target = tuple(finite_number(v, 'target') for v in target_xy)
        isolated = ObservationStore(episode_id)
        if not isinstance(observation, dict):
            raise ObservationError('UNKNOWN_OBSERVATION', 'no measured snapshot')
        if 'received_at' not in observation:
            raise ObservationError('INVALID_OBSERVATION', 'missing received_at')
        isolated.ingest(observation, received_at=observation['received_at'])
        measured = isolated.current(now=now)
        if measured['frame_id'] != current_frame_id:
            raise ObservationError('STALE_OBSERVATION', 'controller requires a same-frame snapshot')
        anchor = PoseState(**measured['pose'], speed_mps=measured['speed_mps'])
        state = anchor
        clone = deepcopy(tracker)
        clock, frame = now, current_frame_id
        end = now + min(duration, horizon)
        cosine, sine = math.cos(anchor.yaw_rad), math.sin(anchor.yaw_rad)
        samples = []

        def sample():
            dx, dy = state.x-anchor.x, state.y-anchor.y
            return dict(t_rel_sec=clock-now, sim_time_sec=clock, frame_id=frame,
                **asdict(state), x_forward_m=cosine*dx+sine*dy,
                y_left_m=-sine*dx+cosine*dy,
                delta_yaw_rad=math.atan2(math.sin(state.yaw_rad-anchor.yaw_rad),
                                        math.cos(state.yaw_rad-anchor.yaw_rad)))

        samples.append(sample())
        while clock < end - 1e-9:
            if math.dist((state.x, state.y), target) <= .6 and state.speed_mps <= .05:
                break
            # Match the executor's fixed step, including its segment-end convention.
            control = clone.command(x=state.x, y=state.y, yaw_rad=state.yaw_rad,
                speed_mps=state.speed_mps, frame_id=frame, sim_time_sec=clock,
                dt_sec=config.dt_sec)
            action = Action(throttle=control['throttle'], brake=control['brake'],
                            steer_left_normalized=control['steer_left_normalized'])
            state = integrate(state, action, config.dt_sec, config)
            clock += config.dt_sec
            frame += 1
            samples.append(sample())
        # The prediction is accepted only if its entire serializable value is finite.
        for item in samples:
            for value in item.values():
                finite_number(value, 'predicted_sample')
        result.update(status='available', reason=None, generated_sim_time_sec=now,
            source_observation={key: measured[key] for key in
                ('episode_id','frame_id','observed_at','received_at','source','age_sec')},
            anchor_state=asdict(anchor), coordinate_frame='world',
            local_coordinate_frame='measured_anchor_x_forward_y_left_yaw_ccw',
            units=dict(position='m', angle='rad', time='s', speed='m/s'),
            sample_step_sec=config.dt_sec, requested_horizon_sec=min(duration,horizon),
            sampled_horizon_sec=clock-now, samples=samples,
            constraints=asdict(config), validity='historical prediction bound to this operation and measured anchor; never transferable motion authorization',
            assumptions=['same live route and controller state', 'same fixed plant and actuator mapping',
                         'no future safety interruption, sensor failure or environment response'],
            excludes=['obstacle forecast','safety permission','mandatory braking after the segment',
                      'vehicle dynamics','real robot validation','learned VLA output'])
    except ObservationError as exc:
        result.update(reason=exc.code, detail=str(exc)[:300])
    except Exception as exc:
        result.update(reason='PREVIEW_UNAVAILABLE', detail=type(exc).__name__+': '+str(exc)[:300])
    return result


def compare_preview(preview, actual_trace):
    """Match recorded post-action samples by operation, frame and time; never interpolate."""
    if preview['status'] != 'available':
        return dict(preview_id=preview['preview_id'], status='unavailable',
                    reason=preview['reason'], matched_samples=0, errors=[])
    rows = {row['frame_id']: row for row in actual_trace
            if row.get('operation_id') == preview['operation_id']}
    errors = []
    missing = 0
    for predicted in preview['samples'][1:]:
        actual = rows.get(predicted['frame_id'])
        if actual is None or abs(actual['sim_time_sec']-predicted['sim_time_sec']) > 1e-9:
            missing += 1
            continue
        state = actual['state']
        errors.append(dict(frame_id=actual['frame_id'], sim_time_sec=actual['sim_time_sec'],
            position_error_m=math.dist((state['x'],state['y']), (predicted['x'],predicted['y'])),
            yaw_error_rad=abs(math.atan2(math.sin(state['yaw_rad']-predicted['yaw_rad']),
                                       math.cos(state['yaw_rad']-predicted['yaw_rad']))),
            speed_error_mps=abs(state['speed_mps']-predicted['speed_mps']),
            actual_action=deepcopy(actual['action']),
            safety_assessment=deepcopy(actual.get('safety_assessment'))))
    positions = [item['position_error_m'] for item in errors]
    return dict(preview_id=preview['preview_id'], status='compared' if errors else 'no_matching_motion',
        comparison_source='simulator_trace_evaluator_only', matched_samples=len(errors),
        unmatched_prediction_samples=missing, errors=errors,
        position_rmse_m=math.sqrt(sum(v*v for v in positions)/len(positions)) if positions else None,
        position_max_error_m=max(positions) if positions else None,
        note='Same-model consistency, not evidence of calibrated prediction accuracy or safety; interruptions may cause divergence.')
