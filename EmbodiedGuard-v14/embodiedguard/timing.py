"""Conditional travel forecast in an isolated, obstacle-free kinematic rollout.

This is neither a sensor nor a safety proof. The real executor still validates
freshness, obstacles and actuator limits independently at every motion step.
"""
from dataclasses import asdict
from copy import deepcopy
import math

from .control import LegacyTracker
from .sim import Action, PoseState, STOP, integrate

POST_ARRIVAL_RESERVE_SEC = 5.0  # Fixed local admission policy, not Agent editable.


def _rollout(points, pose, target, config, *, segment_seconds=15., route_tracker=None,
             frame_id=0, sim_time_sec=0.):
    # A resumed route owns a projection cursor. Forecast from a deep copy of
    # that cursor; rebuilding it from point zero changes the tracking problem.
    tracker = deepcopy(route_tracker) if route_tracker is not None else LegacyTracker(points, cruise_speed_mps=1.2)
    elapsed, frame, operations = 0., frame_id, 0
    current = pose
    horizon = getattr(tracker, 'forecast_horizon_sec', 90.)
    while elapsed < horizon:
        if math.dist((current.x, current.y), target) <= .6 and current.speed_mps <= .05:
            break
        operations += 1
        segment_end = elapsed + (segment_seconds if operations == 1 else 15.)
        while elapsed < segment_end - 1e-9 and elapsed < horizon:
            if math.dist((current.x, current.y), target) <= .6 and current.speed_mps <= .05:
                break
            command = tracker.command(x=current.x, y=current.y, yaw_rad=current.yaw_rad,
                speed_mps=current.speed_mps, frame_id=frame, sim_time_sec=sim_time_sec + elapsed,
                dt_sec=config.dt_sec)
            current = integrate(current, Action(throttle=command['throttle'], brake=command['brake'],
                steer_left_normalized=command['steer_left_normalized']), config.dt_sec, config)
            elapsed += config.dt_sec
            frame += 1
        while current.speed_mps > 0:
            current = integrate(current, STOP, config.dt_sec, config)
            elapsed += config.dt_sec
            frame += 1
        tracker.reset_controller()
        if math.dist((current.x, current.y), target) <= .6:
            break
    arrived = math.dist((current.x, current.y), target) <= .6 and current.speed_mps == 0
    estimate = elapsed if arrived else None
    return dict(method='isolated_kinematic_controller_rollout', guaranteed=False,
        estimated_arrival_and_stop_sec=estimate, estimated_execution_segments=operations if arrived else None,
        post_arrival_reserve_sec=POST_ARRIVAL_RESERVE_SEC,
        admission_required_sec=estimate + POST_ARRIVAL_RESERVE_SEC if arrived else None,
        segment_seconds=segment_seconds, assumed_followup_segment_seconds=15., start_pose=asdict(pose),
        assumptions=['same controller and fixed-step Ackermann plant', 'no future obstacle or sensor fault',
                     'fresh measured start pose', 'each segment ends with braking; follow-up segments use 15s',
                     '5s post-arrival reserve is a policy margin, not a model-latency guarantee'],
        excludes=['future decision and approval waits', 'future recovery', 'future targets'],
        validation_scope='synthetic_2d_kinematics_only')


def forecast(points, pose, target, config, **kwargs):
    try:
        return _rollout(points, pose, target, config, **kwargs)
    except Exception as exc:
        # A failed predictor must not invent a duration or authorize motion.
        return dict(method='isolated_kinematic_controller_rollout', guaranteed=False,
            estimated_arrival_and_stop_sec=None, admission_required_sec=None,
            post_arrival_reserve_sec=POST_ARRIVAL_RESERVE_SEC,
            unavailable_reason=type(exc).__name__, validation_scope='synthetic_2d_kinematics_only')
