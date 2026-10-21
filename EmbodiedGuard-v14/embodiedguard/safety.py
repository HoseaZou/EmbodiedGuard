"""Deterministic checks of the final command and its braking continuation."""
from __future__ import annotations

from dataclasses import dataclass
import math

from .sim import Action, PlantConfig, PoseState, STOP, integrate


def segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    fraction = max(0., min(1., ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy))) if dx or dy else 0.
    return math.hypot(px - ax - fraction * dx, py - ay - fraction * dy)


def swept_clearance(previous: PoseState, current: PoseState, obstacles: list[dict],
                    elapsed_from_observation: float, dt: float, radius: float) -> float:
    clearance = math.inf
    for obj in obstacles:
        # Relative line sweep includes constant-velocity obstacle motion.
        x0, y0 = obj['x'] + obj['vx'] * elapsed_from_observation, obj['y'] + obj['vy'] * elapsed_from_observation
        x1, y1 = x0 + obj['vx'] * dt, y0 + obj['vy'] * dt
        clearance = min(clearance, segment_distance(0., 0., previous.x - x0, previous.y - y0,
                                                     current.x - x1, current.y - y1) - obj['radius'] - radius)
    return clearance


def inside_bounds(state: PoseState, bounds: tuple, radius: float) -> bool:
    xmin, ymin, xmax, ymax = bounds
    return xmin + radius <= state.x <= xmax - radius and ymin + radius <= state.y <= ymax - radius


@dataclass(frozen=True)
class SafetyVerdict:
    allowed: bool
    reason: str
    min_clearance_m: float | None
    horizon_sec: float


def assess_action(state: PoseState, action: Action, observation: dict, *, now: float,
                  config: PlantConfig, bounds: tuple) -> SafetyVerdict:
    if observation.get('obstacles_valid') is not True or not isinstance(observation.get('obstacles'), list):
        return SafetyVerdict(False, 'UNKNOWN_OBSERVATION', None, 0.)
    if not 0 <= now - observation['observed_at'] <= .3 + 1e-9:
        return SafetyVerdict(False, 'STALE_OBSERVATION', None, 0.)
    elapsed, minimum = 0., math.inf
    predicted = state
    # Maximum one action step followed by a straight full-brake stop.
    max_steps = math.ceil((config.max_speed_mps / config.braking_mps2 + .2) / config.dt_sec) + 2
    for index in range(max_steps):
        command = action if index == 0 else STOP
        after = integrate(predicted, command, config.dt_sec, config)
        if after.speed_mps > config.max_speed_mps + 1e-9:
            return SafetyVerdict(False, 'SPEED_LIMIT', None, elapsed)
        if not inside_bounds(after, bounds, config.radius_m + config.safety_margin_m):
            return SafetyVerdict(False, 'BOUNDARY_RISK', None, elapsed)
        minimum = min(minimum, swept_clearance(predicted, after, observation['obstacles'],
                                               now - observation['observed_at'] + elapsed,
                                               config.dt_sec, config.radius_m + config.safety_margin_m))
        elapsed += config.dt_sec
        if minimum < 0:
            return SafetyVerdict(False, 'PATH_BLOCKED', minimum, elapsed)
        predicted = after
        if after.speed_mps == 0:
            break
    else:
        return SafetyVerdict(False, 'STOPPING_HORIZON_EXCEEDED', None, elapsed)
    # A stopped vehicle must also be outside the future ray of each observed
    # constant-velocity object. Otherwise an apparently successful brake could
    # leave it parked in crossing traffic while the Agent is deciding/waiting.
    # Conservative for this synthetic motion model; not a prediction guarantee
    # for real objects that may accelerate or change direction.
    for obj in observation['obstacles']:
        age = now - observation['observed_at'] + elapsed
        dx = obj['x'] + obj['vx'] * age - predicted.x
        dy = obj['y'] + obj['vy'] * age - predicted.y
        velocity2 = obj['vx']**2 + obj['vy']**2
        closest_t = max(0., -(dx * obj['vx'] + dy * obj['vy']) / velocity2) if velocity2 else 0.
        clearance = math.hypot(dx + obj['vx'] * closest_t, dy + obj['vy'] * closest_t) - obj['radius'] - config.radius_m - config.safety_margin_m
        minimum = min(minimum, clearance)
    if minimum < 0:
        return SafetyVerdict(False, 'PATH_BLOCKED', minimum, elapsed)
    return SafetyVerdict(True, 'OK', minimum if math.isfinite(minimum) else None, elapsed)
