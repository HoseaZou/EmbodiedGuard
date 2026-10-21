from __future__ import annotations

import math
from typing import Tuple


def wrap_degrees(angle_deg: float) -> float:
    """Wrap an angle to [-180, 180)."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def world_to_ego_xy(
    target_x: float,
    target_y: float,
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
) -> Tuple[float, float]:
    """Project CARLA world XY into ego-local forward/right coordinates.

    The returned convention matches the existing Action Token decoder:
    ``x`` is forward and positive ``y`` is to the vehicle's right.
    """
    dx = float(target_x) - float(ego_x)
    dy = float(target_y) - float(ego_y)
    yaw = math.radians(float(ego_yaw_deg))
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def ego_to_world_xy(
    local_x: float,
    local_y: float,
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
) -> Tuple[float, float]:
    """Convert ego-local forward/right coordinates back to CARLA world XY."""
    yaw = math.radians(float(ego_yaw_deg))
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    world_x = float(ego_x) + cosine * float(local_x) - sine * float(local_y)
    world_y = float(ego_y) + sine * float(local_x) + cosine * float(local_y)
    return world_x, world_y
