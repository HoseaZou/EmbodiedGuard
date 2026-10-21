from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from typing import Any, Dict, Mapping, Optional, Tuple


ROAD_OPTIONS = frozenset(
    {
        "LANEFOLLOW",
        "STRAIGHT",
        "LEFT",
        "RIGHT",
        "CHANGELANELEFT",
        "CHANGELANERIGHT",
        "VOID",
    }
)

ROUTE_PLAN_SCHEMA = "safehybriddrive.route_plan.v1"
ROUTE_PLAN_IDENTITY_SCHEMA = "safehybriddrive.route_plan_identity.v1"
ROUTE_STATE_SCHEMA = "safehybriddrive.route_state.v1"


def normalize_road_option(value: Any) -> str:
    """Convert CARLA RoadOption values into a stable JSON-safe string."""
    raw = getattr(value, "name", value)
    text = str(raw or "VOID").rsplit(".", 1)[-1].replace("_", "").upper()
    aliases = {
        "FOLLOWLANE": "LANEFOLLOW",
        "LANECHANGELEFT": "CHANGELANELEFT",
        "LANECHANGERIGHT": "CHANGELANERIGHT",
    }
    text = aliases.get(text, text)
    return text if text in ROAD_OPTIONS else "VOID"


@dataclass(frozen=True)
class RoutePoint:
    index: int
    x: float
    y: float
    z: float = 0.0
    yaw_deg: float = 0.0
    road_id: int = 0
    section_id: int = 0
    lane_id: int = 0
    s: float = 0.0
    road_option: str = "LANEFOLLOW"
    cumulative_distance_m: float = 0.0

    def __post_init__(self) -> None:
        if self.index < 0:
            raise ValueError("route point index must be non-negative")
        for name in ("x", "y", "z", "yaw_deg", "s", "cumulative_distance_m"):
            if not math.isfinite(float(getattr(self, name))):
                raise ValueError(f"route point {name} must be finite")
        if self.cumulative_distance_m < 0.0:
            raise ValueError("cumulative route distance must be non-negative")
        object.__setattr__(self, "road_option", normalize_road_option(self.road_option))

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class RoutePlan:
    route_id: str
    map_name: str
    start: Mapping[str, float]
    destination: Mapping[str, float]
    sampling_resolution_m: float
    points: Tuple[RoutePoint, ...]
    total_distance_m: float

    def __post_init__(self) -> None:
        if not self.route_id:
            raise ValueError("route_id is required")
        if not self.points:
            raise ValueError("route plan must contain at least one point")
        if self.sampling_resolution_m <= 0.0:
            raise ValueError("sampling_resolution_m must be positive")
        previous_distance = -1.0
        for expected_index, point in enumerate(self.points):
            if point.index != expected_index:
                raise ValueError("route point indexes must be contiguous and zero-based")
            if point.cumulative_distance_m < previous_distance:
                raise ValueError("route cumulative distances must be monotonic")
            previous_distance = point.cumulative_distance_m
        if not math.isfinite(float(self.total_distance_m)) or self.total_distance_m < 0.0:
            raise ValueError("total_distance_m must be finite and non-negative")
        if abs(self.total_distance_m - self.points[-1].cumulative_distance_m) > 1e-3:
            raise ValueError("total_distance_m must match the final cumulative distance")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema": ROUTE_PLAN_SCHEMA,
            "route_id": self.route_id,
            "map_name": self.map_name,
            "start": dict(self.start),
            "destination": dict(self.destination),
            "sampling_resolution_m": float(self.sampling_resolution_m),
            "points": [point.to_dict() for point in self.points],
            "total_distance_m": float(self.total_distance_m),
        }


def route_plan_identity(route_plan: RoutePlan) -> Dict[str, Any]:
    """Return a compact identity for topology-sensitive route artifacts."""

    def _float_token(value: float) -> str:
        return format(float(value), ".6f")

    point_payload = [
        [
            int(point.index),
            _float_token(point.x),
            _float_token(point.y),
            _float_token(point.z),
            _float_token(point.yaw_deg),
            int(point.road_id),
            int(point.section_id),
            int(point.lane_id),
            _float_token(point.s),
            str(point.road_option),
            _float_token(point.cumulative_distance_m),
        ]
        for point in route_plan.points
    ]
    points_sha256 = hashlib.sha256(
        json.dumps(
            point_payload,
            ensure_ascii=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    return {
        "schema": ROUTE_PLAN_IDENTITY_SCHEMA,
        "route_id": route_plan.route_id,
        "map_name": route_plan.map_name,
        "sampling_resolution_m": float(route_plan.sampling_resolution_m),
        "point_count": len(route_plan.points),
        "total_distance_m": float(route_plan.total_distance_m),
        "points_sha256": points_sha256,
    }


def validate_route_plan_identity(
    expected: Mapping[str, Any],
    route_plan: RoutePlan,
) -> Dict[str, Any]:
    """Reject a site/runtime route mismatch before actors are materialized."""

    if not isinstance(expected, Mapping):
        raise ValueError("route plan identity is missing")
    actual = route_plan_identity(route_plan)
    mismatches = []
    for key in ("schema", "route_id", "map_name", "point_count", "points_sha256"):
        if expected.get(key) != actual[key]:
            mismatches.append(key)
    for key, tolerance in (
        ("sampling_resolution_m", 1e-9),
        ("total_distance_m", 1e-3),
    ):
        try:
            matches = (
                abs(float(expected.get(key)) - float(actual[key]))
                <= tolerance
            )
        except (TypeError, ValueError):
            matches = False
        if not matches:
            mismatches.append(key)
    if mismatches:
        raise ValueError(
            "route plan identity mismatch: " + ", ".join(mismatches)
        )
    return actual


@dataclass(frozen=True)
class RouteState:
    route_id: str
    route_revision: int
    cursor: int
    route_size: int
    route_progress: float
    next_navigation_action: str
    target_waypoint_world: Mapping[str, float]
    target_waypoint_ego: Mapping[str, float]
    preview_waypoints_ego: Tuple[Mapping[str, Any], ...]
    target_lane_id: Optional[int]
    distance_to_target_m: float
    distance_to_goal_m: float
    cross_track_error_m: float
    heading_error_deg: float
    route_deviation: bool
    recovery_active: bool
    recovery_plan: Optional[str]
    local_task_active: bool
    route_restored: bool
    route_completed: bool
    signed_cross_track_error_m: float = 0.0
    # RouteManager owns the finite-horizon geometry consumed by the runtime
    # adapter.  These values are ego-local and are deliberately separate from
    # the model's trajectory tokens.
    route_tangent_yaw_rad: float = 0.0
    route_curvature_1pm: float = 0.0
    route_tangent_lookahead_m: float = 0.0
    route_tangent_source: str = "src.route.RouteManager.finite_horizon"
    route_tracking_preview: Optional[Mapping[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        value = asdict(self)
        value["schema"] = ROUTE_STATE_SCHEMA
        return value

    def to_context(self) -> Dict[str, Any]:
        """Return the compact runtime/trace handoff fields.

        This object is not injected into the Stage 1/2 training sequence. The
        numerical waypoint fields stay available to Route, Safety and control.
        """
        return {
            "route_progress": float(self.route_progress),
            "next_navigation_action": normalize_road_option(self.next_navigation_action),
            "target_waypoint_ego": dict(self.target_waypoint_ego),
            "distance_to_goal_m": float(self.distance_to_goal_m),
            "signed_cross_track_error_m": float(
                self.signed_cross_track_error_m
            ),
            "route_tangent_yaw_rad": float(self.route_tangent_yaw_rad),
            "route_curvature_1pm": float(self.route_curvature_1pm),
            "route_tangent_lookahead_m": float(
                self.route_tangent_lookahead_m
            ),
            "route_deviation": bool(self.route_deviation),
            "recovery_active": bool(self.recovery_active),
            "local_task_active": bool(self.local_task_active),
            "route_completed": bool(self.route_completed),
        }


def _finite_number(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _bool_text(value: Any) -> str:
    if isinstance(value, str):
        return "true" if value.strip().lower() in {"1", "true", "yes", "on"} else "false"
    return "true" if bool(value) else "false"


def route_context_to_text(route_context: Any) -> str:
    """Format a deterministic runtime diagnostic block; empty input stays empty.

    Keeping this formatter in the Route boundary does not make Route values
    model tokens. Training sequence builders intentionally do not call it.
    """
    if route_context is None:
        return ""
    if isinstance(route_context, RouteState):
        context: Mapping[str, Any] = route_context.to_context()
    elif isinstance(route_context, Mapping):
        context = route_context
    elif hasattr(route_context, "to_context"):
        context = route_context.to_context()
    else:
        raise TypeError("route_context must be a mapping, RouteState, or None")
    if not context:
        return ""

    lines = ["[ROUTE_CONTEXT]"]
    if "next_navigation_action" in context:
        lines.append(
            "next_navigation_action="
            + normalize_road_option(context.get("next_navigation_action"))
        )
    target = context.get("target_waypoint_ego")
    if isinstance(target, Mapping):
        x_value = _finite_number(target.get("x"))
        y_value = _finite_number(target.get("y"))
        if x_value is not None:
            lines.append(f"target_waypoint_ego_x_m={x_value:.3f}")
        if y_value is not None:
            lines.append(f"target_waypoint_ego_y_m={y_value:.3f}")
    for key in ("route_progress", "distance_to_goal_m"):
        number = _finite_number(context.get(key))
        if number is not None:
            lines.append(f"{key}={number:.3f}")
    for key in (
        "route_deviation",
        "recovery_active",
        "local_task_active",
        "route_completed",
    ):
        if key in context:
            lines.append(f"{key}={_bool_text(context.get(key))}")
    return "\n".join(lines) if len(lines) > 1 else ""
