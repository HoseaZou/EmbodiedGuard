from __future__ import annotations

import copy
import json
import math
from typing import Any

from embodiedguard.legacy.backend.route_context import (
    NAVIGATION_COMMAND_TEXT,
    ROUTE_CONTEXT_SCHEMA_VERSION,
    validate_route_context,
)
from embodiedguard.legacy.route.route_manager import RouteManager
from embodiedguard.legacy.route.route_types import RouteState, normalize_road_option
from embodiedguard.legacy.route.coordinate_adapter import wrap_degrees, world_to_ego_xy


GLOBAL_ROUTE_RUNTIME_ADAPTER_VERSION = (
    "safehybriddrive.global_route_runtime_adapter.v1"
)

_NAVIGATION_ACTIONS = {
    "LEFT": "turn_left",
    "RIGHT": "turn_right",
    "STRAIGHT": "go_straight",
    "LANEFOLLOW": "follow_lane",
    "CHANGELANELEFT": "change_lane_left",
    "CHANGELANERIGHT": "change_lane_right",
    "VOID": "void",
}


def _nonnegative_integer(value: Any, name: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be numeric")
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _nonnegative_float(value: Any, name: str) -> float:
    parsed = _finite_float(value, name)
    if parsed < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return parsed


def _navigation_action(road_option: Any) -> str:
    return _NAVIGATION_ACTIONS[normalize_road_option(road_option)]


class GlobalRouteRuntimeAdapter:
    """Expose the canonical global RouteManager through the runtime contract.

    The wrapped ``src.route.RouteManager`` remains the only route-state owner.
    This adapter only serializes its ``RouteState`` for the existing Stage 3,
    Safety, controller and trace interfaces.
    """

    def __init__(self, route_manager: RouteManager) -> None:
        if not isinstance(route_manager, RouteManager):
            raise ValueError("route_manager must be src.route.RouteManager")
        self.route_manager = route_manager
        self._last_frame_id: int | None = None
        self._last_observed_monotonic_ns: int | None = None
        self._last_progress_m = 0.0
        self._last_deviated = False
        self._updates = 0
        self._last_context: dict[str, Any] | None = None

    @property
    def route_id(self) -> str:
        return self.route_manager.plan.route_id

    def update(
        self,
        *,
        frame_id: int,
        observed_monotonic_ns: int,
        ego_x: float,
        ego_y: float,
        ego_yaw_deg: float,
        speed_mps: float,
    ) -> dict[str, Any]:
        frame = _nonnegative_integer(frame_id, "frame_id")
        observed_ns = _nonnegative_integer(
            observed_monotonic_ns,
            "observed_monotonic_ns",
        )
        if self._last_frame_id is not None and frame <= self._last_frame_id:
            raise ValueError("frame_id must be strictly increasing")
        if (
            self._last_observed_monotonic_ns is not None
            and observed_ns < self._last_observed_monotonic_ns
        ):
            raise ValueError("observed_monotonic_ns must be monotonic")

        ego_x_value = _finite_float(ego_x, "ego_x")
        ego_y_value = _finite_float(ego_y, "ego_y")
        ego_yaw_value = _finite_float(ego_yaw_deg, "ego_yaw_deg")
        state = self.route_manager.update(
            ego_x=ego_x_value,
            ego_y=ego_y_value,
            ego_yaw_deg=ego_yaw_value,
            speed_mps=_nonnegative_float(speed_mps, "speed_mps"),
        )
        context = self._serialize_state(
            state,
            frame_id=frame,
            observed_monotonic_ns=observed_ns,
            ego_x=ego_x_value,
            ego_y=ego_y_value,
            ego_yaw_deg=ego_yaw_value,
        )
        errors = validate_route_context(context)
        if errors:
            raise RuntimeError(
                "global RouteState produced invalid runtime context: "
                + ",".join(errors[:10])
            )
        json.dumps(context, allow_nan=False)

        self._last_frame_id = frame
        self._last_observed_monotonic_ns = observed_ns
        self._last_progress_m = context["route_progress_m"]
        self._last_deviated = state.route_deviation
        self._updates += 1
        self._last_context = copy.deepcopy(context)
        return copy.deepcopy(context)

    def _serialize_state(
        self,
        state: RouteState,
        *,
        frame_id: int,
        observed_monotonic_ns: int,
        ego_x: float,
        ego_y: float,
        ego_yaw_deg: float,
    ) -> dict[str, Any]:
        plan = self.route_manager.plan
        route_length_m = float(plan.total_distance_m)
        if route_length_m <= 0.0:
            raise ValueError("global route length must be positive")
        progress_m = min(
            route_length_m,
            max(0.0, float(state.route_progress) * route_length_m),
        )
        action = _navigation_action(state.next_navigation_action)
        action_distance_m = self._navigation_action_distance(
            state,
            progress_m=progress_m,
            action=action,
        )
        waypoints = []
        for item in state.preview_waypoints_ego:
            index = int(item["index"])
            point = plan.points[index]
            route_distance_m = float(point.cumulative_distance_m)
            waypoints.append(
                {
                    "route_distance_m": route_distance_m,
                    "lookahead_m": max(0.0, route_distance_m - progress_m),
                    "x_forward_m": float(item["x"]),
                    # RouteManager uses CARLA ego-local right-positive y.
                    # The runtime trajectory contract is left-positive.
                    "y_left_m": -float(item["y"]),
                }
            )
        if not waypoints:
            point = plan.points[state.cursor]
            waypoints.append(
                {
                    "route_distance_m": float(point.cumulative_distance_m),
                    "lookahead_m": max(
                        0.0,
                        float(point.cumulative_distance_m) - progress_m,
                    ),
                    "x_forward_m": float(state.target_waypoint_ego["x"]),
                    "y_left_m": -float(state.target_waypoint_ego["y"]),
                }
            )
        # RouteManager follows CARLA's ego-local right-positive convention.
        # The Stage 3/runtime contract is left-positive for both lateral
        # displacement and yaw, so both fields must cross the same sign
        # boundary here.
        route_tangent_yaw_rad = -float(state.route_tangent_yaw_rad)
        route_curvature_1pm = float(state.route_curvature_1pm)

        # Bench2Drive's RouteCompletionTest advances its final index only
        # after the ego crosses the last route transform's forward plane.
        # Distance to the point is not equivalent to that signed criterion.
        terminal = plan.points[-1]
        terminal_forward_ego, terminal_right_ego = world_to_ego_xy(
            terminal.x,
            terminal.y,
            ego_x,
            ego_y,
            ego_yaw_deg,
        )
        terminal_yaw_rad = math.radians(float(terminal.yaw_deg))
        terminal_dx = float(ego_x) - float(terminal.x)
        terminal_dy = float(ego_y) - float(terminal.y)
        terminal_plane_signed_progress_m = (
            terminal_dx * math.cos(terminal_yaw_rad)
            + terminal_dy * math.sin(terminal_yaw_rad)
        )
        terminal_plane_lateral_m = (
            -terminal_dx * math.sin(terminal_yaw_rad)
            + terminal_dy * math.cos(terminal_yaw_rad)
        )
        # Runtime yaw is left-positive while CARLA yaw is right-positive.
        terminal_heading_error_rad = -math.radians(
            wrap_degrees(float(terminal.yaw_deg) - float(ego_yaw_deg))
        )

        recovered = self._last_deviated and not state.route_deviation
        route_state = (
            "completed"
            if state.route_completed
            else "recovered"
            if recovered
            else "deviated"
            if state.route_deviation
            else "tracking"
        )
        heading_deviated = (
            float(state.heading_error_deg)
            > float(self.route_manager.config.deviation_heading_deg)
        )
        return {
            "schema_version": ROUTE_CONTEXT_SCHEMA_VERSION,
            "route_id": state.route_id,
            "source_frame_id": frame_id,
            "observed_monotonic_ns": observed_monotonic_ns,
            "route_cursor_index": int(state.cursor),
            "route_length_m": route_length_m,
            "route_progress_m": progress_m,
            "route_progress": float(state.route_progress),
            "progress_delta_m": max(0.0, progress_m - self._last_progress_m),
            "distance_to_route_m": float(state.cross_track_error_m),
            "signed_distance_to_route_m": float(
                state.signed_cross_track_error_m
            ),
            "distance_to_goal_m": float(state.distance_to_goal_m),
            "heading_error_deg": float(state.heading_error_deg),
            "next_navigation_action": action,
            "navigation_command_text": NAVIGATION_COMMAND_TEXT[action],
            "navigation_action_distance_m": action_distance_m,
            "ego_local_waypoints": waypoints,
            "route_tangent_yaw_rad": route_tangent_yaw_rad,
            "route_tracking_preview": (
                None if state.route_tracking_preview is None else {
                    **state.route_tracking_preview,
                    "yaw_offset_rad": -float(
                        state.route_tracking_preview["yaw_offset_rad"]
                    ),
                }
            ),
            "route_tangent_source": state.route_tangent_source,
            "route_curvature_1pm": route_curvature_1pm,
            "route_tangent_lookahead_m": float(
                state.route_tangent_lookahead_m
            ),
            "terminal_route_plane": {
                "source": "evaluator_dense_global_plan_last_transform",
                "x_world_m": float(terminal.x),
                "y_world_m": float(terminal.y),
                "yaw_world_deg": float(terminal.yaw_deg),
                "x_forward_m": float(terminal_forward_ego),
                "y_left_m": -float(terminal_right_ego),
                "heading_error_rad": float(terminal_heading_error_rad),
                "signed_progress_m": float(
                    terminal_plane_signed_progress_m
                ),
                "lateral_error_m": float(terminal_plane_lateral_m),
                "crossed": bool(terminal_plane_signed_progress_m > 0.0),
            },
            "route_state": route_state,
            "conditions": {
                "route_deviated": bool(state.route_deviation),
                "heading_deviated": bool(heading_deviated),
                "recovery_detected": bool(recovered),
                "route_completed": bool(state.route_completed),
            },
            "boundary": {
                "runtime_only": True,
                "route_context_in_stage3_loss": False,
                "numeric_waypoints_are_model_tokens": False,
                "official_route_manager_reproduced": False,
                "input_coordinate_contract": (
                    "CARLA world XY and CARLA yaw degrees"
                ),
                "output_coordinate_contract": (
                    "ego-local x forward, y left, meters"
                ),
            },
        }

    def _navigation_action_distance(
        self,
        state: RouteState,
        *,
        progress_m: float,
        action: str,
    ) -> float:
        for point in self.route_manager.plan.points[state.cursor :]:
            if _navigation_action(point.road_option) == action:
                return max(
                    0.0,
                    float(point.cumulative_distance_m) - progress_m,
                )
        return max(0.0, float(state.distance_to_target_m))

    def report(self) -> dict[str, Any]:
        return {
            "adapter_version": GLOBAL_ROUTE_RUNTIME_ADAPTER_VERSION,
            "route_manager": "src.route.RouteManager",
            "route_id": self.route_manager.plan.route_id,
            "route_points": len(self.route_manager.plan.points),
            "route_length_m": float(self.route_manager.plan.total_distance_m),
            "updates": self._updates,
            "last_context": copy.deepcopy(self._last_context),
            "boundary": {
                "route_state_owner": "src.route.RouteManager",
                "runtime_serialization_only": True,
                "numeric_waypoints_in_model_tokens": False,
                "stage3_checkpoint_unchanged": True,
            },
        }


__all__ = [
    "GLOBAL_ROUTE_RUNTIME_ADAPTER_VERSION",
    "GlobalRouteRuntimeAdapter",
]
