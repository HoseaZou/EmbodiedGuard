from __future__ import annotations

import copy
import json
import math
from typing import Any, Mapping, Sequence


ROUTE_CONTEXT_SCHEMA_VERSION = "autovla.route_context.v1"
ROUTE_MANAGER_SCHEMA_VERSION = "autovla.route_context_manager.v1"

NAVIGATION_ACTIONS = (
    "turn_left",
    "turn_right",
    "go_straight",
    "follow_lane",
    "change_lane_left",
    "change_lane_right",
    "void",
)

NAVIGATION_COMMAND_TEXT = {
    "turn_left": "前方路口左转",
    "turn_right": "前方路口右转",
    "go_straight": "前方路口继续直行",
    "follow_lane": "保持当前车道平稳行驶",
    "change_lane_left": "条件允许时向左变道",
    "change_lane_right": "条件允许时向右变道",
    "void": "保持车道并减速，等待导航确认",
}

_ROAD_OPTION_ALIASES = {
    "LEFT": "turn_left",
    "TURN_LEFT": "turn_left",
    "RIGHT": "turn_right",
    "TURN_RIGHT": "turn_right",
    "STRAIGHT": "go_straight",
    "GO_STRAIGHT": "go_straight",
    "LANEFOLLOW": "follow_lane",
    "LANE_FOLLOW": "follow_lane",
    "FOLLOW_LANE": "follow_lane",
    "CHANGELANELEFT": "change_lane_left",
    "CHANGE_LANE_LEFT": "change_lane_left",
    "CHANGELANERIGHT": "change_lane_right",
    "CHANGE_LANE_RIGHT": "change_lane_right",
    "VOID": "void",
}


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


def _positive_float(value: Any, name: str) -> float:
    parsed = _finite_float(value, name)
    if parsed <= 0.0:
        raise ValueError(f"{name} must be positive")
    return parsed


def _nonnegative_float(value: Any, name: str) -> float:
    parsed = _finite_float(value, name)
    if parsed < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return parsed


def _positive_integer(value: Any, name: str) -> int:
    if type(value) is not int or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _nonnegative_integer(value: Any, name: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _nonempty_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


def _road_option_name(value: Any) -> str:
    candidate = getattr(value, "name", value)
    if not isinstance(candidate, str) or not candidate.strip():
        raise ValueError("road_option must have a non-empty name")
    normalized = candidate.strip().upper().replace("-", "_").replace(" ", "_")
    try:
        return _ROAD_OPTION_ALIASES[normalized]
    except KeyError as exc:
        raise ValueError(
            "road_option must map to a supported navigation action"
        ) from exc


def _xy_from_pose(value: Any, name: str) -> tuple[float, float]:
    pose = value
    if isinstance(pose, Mapping):
        if "x" in pose and "y" in pose:
            return (
                _finite_float(pose["x"], f"{name}_x"),
                _finite_float(pose["y"], f"{name}_y"),
            )
        for field in ("location", "transform", "waypoint"):
            if field in pose:
                return _xy_from_pose(pose[field], f"{name}_{field}")
    # Resolve coordinate-bearing objects before wrapper attributes. In CARLA,
    # ``Waypoint.transform`` is a Transform value, while
    # ``Transform.transform`` is a callable that mutates a Location. Treating
    # that method as another nested pose recurses into a bound method and
    # rejects valid GlobalRoutePlanner waypoints.
    if hasattr(pose, "x") and hasattr(pose, "y"):
        return (
            _finite_float(getattr(pose, "x"), f"{name}_x"),
            _finite_float(getattr(pose, "y"), f"{name}_y"),
        )
    location = getattr(pose, "location", None)
    if location is not None and not callable(location):
        return _xy_from_pose(location, f"{name}_location")
    transform = getattr(pose, "transform", None)
    if transform is not None and not callable(transform):
        return _xy_from_pose(transform, f"{name}_transform")
    waypoint = getattr(pose, "waypoint", None)
    if waypoint is not None and not callable(waypoint):
        return _xy_from_pose(waypoint, f"{name}_waypoint")
    raise ValueError(f"{name} must expose finite x and y coordinates")


def normalize_carla_global_plan(global_plan: Any) -> list[dict[str, Any]]:
    """Normalize a CARLA-style global plan without importing the CARLA API.

    Each item may be a ``(pose, RoadOption)`` pair or a mapping containing
    ``x``, ``y``, and ``road_option``. Actions are mapped to the existing
    Stage 3 command-text distribution; numeric waypoints remain runtime-only.
    """
    if not isinstance(global_plan, Sequence) or isinstance(
        global_plan, (str, bytes)
    ):
        raise ValueError("global_plan must be an array")
    normalized = []
    for index, item in enumerate(global_plan):
        if isinstance(item, Mapping):
            road_option = item.get("road_option")
            pose = item
        elif isinstance(item, Sequence) and not isinstance(item, (str, bytes)):
            if len(item) != 2:
                raise ValueError(
                    f"global_plan_{index} pair must contain pose and road_option"
                )
            pose, road_option = item
        else:
            raise ValueError(f"global_plan_{index} has an unsupported shape")
        x_value, y_value = _xy_from_pose(pose, f"global_plan_{index}")
        normalized.append(
            {
                "x": x_value,
                "y": y_value,
                "road_option": _road_option_name(road_option),
            }
        )
    if len(normalized) < 2:
        raise ValueError("global_plan must contain at least two points")
    route_length = sum(
        math.hypot(end["x"] - start["x"], end["y"] - start["y"])
        for start, end in zip(normalized[:-1], normalized[1:])
    )
    if route_length <= 1e-9:
        raise ValueError("global_plan must have positive route length")
    return normalized


def _wrap_degrees(value: float) -> float:
    wrapped = (value + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def _carla_world_to_ego_left(
    *, world_x: float, world_y: float, ego_x: float, ego_y: float, ego_yaw_deg: float
) -> tuple[float, float]:
    """Convert CARLA world XY to ego-local x-forward/y-left coordinates."""
    dx = world_x - ego_x
    dy = world_y - ego_y
    yaw = math.radians(ego_yaw_deg)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    local_forward = cosine * dx + sine * dy
    local_left = sine * dx - cosine * dy
    return local_forward, local_left


def _strict_json(value: Mapping[str, Any]) -> None:
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("route context must be strict JSON") from exc


class RouteContextManager:
    """Track an external CARLA global plan and emit runtime-only context."""

    def __init__(
        self,
        *,
        route_id: str,
        global_plan: Any,
        lookahead_distances_m: Sequence[float] = (2, 4, 6, 8, 10, 12),
        navigation_lookahead_m: float = 30.0,
        deviation_threshold_m: float = 4.0,
        recovery_threshold_m: float = 2.0,
        heading_deviation_threshold_deg: float = 90.0,
        heading_check_min_speed_mps: float = 0.5,
        completion_tolerance_m: float = 1.5,
        completion_speed_threshold_mps: float = 0.5,
        completion_hold_frames: int = 3,
        cursor_backtrack_segments: int = 2,
    ) -> None:
        self.route_id = _nonempty_string(route_id, "route_id")
        self.global_plan = normalize_carla_global_plan(global_plan)
        if not isinstance(lookahead_distances_m, Sequence) or isinstance(
            lookahead_distances_m, (str, bytes)
        ):
            raise ValueError("lookahead_distances_m must be an array")
        self.lookahead_distances_m = tuple(
            _positive_float(value, f"lookahead_distances_m_{index}")
            for index, value in enumerate(lookahead_distances_m)
        )
        if not self.lookahead_distances_m:
            raise ValueError("lookahead_distances_m must not be empty")
        if any(
            end <= start
            for start, end in zip(
                self.lookahead_distances_m[:-1], self.lookahead_distances_m[1:]
            )
        ):
            raise ValueError("lookahead_distances_m must be strictly increasing")
        self.navigation_lookahead_m = _positive_float(
            navigation_lookahead_m, "navigation_lookahead_m"
        )
        self.deviation_threshold_m = _positive_float(
            deviation_threshold_m, "deviation_threshold_m"
        )
        self.recovery_threshold_m = _nonnegative_float(
            recovery_threshold_m, "recovery_threshold_m"
        )
        if self.recovery_threshold_m >= self.deviation_threshold_m:
            raise ValueError(
                "recovery_threshold_m must be smaller than deviation_threshold_m"
            )
        self.heading_deviation_threshold_deg = _finite_float(
            heading_deviation_threshold_deg, "heading_deviation_threshold_deg"
        )
        if not 0.0 < self.heading_deviation_threshold_deg <= 180.0:
            raise ValueError("heading_deviation_threshold_deg must be in (0, 180]")
        self.heading_check_min_speed_mps = _nonnegative_float(
            heading_check_min_speed_mps, "heading_check_min_speed_mps"
        )
        self.completion_tolerance_m = _positive_float(
            completion_tolerance_m, "completion_tolerance_m"
        )
        self.completion_speed_threshold_mps = _nonnegative_float(
            completion_speed_threshold_mps, "completion_speed_threshold_mps"
        )
        self.completion_hold_frames = _positive_integer(
            completion_hold_frames, "completion_hold_frames"
        )
        self.cursor_backtrack_segments = _nonnegative_integer(
            cursor_backtrack_segments, "cursor_backtrack_segments"
        )

        self._cumulative_m = [0.0]
        for start, end in zip(self.global_plan[:-1], self.global_plan[1:]):
            self._cumulative_m.append(
                self._cumulative_m[-1]
                + math.hypot(end["x"] - start["x"], end["y"] - start["y"])
            )
        self.route_length_m = self._cumulative_m[-1]
        self._cursor_segment = 0
        self._progress_m = 0.0
        self._last_frame_id: int | None = None
        self._last_observed_monotonic_ns: int | None = None
        self._deviated = False
        self._completion_count = 0
        self._completed = False
        self._update_count = 0
        self._last_context: dict[str, Any] | None = None

    def _projection(self, ego_x: float, ego_y: float) -> dict[str, Any]:
        start_index = max(0, self._cursor_segment - self.cursor_backtrack_segments)
        best = None
        for index in range(start_index, len(self.global_plan) - 1):
            start = self.global_plan[index]
            end = self.global_plan[index + 1]
            dx = end["x"] - start["x"]
            dy = end["y"] - start["y"]
            denominator = dx * dx + dy * dy
            if denominator <= 1e-18:
                continue
            ratio = ((ego_x - start["x"]) * dx + (ego_y - start["y"]) * dy)
            ratio = max(0.0, min(1.0, ratio / denominator))
            projected_x = start["x"] + ratio * dx
            projected_y = start["y"] + ratio * dy
            distance = math.hypot(ego_x - projected_x, ego_y - projected_y)
            segment_length = math.sqrt(denominator)
            along = self._cumulative_m[index] + ratio * segment_length
            candidate = {
                "segment_index": index,
                "ratio": ratio,
                "projected_x": projected_x,
                "projected_y": projected_y,
                "distance_m": distance,
                "along_m": along,
                "heading_deg": math.degrees(math.atan2(dy, dx)),
            }
            if best is None or (distance, -along) < (
                best["distance_m"],
                -best["along_m"],
            ):
                best = candidate
        if best is None:
            raise RuntimeError("global plan has no non-zero route segment")
        return best

    def _sample_world(self, along_m: float) -> tuple[float, float]:
        target = max(0.0, min(self.route_length_m, along_m))
        for index in range(len(self.global_plan) - 1):
            if target > self._cumulative_m[index + 1] + 1e-9:
                continue
            start = self.global_plan[index]
            end = self.global_plan[index + 1]
            segment_length = self._cumulative_m[index + 1] - self._cumulative_m[index]
            if segment_length <= 1e-12:
                continue
            ratio = (target - self._cumulative_m[index]) / segment_length
            return (
                start["x"] + ratio * (end["x"] - start["x"]),
                start["y"] + ratio * (end["y"] - start["y"]),
            )
        final = self.global_plan[-1]
        return final["x"], final["y"]

    def _navigation_action(self, progress_m: float, segment_index: int) -> tuple[str, float]:
        start_index = min(segment_index + 1, len(self.global_plan) - 1)
        fallback_index = start_index
        selected_index = None
        for index in range(start_index, len(self.global_plan)):
            distance = max(0.0, self._cumulative_m[index] - progress_m)
            if distance > self.navigation_lookahead_m:
                break
            if self.global_plan[index]["road_option"] != "follow_lane":
                selected_index = index
                break
        index = fallback_index if selected_index is None else selected_index
        action = self.global_plan[index]["road_option"]
        return action, max(0.0, self._cumulative_m[index] - progress_m)

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
            observed_monotonic_ns, "observed_monotonic_ns"
        )
        if self._last_frame_id is not None and frame <= self._last_frame_id:
            raise ValueError("frame_id must be strictly increasing")
        if (
            self._last_observed_monotonic_ns is not None
            and observed_ns < self._last_observed_monotonic_ns
        ):
            raise ValueError("observed_monotonic_ns must be monotonic")
        x_value = _finite_float(ego_x, "ego_x")
        y_value = _finite_float(ego_y, "ego_y")
        yaw_value = _finite_float(ego_yaw_deg, "ego_yaw_deg")
        speed_value = _nonnegative_float(speed_mps, "speed_mps")

        projection = self._projection(x_value, y_value)
        previous_progress = self._progress_m
        self._progress_m = max(self._progress_m, projection["along_m"])
        self._cursor_segment = max(
            self._cursor_segment, projection["segment_index"]
        )
        heading_error = _wrap_degrees(projection["heading_deg"] - yaw_value)
        heading_deviation = bool(
            speed_value >= self.heading_check_min_speed_mps
            and abs(heading_error) >= self.heading_deviation_threshold_deg
        )
        deviation_condition = bool(
            projection["distance_m"] > self.deviation_threshold_m
            or heading_deviation
        )
        recovered = False
        if self._deviated:
            if (
                projection["distance_m"] <= self.recovery_threshold_m
                and not heading_deviation
            ):
                self._deviated = False
                recovered = True
        elif deviation_condition:
            self._deviated = True

        goal = self.global_plan[-1]
        goal_distance = math.hypot(x_value - goal["x"], y_value - goal["y"])
        if (
            goal_distance <= self.completion_tolerance_m
            and speed_value <= self.completion_speed_threshold_mps
        ):
            self._completion_count += 1
        else:
            self._completion_count = 0
        if self._completion_count >= self.completion_hold_frames:
            self._completed = True

        action, action_distance = self._navigation_action(
            self._progress_m, self._cursor_segment
        )
        ego_local_waypoints = []
        seen_along = set()
        for lookahead in self.lookahead_distances_m:
            target_along = min(self.route_length_m, self._progress_m + lookahead)
            rounded = round(target_along, 9)
            if rounded in seen_along:
                continue
            seen_along.add(rounded)
            world_x, world_y = self._sample_world(target_along)
            local_x, local_y = _carla_world_to_ego_left(
                world_x=world_x,
                world_y=world_y,
                ego_x=x_value,
                ego_y=y_value,
                ego_yaw_deg=yaw_value,
            )
            ego_local_waypoints.append(
                {
                    "route_distance_m": target_along,
                    "lookahead_m": max(0.0, target_along - self._progress_m),
                    "x_forward_m": local_x,
                    "y_left_m": local_y,
                }
            )

        route_state = (
            "completed"
            if self._completed
            else "recovered"
            if recovered
            else "deviated"
            if self._deviated
            else "tracking"
        )
        context = {
            "schema_version": ROUTE_CONTEXT_SCHEMA_VERSION,
            "route_id": self.route_id,
            "source_frame_id": frame,
            "observed_monotonic_ns": observed_ns,
            "route_cursor_index": self._cursor_segment,
            "route_length_m": self.route_length_m,
            "route_progress_m": self._progress_m,
            "route_progress": self._progress_m / self.route_length_m,
            "progress_delta_m": self._progress_m - previous_progress,
            "distance_to_route_m": projection["distance_m"],
            "distance_to_goal_m": goal_distance,
            "heading_error_deg": heading_error,
            "next_navigation_action": action,
            "navigation_command_text": NAVIGATION_COMMAND_TEXT[action],
            "navigation_action_distance_m": action_distance,
            "ego_local_waypoints": ego_local_waypoints,
            "route_state": route_state,
            "conditions": {
                "route_deviated": self._deviated,
                "heading_deviated": heading_deviation,
                "recovery_detected": recovered,
                "route_completed": self._completed,
            },
            "boundary": {
                "runtime_only": True,
                "route_context_in_stage3_loss": False,
                "numeric_waypoints_are_model_tokens": False,
                "official_route_manager_reproduced": False,
                "input_coordinate_contract": "CARLA world XY and CARLA yaw degrees",
                "output_coordinate_contract": "ego-local x forward, y left, meters",
            },
        }
        _strict_json(context)
        validation_errors = validate_route_context(context)
        if validation_errors:
            raise RuntimeError(
                "generated route context is invalid: "
                + ",".join(validation_errors[:10])
            )
        self._last_frame_id = frame
        self._last_observed_monotonic_ns = observed_ns
        self._update_count += 1
        self._last_context = copy.deepcopy(context)
        return copy.deepcopy(context)

    def report(self) -> dict[str, Any]:
        return {
            "schema_version": ROUTE_MANAGER_SCHEMA_VERSION,
            "route_id": self.route_id,
            "route_points": len(self.global_plan),
            "route_length_m": self.route_length_m,
            "updates": self._update_count,
            "last_context": copy.deepcopy(self._last_context),
            "configuration": {
                "lookahead_distances_m": list(self.lookahead_distances_m),
                "navigation_lookahead_m": self.navigation_lookahead_m,
                "deviation_threshold_m": self.deviation_threshold_m,
                "recovery_threshold_m": self.recovery_threshold_m,
                "heading_deviation_threshold_deg": self.heading_deviation_threshold_deg,
                "heading_check_min_speed_mps": self.heading_check_min_speed_mps,
                "completion_tolerance_m": self.completion_tolerance_m,
                "completion_speed_threshold_mps": self.completion_speed_threshold_mps,
                "completion_hold_frames": self.completion_hold_frames,
            },
            "boundary": {
                "runtime_only": True,
                "carla_imported": False,
                "official_route_manager_reproduced": False,
            },
        }


def validate_route_context(value: Any) -> list[str]:
    if not isinstance(value, Mapping):
        return ["route_context_not_object"]
    errors = []
    required_keys = {
        "schema_version",
        "route_id",
        "source_frame_id",
        "observed_monotonic_ns",
        "route_cursor_index",
        "route_length_m",
        "route_progress_m",
        "route_progress",
        "progress_delta_m",
        "distance_to_route_m",
        "distance_to_goal_m",
        "heading_error_deg",
        "next_navigation_action",
        "navigation_command_text",
        "navigation_action_distance_m",
        "ego_local_waypoints",
        "route_state",
        "conditions",
        "boundary",
    }
    allowed_keys = required_keys | {
        "signed_distance_to_route_m",
        "route_tangent_yaw_rad",
        "route_tangent_source",
        "route_curvature_1pm",
        "route_tangent_lookahead_m",
        "terminal_route_plane",
        "route_tracking_preview",
    }
    if not required_keys.issubset(value) or not set(value).issubset(
        allowed_keys
    ):
        errors.append("route_context_keys_mismatch")
    if value.get("schema_version") != ROUTE_CONTEXT_SCHEMA_VERSION:
        errors.append("route_context_schema_mismatch")
    if not isinstance(value.get("route_id"), str) or not value.get("route_id", "").strip():
        errors.append("route_id_invalid")
    for field in ("source_frame_id", "observed_monotonic_ns", "route_cursor_index"):
        if type(value.get(field)) is not int or value.get(field) < 0:
            errors.append(f"{field}_invalid")
    for field in (
        "route_length_m",
        "route_progress_m",
        "progress_delta_m",
        "distance_to_route_m",
        "distance_to_goal_m",
        "heading_error_deg",
        "navigation_action_distance_m",
    ):
        try:
            parsed = _finite_float(value.get(field), field)
        except ValueError:
            errors.append(f"{field}_invalid")
            continue
        if field != "heading_error_deg" and parsed < 0.0:
            errors.append(f"{field}_negative")
    if "signed_distance_to_route_m" in value:
        try:
            signed_distance = _finite_float(
                value.get("signed_distance_to_route_m"),
                "signed_distance_to_route_m",
            )
        except ValueError:
            errors.append("signed_distance_to_route_m_invalid")
        else:
            route_distance = value.get("distance_to_route_m")
            if (
                isinstance(route_distance, (int, float))
                and not isinstance(route_distance, bool)
                and abs(abs(signed_distance) - float(route_distance)) > 1e-3
            ):
                errors.append(
                    "signed_distance_to_route_m_magnitude_mismatch"
                )
    tangent_keys = {
        "route_tangent_yaw_rad",
        "route_tangent_source",
        "route_curvature_1pm",
        "route_tangent_lookahead_m",
    }
    present_tangent_keys = tangent_keys.intersection(value)
    if present_tangent_keys and present_tangent_keys != tangent_keys:
        errors.append("route_tangent_metadata_incomplete")
    elif present_tangent_keys == tangent_keys:
        try:
            _finite_float(
                value.get("route_tangent_yaw_rad"),
                "route_tangent_yaw_rad",
            )
        except ValueError:
            errors.append("route_tangent_yaw_rad_invalid")
        if (
            not isinstance(value.get("route_tangent_source"), str)
            or not value.get("route_tangent_source", "").strip()
        ):
            errors.append("route_tangent_source_invalid")
        for field in ("route_curvature_1pm", "route_tangent_lookahead_m"):
            try:
                parsed = _finite_float(value.get(field), field)
            except ValueError:
                errors.append(f"{field}_invalid")
                continue
            if parsed < 0.0 or (
                field == "route_tangent_lookahead_m" and parsed <= 0.0
            ):
                errors.append(f"{field}_invalid")
    preview = value.get("route_tracking_preview")
    if preview is not None:
        try:
            if (not isinstance(preview, Mapping)
                    or set(preview) != {"source", "lookahead_m", "yaw_offset_rad"}
                    or preview["source"] != "route_polyline_near_heading_v1"):
                raise ValueError("invalid preview metadata")
            lookahead = _finite_float(preview["lookahead_m"], "lookahead_m")
            offset = _finite_float(preview["yaw_offset_rad"], "yaw_offset_rad")
            if (not 0.0 <= lookahead <= 2.0 or abs(offset) > 0.12
                    or (lookahead == 0.0 and offset != 0.0)):
                errors.append("route_tracking_preview_invalid")
        except (ValueError, TypeError, KeyError):
            errors.append("route_tracking_preview_invalid")
    terminal_plane = value.get("terminal_route_plane")
    if terminal_plane is not None:
        terminal_plane_keys = {
            "source",
            "x_world_m",
            "y_world_m",
            "yaw_world_deg",
            "x_forward_m",
            "y_left_m",
            "heading_error_rad",
            "signed_progress_m",
            "lateral_error_m",
            "crossed",
        }
        if (
            not isinstance(terminal_plane, Mapping)
            or set(terminal_plane) != terminal_plane_keys
        ):
            errors.append("terminal_route_plane_invalid")
        else:
            if terminal_plane.get("source") != (
                "evaluator_dense_global_plan_last_transform"
            ):
                errors.append("terminal_route_plane_source_invalid")
            for field in terminal_plane_keys - {"source", "crossed"}:
                try:
                    _finite_float(
                        terminal_plane.get(field),
                        f"terminal_route_plane.{field}",
                    )
                except ValueError:
                    errors.append(f"terminal_route_plane_{field}_invalid")
            if type(terminal_plane.get("crossed")) is not bool:
                errors.append("terminal_route_plane_crossed_invalid")
    try:
        progress = _finite_float(value.get("route_progress"), "route_progress")
    except ValueError:
        errors.append("route_progress_invalid")
    else:
        if not 0.0 <= progress <= 1.0:
            errors.append("route_progress_out_of_range")
    action = value.get("next_navigation_action")
    if action not in NAVIGATION_ACTIONS:
        errors.append("next_navigation_action_invalid")
    if value.get("navigation_command_text") != NAVIGATION_COMMAND_TEXT.get(action):
        errors.append("navigation_command_text_mismatch")
    if value.get("route_state") not in {
        "tracking",
        "deviated",
        "recovered",
        "completed",
    }:
        errors.append("route_state_invalid")
    waypoints = value.get("ego_local_waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        errors.append("ego_local_waypoints_invalid")
    else:
        expected_waypoint_keys = {
            "route_distance_m",
            "lookahead_m",
            "x_forward_m",
            "y_left_m",
        }
        previous_distance = -1.0
        for index, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, Mapping) or set(waypoint) != expected_waypoint_keys:
                errors.append(f"ego_local_waypoint_{index}_invalid")
                continue
            for field in expected_waypoint_keys:
                try:
                    parsed = _finite_float(
                        waypoint.get(field), f"ego_local_waypoint_{index}_{field}"
                    )
                except ValueError:
                    errors.append(f"ego_local_waypoint_{index}_{field}_invalid")
                    continue
                if field in {"route_distance_m", "lookahead_m"} and parsed < 0.0:
                    errors.append(f"ego_local_waypoint_{index}_{field}_negative")
            route_distance = waypoint.get("route_distance_m")
            if isinstance(route_distance, (int, float)) and not isinstance(
                route_distance, bool
            ):
                if route_distance <= previous_distance:
                    errors.append("ego_local_waypoint_distance_not_increasing")
                previous_distance = float(route_distance)
    conditions = value.get("conditions")
    condition_keys = {
        "route_deviated",
        "heading_deviated",
        "recovery_detected",
        "route_completed",
    }
    if not isinstance(conditions, Mapping) or set(conditions) != condition_keys:
        errors.append("route_conditions_invalid")
    elif any(type(conditions.get(field)) is not bool for field in condition_keys):
        errors.append("route_condition_not_boolean")
    boundary = value.get("boundary")
    expected_boundary = {
        "runtime_only": True,
        "route_context_in_stage3_loss": False,
        "numeric_waypoints_are_model_tokens": False,
        "official_route_manager_reproduced": False,
        "input_coordinate_contract": "CARLA world XY and CARLA yaw degrees",
        "output_coordinate_contract": "ego-local x forward, y left, meters",
    }
    if not isinstance(boundary, Mapping) or dict(boundary) != expected_boundary:
        errors.append("route_context_boundary_invalid")
    try:
        _strict_json(value)
    except ValueError:
        errors.append("route_context_not_strict_json")
    return sorted(set(errors))


__all__ = [
    "NAVIGATION_ACTIONS",
    "NAVIGATION_COMMAND_TEXT",
    "ROUTE_CONTEXT_SCHEMA_VERSION",
    "ROUTE_MANAGER_SCHEMA_VERSION",
    "RouteContextManager",
    "normalize_carla_global_plan",
    "validate_route_context",
]
