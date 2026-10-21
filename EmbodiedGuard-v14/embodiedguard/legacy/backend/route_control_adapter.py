from __future__ import annotations

import copy
import math
from dataclasses import asdict, dataclass
from typing import Any, Mapping


ROUTE_CONTROL_ADAPTER_SCHEMA_VERSION = "autovla.route_control_adapter.v1"


def _finite_float(value: Any, name: str) -> float:
    if type(value) is bool:
        raise ValueError(f"{name} must be numeric, not boolean")
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


def _clamp(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


def _wrap_radians(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _continuous_weight(
    magnitude: float,
    *,
    deadband: float,
    full_scale: float,
) -> float:
    if full_scale <= deadband:
        raise ValueError("full-scale correction must exceed its deadband")
    return _clamp(
        (abs(float(magnitude)) - float(deadband))
        / (float(full_scale) - float(deadband)),
        0.0,
        1.0,
    )


def _weighted_route_tangent(
    waypoints: list[tuple[float, float]],
    *,
    lookahead_m: float,
) -> tuple[float, dict[str, Any]]:
    """Estimate route heading without folding cross-track error into it."""
    if not waypoints:
        raise ValueError("route tangent requires at least one waypoint")
    horizon = _positive_float(lookahead_m, "route_tangent_lookahead_m")
    weighted_forward = 0.0
    weighted_left = 0.0
    total_weight = 0.0
    used_segments: list[dict[str, float | int]] = []
    for index, (first, second) in enumerate(zip(waypoints, waypoints[1:])):
        delta_forward = second[0] - first[0]
        delta_left = second[1] - first[1]
        segment_length = math.hypot(delta_forward, delta_left)
        if segment_length <= 1e-6:
            continue
        midpoint_distance = math.hypot(
            0.5 * (first[0] + second[0]),
            0.5 * (first[1] + second[1]),
        )
        weight = _clamp(1.0 - midpoint_distance / horizon, 0.0, 1.0)
        if weight <= 1e-9:
            continue
        unit_forward = delta_forward / segment_length
        unit_left = delta_left / segment_length
        weighted_forward += weight * unit_forward
        weighted_left += weight * unit_left
        total_weight += weight
        used_segments.append(
            {
                "index": index,
                "weight": weight,
                "midpoint_distance_m": midpoint_distance,
                "unit_forward": unit_forward,
                "unit_left": unit_left,
            }
        )

    if total_weight <= 1e-9:
        first_forward, first_left = waypoints[0]
        return math.atan2(first_left, max(0.1, first_forward)), {
            "reliable": False,
            "method": "first_waypoint_bearing_fallback",
            "lookahead_m": horizon,
            "segments": [],
        }
    return math.atan2(weighted_left, weighted_forward), {
        "reliable": True,
        "method": "triangular_weighted_unit_tangents",
        "lookahead_m": horizon,
        "segments": used_segments,
        "weighted_forward": weighted_forward / total_weight,
        "weighted_left": weighted_left / total_weight,
    }


@dataclass(frozen=True)
class RouteControlAdapterConfig:
    """Bounded route feedback applied after learned trajectory generation."""

    correction_deadband_m: float = 0.25
    full_correction_distance_m: float = 1.0
    max_route_lateral_target_m: float = 2.0
    max_route_yaw_target_rad: float = 0.35
    goal_slowdown_distance_m: float = 8.0
    # Start the stationary target at the formal acceptance boundary. Braking
    # then carries the vehicle into, rather than through, the verified goal.
    goal_stop_distance_m: float = 1.5
    goal_deceleration_mps2: float = 1.0
    terminal_capture_remaining_route_m: float = 1.0
    terminal_capture_speed_limit_mps: float = 0.45
    correction_speed_limit_start_weight: float = 0.75
    full_correction_speed_limit_mps: float = 1.0
    maneuver_full_correction_speed_limit_mps: float = 2.0
    recovery_speed_limit_mps: float = 1.0
    recovery_min_lateral_m: float = 0.25
    turn_detection_distance_m: float = 12.0
    turn_yaw_threshold_rad: float = 0.35
    # Local CARLA sweeps found 5.95 m/s to be the highest repeatable
    # body-envelope-safe turn speed, while 6.0 m/s crossed the envelope on
    # the most demanding route.  Keep 0.20 m/s of operating margin.
    turn_speed_limit_mps: float = 5.75
    route_tangent_lookahead_m: float = 8.0
    near_curve_preview_enabled: bool = True
    heading_correction_deadband_rad: float = 0.08
    full_heading_correction_rad: float = 0.35
    lateral_disagreement_deadband_m: float = 0.20
    full_lateral_disagreement_m: float = 0.75
    yaw_disagreement_deadband_rad: float = 0.10
    full_yaw_disagreement_rad: float = 0.35
    cross_track_yaw_start_distance_m: float = 0.75
    full_cross_track_yaw_distance_m: float = 1.50
    max_cross_track_yaw_correction_rad: float = 0.20
    # A maneuver needs one stable geometric owner.  Without this latch the
    # instantaneous model/route disagreement weights can hand steering back
    # and forth at every Stage-3 replan, exactly where visual lane markings
    # are weakest.  These values are per 20 Hz control cycle; attack is quick
    # enough to own the intersection before entry, while the hold/release
    # carries the same route reference through the exit.
    maneuver_authority_attack_per_cycle: float = 0.25
    maneuver_authority_release_per_cycle: float = 0.05
    maneuver_authority_release_hold_cycles: int = 12

    def validate(self) -> None:
        if type(self.near_curve_preview_enabled) is not bool:
            raise ValueError("near_curve_preview_enabled must be boolean")
        deadband = _nonnegative_float(
            self.correction_deadband_m,
            "correction_deadband_m",
        )
        full_distance = _positive_float(
            self.full_correction_distance_m,
            "full_correction_distance_m",
        )
        if full_distance <= deadband:
            raise ValueError(
                "full_correction_distance_m must exceed correction_deadband_m"
            )
        _positive_float(
            self.max_route_lateral_target_m,
            "max_route_lateral_target_m",
        )
        _positive_float(
            self.max_route_yaw_target_rad,
            "max_route_yaw_target_rad",
        )
        slowdown_distance = _positive_float(
            self.goal_slowdown_distance_m,
            "goal_slowdown_distance_m",
        )
        stop_distance = _nonnegative_float(
            self.goal_stop_distance_m,
            "goal_stop_distance_m",
        )
        if slowdown_distance <= stop_distance:
            raise ValueError(
                "goal_slowdown_distance_m must exceed goal_stop_distance_m"
            )
        _positive_float(
            self.goal_deceleration_mps2,
            "goal_deceleration_mps2",
        )
        _positive_float(
            self.terminal_capture_remaining_route_m,
            "terminal_capture_remaining_route_m",
        )
        _positive_float(
            self.terminal_capture_speed_limit_mps,
            "terminal_capture_speed_limit_mps",
        )
        correction_speed_limit_start_weight = _finite_float(
            self.correction_speed_limit_start_weight,
            "correction_speed_limit_start_weight",
        )
        if not 0.0 <= correction_speed_limit_start_weight < 1.0:
            raise ValueError(
                "correction_speed_limit_start_weight must be in [0, 1)"
            )
        _positive_float(
            self.full_correction_speed_limit_mps,
            "full_correction_speed_limit_mps",
        )
        _positive_float(
            self.maneuver_full_correction_speed_limit_mps,
            "maneuver_full_correction_speed_limit_mps",
        )
        _positive_float(
            self.recovery_speed_limit_mps,
            "recovery_speed_limit_mps",
        )
        _positive_float(
            self.recovery_min_lateral_m,
            "recovery_min_lateral_m",
        )
        _positive_float(
            self.turn_detection_distance_m,
            "turn_detection_distance_m",
        )
        _positive_float(
            self.turn_yaw_threshold_rad,
            "turn_yaw_threshold_rad",
        )
        _positive_float(
            self.turn_speed_limit_mps,
            "turn_speed_limit_mps",
        )
        _positive_float(
            self.route_tangent_lookahead_m,
            "route_tangent_lookahead_m",
        )
        heading_deadband = _nonnegative_float(
            self.heading_correction_deadband_rad,
            "heading_correction_deadband_rad",
        )
        heading_full = _positive_float(
            self.full_heading_correction_rad,
            "full_heading_correction_rad",
        )
        if heading_full <= heading_deadband:
            raise ValueError(
                "full_heading_correction_rad must exceed "
                "heading_correction_deadband_rad"
            )
        lateral_disagreement_deadband = _nonnegative_float(
            self.lateral_disagreement_deadband_m,
            "lateral_disagreement_deadband_m",
        )
        lateral_disagreement_full = _positive_float(
            self.full_lateral_disagreement_m,
            "full_lateral_disagreement_m",
        )
        if lateral_disagreement_full <= lateral_disagreement_deadband:
            raise ValueError(
                "full_lateral_disagreement_m must exceed "
                "lateral_disagreement_deadband_m"
            )
        disagreement_deadband = _nonnegative_float(
            self.yaw_disagreement_deadband_rad,
            "yaw_disagreement_deadband_rad",
        )
        disagreement_full = _positive_float(
            self.full_yaw_disagreement_rad,
            "full_yaw_disagreement_rad",
        )
        if disagreement_full <= disagreement_deadband:
            raise ValueError(
                "full_yaw_disagreement_rad must exceed "
                "yaw_disagreement_deadband_rad"
            )
        cross_track_yaw_start = _nonnegative_float(
            self.cross_track_yaw_start_distance_m,
            "cross_track_yaw_start_distance_m",
        )
        full_cross_track_yaw = _positive_float(
            self.full_cross_track_yaw_distance_m,
            "full_cross_track_yaw_distance_m",
        )
        if full_cross_track_yaw <= cross_track_yaw_start:
            raise ValueError(
                "full_cross_track_yaw_distance_m must exceed "
                "cross_track_yaw_start_distance_m"
            )
        max_cross_track_yaw = _positive_float(
            self.max_cross_track_yaw_correction_rad,
            "max_cross_track_yaw_correction_rad",
        )
        if max_cross_track_yaw > float(self.max_route_yaw_target_rad):
            raise ValueError(
                "max_cross_track_yaw_correction_rad must not exceed "
                "max_route_yaw_target_rad"
            )
        for name in (
            "maneuver_authority_attack_per_cycle",
            "maneuver_authority_release_per_cycle",
        ):
            value = _positive_float(getattr(self, name), name)
            if value > 1.0:
                raise ValueError(f"{name} must not exceed 1")
        if (
            type(self.maneuver_authority_release_hold_cycles) is not int
            or self.maneuver_authority_release_hold_cycles < 0
        ):
            raise ValueError(
                "maneuver_authority_release_hold_cycles must be a "
                "non-negative integer"
            )


class RouteControlAdapter:
    """Blend model lateral/yaw targets with bounded RouteManager feedback.

    Model speed remains an upper bound. Model targets pass through only while
    they agree with the current route corridor. Cross-track error or a large
    model/route disagreement smoothly increases route authority without adding
    a speed penalty for disagreement alone. The stateful PID and its CARLA sign
    conversion remain the only actuator boundary.
    """

    def __init__(self, config: RouteControlAdapterConfig | None = None) -> None:
        self.config = config or RouteControlAdapterConfig()
        if not isinstance(self.config, RouteControlAdapterConfig):
            raise ValueError("config must be RouteControlAdapterConfig")
        self.config.validate()
        self._adaptations = 0
        self._activated_adaptations = 0
        self._last_evidence: dict[str, Any] | None = None
        self._maneuver_route_authority = 0.0
        self._maneuver_release_cycles_remaining = 0

    def _update_maneuver_route_authority(
        self,
        *,
        requested: bool,
        ordinary_route_follow: bool,
    ) -> tuple[float, int]:
        """Latch route geometry across a turn/intersection handoff.

        The latch is deliberately unavailable to ASR, bypass and other
        explicit maneuvers: their callers pass ordinary_route_follow=False.
        This prevents stale route authority leaking into a higher-priority
        command while keeping the normal route reference continuous through
        short waypoint/action reclassification at an intersection exit.
        """
        if not ordinary_route_follow:
            self._maneuver_route_authority = 0.0
            self._maneuver_release_cycles_remaining = 0
        elif requested:
            self._maneuver_release_cycles_remaining = int(
                self.config.maneuver_authority_release_hold_cycles
            )
            self._maneuver_route_authority = min(
                1.0,
                self._maneuver_route_authority
                + float(self.config.maneuver_authority_attack_per_cycle),
            )
        elif self._maneuver_release_cycles_remaining > 0:
            self._maneuver_release_cycles_remaining -= 1
        else:
            self._maneuver_route_authority = max(
                0.0,
                self._maneuver_route_authority
                - float(self.config.maneuver_authority_release_per_cycle),
            )
        return (
            self._maneuver_route_authority,
            self._maneuver_release_cycles_remaining,
        )

    def adapt(
        self,
        *,
        model_control_hint: Mapping[str, Any],
        route_context: Mapping[str, Any],
        ordinary_route_follow: bool = False,
    ) -> dict[str, Any]:
        if not isinstance(model_control_hint, Mapping):
            raise ValueError("model_control_hint must be an object")
        if not isinstance(route_context, Mapping):
            raise ValueError("route_context must be an object")

        model_hint = {
            "target_speed_mps": _nonnegative_float(
                model_control_hint.get("target_speed_mps"),
                "target_speed_mps",
            ),
            "target_lateral": _finite_float(
                model_control_hint.get("target_lateral"),
                "target_lateral",
            ),
            "target_yaw": _finite_float(
                model_control_hint.get("target_yaw"),
                "target_yaw",
            ),
        }
        yaw_horizon_value = model_control_hint.get("target_yaw_horizon_sec")
        if yaw_horizon_value is not None:
            model_hint["target_yaw_horizon_sec"] = _positive_float(
                yaw_horizon_value,
                "target_yaw_horizon_sec",
            )
        route_distance = _nonnegative_float(
            route_context.get("distance_to_route_m"),
            "distance_to_route_m",
        )
        signed_route_distance_value = route_context.get(
            "signed_distance_to_route_m"
        )
        signed_route_distance = (
            None
            if signed_route_distance_value is None
            else _finite_float(
                signed_route_distance_value,
                "signed_distance_to_route_m",
            )
        )
        if (
            signed_route_distance is not None
            and abs(abs(signed_route_distance) - route_distance) > 1e-3
        ):
            raise ValueError(
                "signed_distance_to_route_m magnitude must match "
                "distance_to_route_m"
            )
        goal_distance = _nonnegative_float(
            route_context.get("distance_to_goal_m"),
            "distance_to_goal_m",
        )
        route_length = _positive_float(
            route_context.get("route_length_m"),
            "route_length_m",
        )
        route_progress = _nonnegative_float(
            route_context.get("route_progress_m"),
            "route_progress_m",
        )
        if route_progress > route_length + 1e-6:
            raise ValueError("route_progress_m must not exceed route_length_m")
        remaining_route = max(0.0, route_length - route_progress)
        waypoints = route_context.get("ego_local_waypoints")
        if not isinstance(waypoints, list) or not waypoints:
            raise ValueError("route_context must contain ego_local_waypoints")
        parsed_waypoints: list[tuple[float, float]] = []
        for index, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, Mapping):
                raise ValueError(f"ego-local waypoint {index} must be an object")
            parsed_waypoints.append(
                (
                    _finite_float(
                        waypoint.get("x_forward_m"),
                        f"route_waypoint_{index}.x_forward_m",
                    ),
                    _finite_float(
                        waypoint.get("y_left_m"),
                        f"route_waypoint_{index}.y_left_m",
                    ),
                )
            )
        route_forward, route_left = parsed_waypoints[0]
        conditions = route_context.get("conditions")
        route_deviated = bool(
            isinstance(conditions, Mapping)
            and (
                conditions.get("route_deviated") is True
                or conditions.get("heading_deviated") is True
            )
        )
        terminal_capture = bool(
            remaining_route
            <= float(self.config.terminal_capture_remaining_route_m)
            and goal_distance <= float(self.config.goal_slowdown_distance_m)
        )
        recovery_capture = bool(
            route_deviated
            and route_forward <= 0.0
            and abs(route_left) >= float(self.config.recovery_min_lateral_m)
        )
        if route_forward <= 0.0 and not (terminal_capture or recovery_capture):
            raise ValueError("first ego-local route waypoint must be forward")
        bearing_forward = (
            max(0.1, route_forward)
            if terminal_capture or recovery_capture
            else route_forward
        )

        deadband = float(self.config.correction_deadband_m)
        full_distance = float(self.config.full_correction_distance_m)
        cross_track_weight = _continuous_weight(
            route_distance,
            deadband=deadband,
            full_scale=full_distance,
        )

        # The first preview waypoint contains both cross-track and heading
        # geometry. Prefer the signed projection for lateral control so a pure
        # heading error does not also request a large lateral displacement.
        correction_left = (
            route_left
            if signed_route_distance is None
            else signed_route_distance
        )
        route_lateral_target = _clamp(
            correction_left,
            -float(self.config.max_route_lateral_target_m),
            float(self.config.max_route_lateral_target_m),
        )
        tangent_end_index = min(3, len(parsed_waypoints) - 1)
        tangent_forward = (
            parsed_waypoints[tangent_end_index][0] - parsed_waypoints[0][0]
        )
        tangent_left = (
            parsed_waypoints[tangent_end_index][1] - parsed_waypoints[0][1]
        )
        route_manager_tangent = route_context.get("route_tangent_yaw_rad")
        if route_manager_tangent is not None:
            route_tangent_yaw = _wrap_radians(
                _finite_float(
                    route_manager_tangent,
                    "route_tangent_yaw_rad",
                )
            )
            tangent_lookahead = _positive_float(
                route_context.get(
                    "route_tangent_lookahead_m",
                    self.config.route_tangent_lookahead_m,
                ),
                "route_tangent_lookahead_m",
            )
            tangent_evidence = {
                "reliable": True,
                "method": "route_manager_tangent",
                "source": route_context.get(
                    "route_tangent_source",
                    "route_manager",
                ),
                "lookahead_m": tangent_lookahead,
                "segments": [],
            }
        else:
            route_tangent_yaw, tangent_evidence = _weighted_route_tangent(
                parsed_waypoints,
                lookahead_m=float(self.config.route_tangent_lookahead_m),
            )
        tangent_reliable = bool(tangent_evidence["reliable"])
        preview = route_context.get("route_tracking_preview")
        preview_offset = 0.0
        preview_allowed = bool(
            self.config.near_curve_preview_enabled
            and ordinary_route_follow is True
            and route_manager_tangent is not None
            and signed_route_distance is not None
            and not route_deviated and not terminal_capture and not recovery_capture
            and goal_distance > float(self.config.goal_slowdown_distance_m)
        )
        if preview is not None:
            if (not isinstance(preview, Mapping)
                    or set(preview) != {"source", "lookahead_m", "yaw_offset_rad"}
                    or preview["source"] != "route_polyline_near_heading_v1"):
                raise ValueError("route_tracking_preview invalid source or fields")
            preview_horizon = _nonnegative_float(preview["lookahead_m"], "preview lookahead")
            candidate_offset = _finite_float(preview["yaw_offset_rad"], "preview yaw offset")
            if (preview_horizon > 2.0 or abs(candidate_offset) > 0.12
                    or (preview_horizon == 0.0 and candidate_offset != 0.0)):
                raise ValueError("route_tracking_preview exceeds bounded geometry")
            if preview_allowed:
                preview_offset = candidate_offset
        tangent_evidence["current_tangent_yaw_rad"] = route_tangent_yaw
        tangent_evidence["near_preview_applied"] = abs(preview_offset) > 1e-12
        tangent_evidence["near_preview_yaw_offset_rad"] = preview_offset
        route_tangent_yaw = _wrap_radians(route_tangent_yaw + preview_offset)
        cross_track_bearing_yaw = math.atan2(
            correction_left,
            max(0.5, bearing_forward),
        )
        cross_track_yaw_weight = (
            _continuous_weight(
                route_distance,
                deadband=float(self.config.cross_track_yaw_start_distance_m),
                full_scale=float(self.config.full_cross_track_yaw_distance_m),
            )
            if signed_route_distance is not None and tangent_reliable
            else 0.0
        )
        cross_track_yaw_correction = cross_track_yaw_weight * _clamp(
            cross_track_bearing_yaw,
            -float(self.config.max_cross_track_yaw_correction_rad),
            float(self.config.max_cross_track_yaw_correction_rad),
        )
        # The lateral PID remains the primary centerline channel. A bounded
        # Stanley-like yaw term is added only for large, signed cross-track
        # error so a vehicle parallel to a curved route can capture it again.
        # It is exactly zero near the centerline, preventing the old sign-flip
        # feedback from returning.
        route_tangent_yaw_target = _clamp(
            route_tangent_yaw,
            -float(self.config.max_route_yaw_target_rad),
            float(self.config.max_route_yaw_target_rad),
        )
        route_yaw_target = _clamp(
            route_tangent_yaw_target + cross_track_yaw_correction,
            -float(self.config.max_route_yaw_target_rad),
            float(self.config.max_route_yaw_target_rad),
        )
        lateral_disagreement = (
            model_hint["target_lateral"] - route_lateral_target
        )
        lateral_disagreement_weight = (
            _continuous_weight(
                lateral_disagreement,
                deadband=float(self.config.lateral_disagreement_deadband_m),
                full_scale=float(self.config.full_lateral_disagreement_m),
            )
            if signed_route_distance is not None
            else 0.0
        )
        yaw_disagreement = _wrap_radians(
            model_hint["target_yaw"] - route_tangent_yaw_target
        )
        heading_demand_weight = (
            _continuous_weight(
                route_tangent_yaw_target,
                deadband=float(self.config.heading_correction_deadband_rad),
                full_scale=float(self.config.full_heading_correction_rad),
            )
            if tangent_reliable
            else 0.0
        )
        yaw_disagreement_weight = (
            _continuous_weight(
                yaw_disagreement,
                deadband=float(self.config.yaw_disagreement_deadband_rad),
                full_scale=float(self.config.full_yaw_disagreement_rad),
            )
            if tangent_reliable
            else 0.0
        )
        # On an explicitly authorized ordinary straight, hold both channels
        # to the same geometric reference. Model disagreement must not switch
        # steering authority back and forth as a new plan arrives. Taper by
        # geometry (not model output) when approaching a curve or the goal.
        straight_authority = 0.0
        navigation_authority = 0.0
        navigation_action = route_context.get("next_navigation_action")
        navigation_action_distance = route_context.get(
            "navigation_action_distance_m"
        )
        if navigation_action_distance is not None:
            navigation_action_distance = _nonnegative_float(
                navigation_action_distance,
                "navigation_action_distance_m",
            )
        if navigation_action == "follow_lane":
            navigation_authority = 1.0
        elif ordinary_route_follow is True and navigation_action in {
            "turn_left", "turn_right", "go_straight"
        } and route_context.get("navigation_action_distance_m") is not None:
            # This field describes the NEXT maneuver, not the current one.
            # A left turn 30 metres away is still ordinary lane following now.
            distance_to_action = _nonnegative_float(
                route_context["navigation_action_distance_m"], "navigation_action_distance_m"
            )
            navigation_authority = _clamp(
                (distance_to_action - float(self.config.turn_detection_distance_m)) / 8.0,
                0.0, 1.0,
            )
        if (
            ordinary_route_follow is True
            and signed_route_distance is not None
            and tangent_reliable
            and navigation_authority > 0
            and not route_deviated
            and not terminal_capture
            and not recovery_capture
            and isinstance(conditions, Mapping)
            and not any(conditions.values())
            and route_context.get("route_curvature_1pm") is not None
        ):
            curvature = abs(_finite_float(
                route_context["route_curvature_1pm"], "route_curvature_1pm"
            ))
            straight_authority = min(
                navigation_authority,
                _clamp((0.01 - curvature) / 0.005, 0.0, 1.0),
                _clamp((min(goal_distance, remaining_route) - 8.0) / 4.0, 0.0, 1.0),
                _clamp((0.35 - abs(route_tangent_yaw)) / 0.15, 0.0, 1.0),
            )

        # A route-following turn or intersection must not lose its geometric
        # owner merely because lane paint disappears or the next 2 Hz model
        # plan disagrees.  Use the RouteManager's monotonic projection and
        # finite-horizon tangent as the reference, then retain it briefly
        # across action/waypoint reclassification at the exit.
        junction_condition = bool(
            isinstance(conditions, Mapping)
            and conditions.get("junction") is True
        )
        explicit_maneuver_ahead = bool(
            navigation_action
            in {"turn_left", "turn_right", "go_straight"}
            and navigation_action_distance is not None
            and navigation_action_distance
            <= float(self.config.turn_detection_distance_m)
        )
        curvature_maneuver = bool(
            route_context.get("route_curvature_1pm") is not None
            and abs(
                _finite_float(
                    route_context["route_curvature_1pm"],
                    "route_curvature_1pm",
                )
            )
            >= 0.01
        )
        heading_maneuver = bool(
            abs(route_tangent_yaw)
            >= float(self.config.heading_correction_deadband_rad)
        )
        maneuver_geometry_requested = bool(
            ordinary_route_follow is True
            and signed_route_distance is not None
            and tangent_reliable
            and not route_deviated
            and not terminal_capture
            and not recovery_capture
            and (
                junction_condition
                or explicit_maneuver_ahead
                or curvature_maneuver
                or heading_maneuver
            )
        )
        maneuver_route_authority, maneuver_release_cycles = (
            self._update_maneuver_route_authority(
                requested=maneuver_geometry_requested,
                ordinary_route_follow=ordinary_route_follow is True,
            )
        )
        lateral_blend_weight = max(
            cross_track_weight,
            lateral_disagreement_weight,
            maneuver_route_authority,
        )
        yaw_blend_weight = max(
            heading_demand_weight,
            yaw_disagreement_weight,
            maneuver_route_authority,
        )
        lateral_blend_weight += straight_authority * (1.0 - lateral_blend_weight)
        yaw_blend_weight += straight_authority * (1.0 - yaw_blend_weight)
        if route_deviated:
            lateral_blend_weight = 1.0
            yaw_blend_weight = 1.0
        blend_weight = max(lateral_blend_weight, yaw_blend_weight)

        # Steering correction and speed reduction have different causes. A bad
        # model yaw on a straight route should be corrected, but it must not by
        # itself force the old 1 m/s route-recovery cap.
        correction_speed_weight = cross_track_weight
        if route_deviated:
            correction_speed_weight = 1.0
        next_navigation_action = navigation_action
        turn_action_ahead = bool(
            next_navigation_action in {"turn_left", "turn_right"}
            and navigation_action_distance is not None
            and navigation_action_distance
            <= float(self.config.turn_detection_distance_m)
        )
        turn_geometry_ahead = bool(
            tangent_reliable
            and abs(route_tangent_yaw)
            >= float(self.config.turn_yaw_threshold_rad)
            and math.hypot(route_forward, route_left)
            <= float(self.config.turn_detection_distance_m)
        )
        turn_ahead = turn_action_ahead or turn_geometry_ahead
        goal_speed_limit = model_hint["target_speed_mps"]
        if goal_distance < float(self.config.goal_slowdown_distance_m):
            remaining_braking_distance = max(
                0.0,
                goal_distance - float(self.config.goal_stop_distance_m),
            )
            goal_speed_limit = math.sqrt(
                2.0
                * float(self.config.goal_deceleration_mps2)
                * remaining_braking_distance
            )
        effective_speed = min(
            model_hint["target_speed_mps"],
            goal_speed_limit,
        )
        correction_speed_limit = model_hint["target_speed_mps"]
        correction_speed_limited = False
        curvature_measurement = route_context.get("route_curvature_1pm")
        correction_curvature = (
            None if curvature_measurement is None else
            abs(_finite_float(curvature_measurement, "route_curvature_1pm"))
        )
        # Raising a correction cap needs bounded geometry, not just a latched
        # route owner.  Missing geometry falls back to the original 1 m/s cap.
        maneuver_geometry_safe = bool(
            correction_curvature is not None
            and correction_curvature <= 0.12
            and tangent_reliable
            and signed_route_distance is not None
            and route_distance <= 1.0
            and abs(route_tangent_yaw) <= 0.35
        )
        maneuver_correction_relaxation = bool(
            ordinary_route_follow is True
            and maneuver_route_authority > 0.0
            and maneuver_geometry_safe
            and not route_deviated
            and not terminal_capture
            and not recovery_capture
        )
        full_correction_speed_limit = float(
            max(
                self.config.full_correction_speed_limit_mps,
                self.config.maneuver_full_correction_speed_limit_mps,
            )
            if maneuver_correction_relaxation
            else self.config.full_correction_speed_limit_mps
        )
        speed_limit_start = float(
            self.config.correction_speed_limit_start_weight
        )
        if correction_speed_weight > speed_limit_start:
            speed_limit_weight = _clamp(
                (correction_speed_weight - speed_limit_start)
                / (1.0 - speed_limit_start),
                0.0,
                1.0,
            )
            correction_speed_limit = (
                (1.0 - speed_limit_weight) * model_hint["target_speed_mps"]
                + speed_limit_weight
                * min(
                    model_hint["target_speed_mps"],
                    full_correction_speed_limit,
                )
            )
            correction_speed_limited = bool(
                effective_speed > correction_speed_limit
            )
            effective_speed = min(effective_speed, correction_speed_limit)
        maneuver_curvature_speed_limit = None
        if maneuver_correction_relaxation:
            maneuver_curvature_speed_limit = math.sqrt(
                1.25 / max(correction_curvature, 1e-6)
            )
            effective_speed = min(effective_speed, maneuver_curvature_speed_limit)
        terminal_capture_speed_limited = bool(
            terminal_capture
            and effective_speed
            > float(self.config.terminal_capture_speed_limit_mps)
        )
        if terminal_capture:
            effective_speed = min(
                effective_speed,
                float(self.config.terminal_capture_speed_limit_mps),
            )
        if recovery_capture:
            effective_speed = min(
                effective_speed,
                float(self.config.recovery_speed_limit_mps),
            )
        turn_speed_limited = bool(
            turn_ahead
            and effective_speed > float(self.config.turn_speed_limit_mps)
        )
        if turn_ahead:
            effective_speed = min(
                effective_speed,
                float(self.config.turn_speed_limit_mps),
            )
        effective_yaw = _clamp(
            (
                (1.0 - yaw_blend_weight) * model_hint["target_yaw"]
                + yaw_blend_weight * route_tangent_yaw_target
            )
            + cross_track_yaw_correction,
            -float(self.config.max_route_yaw_target_rad),
            float(self.config.max_route_yaw_target_rad),
        )
        effective_hint = {
            "target_speed_mps": effective_speed,
            "target_lateral": (
                (1.0 - lateral_blend_weight) * model_hint["target_lateral"]
                + lateral_blend_weight * route_lateral_target
            ),
            "target_yaw": effective_yaw,
        }
        if "target_yaw_horizon_sec" in model_hint:
            effective_hint["target_yaw_horizon_sec"] = model_hint[
                "target_yaw_horizon_sec"
            ]
        evidence = {
            "schema_version": ROUTE_CONTROL_ADAPTER_SCHEMA_VERSION,
            "straight_route_authority": straight_authority,
            "straight_navigation_authority": navigation_authority,
            "ordinary_route_follow_authorized": ordinary_route_follow is True,
            "maneuver_geometry_requested": maneuver_geometry_requested,
            "maneuver_route_authority": maneuver_route_authority,
            "maneuver_release_cycles_remaining": maneuver_release_cycles,
            "maneuver_geometry_reasons": [name for name, active in (
                ("junction_condition", junction_condition),
                ("explicit_navigation_action", explicit_maneuver_ahead),
                ("route_curvature", curvature_maneuver),
                ("route_heading", heading_maneuver),
            ) if active],
            "activated": blend_weight > 0.0,
            "blend_weight": blend_weight,
            "lateral_blend_weight": lateral_blend_weight,
            "yaw_blend_weight": yaw_blend_weight,
            "cross_track_weight": cross_track_weight,
            "lateral_disagreement_weight": lateral_disagreement_weight,
            "lateral_disagreement_m": lateral_disagreement,
            "heading_demand_weight": heading_demand_weight,
            "yaw_disagreement_weight": yaw_disagreement_weight,
            "correction_speed_weight": correction_speed_weight,
            "yaw_disagreement_rad": yaw_disagreement,
            "distance_to_route_m": route_distance,
            "signed_distance_to_route_m": signed_route_distance,
            "distance_to_goal_m": goal_distance,
            "remaining_route_m": remaining_route,
            "terminal_capture": terminal_capture,
            "terminal_capture_speed_limited": terminal_capture_speed_limited,
            "recovery_capture": recovery_capture,
            "turn_speed_limited": turn_speed_limited,
            "turn_ahead": turn_ahead,
            "turn_action_ahead": turn_action_ahead,
            "turn_geometry_ahead": turn_geometry_ahead,
            "next_navigation_action": next_navigation_action,
            "navigation_action_distance_m": navigation_action_distance,
            "turn_speed_limit_mps": self.config.turn_speed_limit_mps,
            "goal_speed_limit_mps": goal_speed_limit,
            "correction_speed_limit_mps": correction_speed_limit,
            "correction_speed_limited": correction_speed_limited,
            "maneuver_correction_relaxation": maneuver_correction_relaxation,
            "maneuver_correction_geometry_safe": maneuver_geometry_safe,
            "correction_curvature_1pm": correction_curvature,
            "maneuver_curvature_speed_limit_mps": maneuver_curvature_speed_limit,
            "full_correction_speed_limit_mps": full_correction_speed_limit,
            "goal_speed_limited": (
                goal_speed_limit + 1e-12 < model_hint["target_speed_mps"]
            ),
            "model_control_hint": copy.deepcopy(model_hint),
            "route_target": {
                "x_forward_m": route_forward,
                "bearing_forward_m": bearing_forward,
                "y_left_m": route_left,
                "signed_cross_track_target_m": signed_route_distance,
                "selected_correction_left_m": correction_left,
                "target_lateral": route_lateral_target,
                "target_yaw": route_yaw_target,
                "tangent_target_yaw": route_tangent_yaw_target,
                "tangent_end_index": tangent_end_index,
                "tangent_forward_m": tangent_forward,
                "tangent_left_m": tangent_left,
                "tangent_yaw": route_tangent_yaw,
                "cross_track_bearing_yaw": cross_track_bearing_yaw,
                "cross_track_yaw_weight": cross_track_yaw_weight,
                "cross_track_yaw_correction_rad": cross_track_yaw_correction,
                "target_yaw_includes_cross_track_bearing": (
                    cross_track_yaw_weight > 0.0
                ),
                "tangent_reliable": tangent_reliable,
                "tangent_evidence": tangent_evidence,
            },
            "effective_control_hint": copy.deepcopy(effective_hint),
            "longitudinal_limits": {
                "model_target_mps": model_hint["target_speed_mps"],
                "effective_target_mps": effective_speed,
                "active_reasons": [name for name, active in (
                    ("cross_track_correction", correction_speed_limited),
                    ("goal_approach", goal_speed_limit < model_hint["target_speed_mps"]),
                    ("terminal_capture", terminal_capture_speed_limited),
                    ("recovery_capture", recovery_capture),
                    ("turn", turn_speed_limited),
                ) if active],
                "scope": "adapter_before_slew_and_final_safety",
            },
            "configuration": asdict(self.config),
            "boundary": {
                "model_speed_is_upper_bound": True,
                "route_goal_may_reduce_speed": True,
                "route_correction_may_reduce_speed": True,
                "route_feedback_modifies_lateral_yaw": True,
                "lateral_and_heading_errors_are_separated": True,
                "lateral_and_yaw_use_independent_blend_weights": True,
                "cross_track_error_drives_lateral_channel": True,
                "cross_track_yaw_capture_is_large_error_only": True,
                "cross_track_yaw_capture_is_bounded": True,
                "cross_track_yaw_capture_requires_signed_distance": True,
                "route_follow_lateral_outliers_are_rejected": True,
                "lateral_disagreement_requires_signed_route_distance": True,
                "authorized_maneuvers_bypass_route_adapter": True,
                "model_yaw_disagreement_does_not_force_speed_cap": True,
                "model_lateral_disagreement_does_not_force_speed_cap": True,
                "maneuver_route_authority_is_stateful": True,
                "maneuver_route_authority_requires_ordinary_route_follow": True,
                "maneuver_route_authority_does_not_raise_speed": True,
                "model_trajectory_replaced": False,
                "safety_shield_replaced": False,
                "official_controller_claimed": False,
            },
        }
        self._adaptations += 1
        self._activated_adaptations += int(evidence["activated"])
        self._last_evidence = copy.deepcopy(evidence)
        return evidence

    def report(self) -> dict[str, Any]:
        return {
            "schema_version": ROUTE_CONTROL_ADAPTER_SCHEMA_VERSION,
            "adaptations": self._adaptations,
            "activated_adaptations": self._activated_adaptations,
            "maneuver_route_authority": self._maneuver_route_authority,
            "maneuver_release_cycles_remaining": (
                self._maneuver_release_cycles_remaining
            ),
            "last_evidence": copy.deepcopy(self._last_evidence),
            "configuration": asdict(self.config),
            "boundary": {
                "runtime_only": True,
                "stage3_checkpoint_unchanged": True,
                "model_speed_is_upper_bound": True,
                "route_goal_may_reduce_speed": True,
                "route_correction_may_reduce_speed": True,
                "official_controller_claimed": False,
            },
        }
