from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Mapping, Optional

from .coordinate_adapter import world_to_ego_xy, wrap_degrees
from .route_types import RoutePlan, RouteState
from .waypoint_queue import RouteProjection, WaypointQueue, WaypointQueueConfig


@dataclass(frozen=True)
class RouteManagerConfig:
    waypoint_queue: WaypointQueueConfig = field(default_factory=WaypointQueueConfig)
    deviation_cross_track_m: float = 3.0
    deviation_heading_deg: float = 45.0
    completion_progress: float = 0.995
    completion_distance_m: float = 2.0
    completion_speed_mps: float = 0.5
    completion_hold_updates: int = 10
    preview_spacing_m: float = 5.0
    preview_count: int = 6
    tangent_lookahead_m: float = 8.0
    terminal_destination_activation_m: float = 0.5

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any] | None) -> "RouteManagerConfig":
        data = dict(values or {})
        queue_values = data.pop("waypoint_queue", {})
        if not isinstance(queue_values, Mapping):
            raise TypeError("waypoint_queue config must be a mapping")
        return cls(
            waypoint_queue=WaypointQueueConfig(**dict(queue_values)),
            **data,
        )


def terminal_target_speed_kmh(
    distance_to_goal_m: float,
    cruise_speed_kmh: float,
    config: RouteManagerConfig,
) -> float:
    target_speed = float(cruise_speed_kmh)
    distance = max(0.0, float(distance_to_goal_m))
    if distance < 10.0:
        target_speed = max(3.0, target_speed * distance / 10.0)
    if distance <= config.completion_distance_m:
        completion_speed_kmh = config.completion_speed_mps * 3.6
        target_speed = min(
            target_speed,
            max(0.5, completion_speed_kmh * 0.8),
        )
    return target_speed


class RouteManager:
    """Track a global route and emit compact state for runtime orchestration."""

    def __init__(
        self,
        plan: RoutePlan,
        config: RouteManagerConfig | None = None,
        route_revision: int = 0,
    ):
        self.plan = plan
        self.config = config or RouteManagerConfig()
        self.route_revision = int(route_revision)
        self.queue = WaypointQueue(plan, self.config.waypoint_queue)
        self._progress_m = 0.0
        self._completion_updates = 0
        self._completed = False

    def update(
        self,
        ego_x: float,
        ego_y: float,
        ego_yaw_deg: float,
        speed_mps: float,
        *,
        local_task_active: bool = False,
        recovery_active: bool = False,
        recovery_plan: Optional[str] = None,
        route_restored: bool = True,
    ) -> RouteState:
        values = (ego_x, ego_y, ego_yaw_deg, speed_mps)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("ego pose and speed must be finite")

        projection = self.queue.project(ego_x, ego_y)
        self.queue.advance(ego_x, ego_y)
        self._progress_m = max(self._progress_m, projection.along_m)
        projection, terminal_destination_guidance = (
            self._terminal_destination_projection(
                projection,
                ego_x=ego_x,
                ego_y=ego_y,
            )
        )

        if terminal_destination_guidance:
            final_point = self.plan.points[-1]
            destination = self.plan.destination
            target_world = {
                "x": float(destination["x"]),
                "y": float(destination["y"]),
                "z": float(destination.get("z", final_point.z)),
            }
            target_ego_x, target_ego_y = world_to_ego_xy(
                target_world["x"],
                target_world["y"],
                ego_x,
                ego_y,
                ego_yaw_deg,
            )
            preview_ego_items = [
                {
                    "index": final_point.index,
                    "x": target_ego_x,
                    "y": target_ego_y,
                    "road_option": "LANEFOLLOW",
                }
            ]
            target_lane_id = final_point.lane_id
            # Terminal geometry is a control-only continuation of the same
            # route command. Changing the navigation action here invalidates
            # the in-flight Stage 3 plan and creates an avoidable inference gap.
            next_navigation_action = self.queue.next_navigation_action()
        else:
            target = self.queue.lookahead_target(
                speed_mps,
                origin_distance_m=self._progress_m,
            )
            preview = self.queue.preview_targets(
                spacing_m=self.config.preview_spacing_m,
                count=self.config.preview_count,
                origin_distance_m=self._progress_m,
            )
            target_world = {"x": target.x, "y": target.y, "z": target.z}
            target_ego_x, target_ego_y = world_to_ego_xy(
                target.x,
                target.y,
                ego_x,
                ego_y,
                ego_yaw_deg,
            )
            preview_ego_items = []
            for point in preview:
                local_x, local_y = world_to_ego_xy(
                    point.x,
                    point.y,
                    ego_x,
                    ego_y,
                    ego_yaw_deg,
                )
                preview_ego_items.append(
                    {
                        "index": point.index,
                        "x": local_x,
                        "y": local_y,
                        "road_option": point.road_option,
                    }
                )
            target_lane_id = target.lane_id
            next_navigation_action = self.queue.next_navigation_action()
        preview_ego = tuple(preview_ego_items)
        cross_track = projection.distance_m
        _, projection_ego_right = world_to_ego_xy(
            projection.projected_x,
            projection.projected_y,
            ego_x,
            ego_y,
            ego_yaw_deg,
        )
        signed_cross_track = (
            0.0
            if cross_track <= 1e-9
            else math.copysign(cross_track, -projection_ego_right)
        )
        heading_error = abs(
            wrap_degrees(projection.heading_deg - float(ego_yaw_deg))
        )
        deviation = (
            cross_track > self.config.deviation_cross_track_m
            or heading_error > self.config.deviation_heading_deg
        )
        progress = min(
            1.0,
            max(
                0.0,
                self._progress_m / max(self.plan.total_distance_m, 1e-9),
            ),
        )
        goal_distance = self.queue.distance_to_goal(ego_x, ego_y)

        if terminal_destination_guidance:
            tangent_yaw_rad = (
                math.atan2(target_ego_y, target_ego_x)
                if math.hypot(target_ego_x, target_ego_y) > 1e-9
                else 0.0
            )
            curvature_1pm = 0.0
            tangent_source = "src.route.RouteManager.terminal_destination"
        else:
            tangent_yaw_rad, curvature_1pm = self._finite_horizon_geometry(
                projection.segment_index,
                origin_distance_m=self._progress_m,
                ego_yaw_deg=ego_yaw_deg,
            )
            tangent_source = "src.route.RouteManager.finite_horizon"

        completion_candidate = (
            progress >= self.config.completion_progress
            and goal_distance <= self.config.completion_distance_m
            and float(speed_mps) <= self.config.completion_speed_mps
            and not local_task_active
            and not recovery_active
        )
        self._completion_updates = self._completion_updates + 1 if completion_candidate else 0
        required_updates = max(1, int(self.config.completion_hold_updates))
        self._completed = self._completed or self._completion_updates >= required_updates

        return RouteState(
            route_id=self.plan.route_id,
            route_revision=self.route_revision,
            cursor=self.queue.cursor,
            route_size=len(self.plan.points),
            route_progress=progress,
            next_navigation_action=next_navigation_action,
            target_waypoint_world=target_world,
            target_waypoint_ego={"x": target_ego_x, "y": target_ego_y},
            preview_waypoints_ego=preview_ego,
            target_lane_id=target_lane_id,
            distance_to_target_m=math.hypot(
                target_world["x"] - ego_x,
                target_world["y"] - ego_y,
            ),
            distance_to_goal_m=goal_distance,
            cross_track_error_m=cross_track,
            heading_error_deg=heading_error,
            route_deviation=deviation,
            recovery_active=bool(recovery_active),
            recovery_plan=recovery_plan,
            local_task_active=bool(local_task_active),
            route_restored=bool(route_restored),
            route_completed=self._completed,
            signed_cross_track_error_m=signed_cross_track,
            route_tangent_yaw_rad=tangent_yaw_rad,
            route_curvature_1pm=curvature_1pm,
            route_tangent_lookahead_m=float(
                self.config.tangent_lookahead_m
            ),
            route_tangent_source=tangent_source,
            route_tracking_preview=(
                None if terminal_destination_guidance else
                self._near_tracking_preview(
                    projection.segment_index,
                    origin_distance_m=projection.along_m,
                    speed_mps=speed_mps,
                )
            ),
        )

    def _near_tracking_preview(
        self, segment_index: int, *, origin_distance_m: float, speed_mps: float,
    ) -> dict[str, Any]:
        """Short, distance-weighted heading lead, separate from the 8 m speed horizon.

        Integrating only the part of each segment inside the horizon avoids
        waypoint midpoint jumps. Never substitute a distant turn bearing for
        the current route heading, and request no lead while stationary.
        Output yaw uses the RouteManager/CARLA right-positive convention.
        """
        horizon = min(2.0, max(0.0, float(speed_mps)) * 0.6)
        points = self.plan.points
        start = max(0, min(int(segment_index), len(points) - 2))
        first, second = points[start:start + 2]
        base = math.atan2(second.y - first.y, second.x - first.x)
        forward = sideways = weight_sum = 0.0
        if horizon > 1e-6:
            for a, b in zip(points[start:], points[start + 1:]):
                lo = max(0.0, float(a.cumulative_distance_m) - origin_distance_m)
                hi = min(horizon, float(b.cumulative_distance_m) - origin_distance_m)
                if lo >= horizon:
                    break
                dx, dy = b.x - a.x, b.y - a.y
                if hi <= lo or math.hypot(dx, dy) <= 1e-9:
                    continue
                weight = (hi - lo) - (hi * hi - lo * lo) / (2.0 * horizon)
                relative = math.atan2(dy, dx) - base
                forward += weight * math.cos(relative)
                sideways += weight * math.sin(relative)
                weight_sum += weight
        offset = math.atan2(sideways, forward) if weight_sum > 1e-9 else 0.0
        return {
            "source": "route_polyline_near_heading_v1",
            "lookahead_m": horizon,
            "yaw_offset_rad": max(-0.12, min(0.12, offset)),
        }

    def _terminal_destination_projection(
        self,
        projection: RouteProjection,
        *,
        ego_x: float,
        ego_y: float,
    ) -> tuple[RouteProjection, bool]:
        activation = float(self.config.terminal_destination_activation_m)
        if not math.isfinite(activation) or activation < 0.0:
            raise ValueError(
                "terminal_destination_activation_m must be finite and non-negative"
            )
        remaining_trace_m = max(
            0.0,
            float(self.plan.total_distance_m) - float(self._progress_m),
        )
        if remaining_trace_m > activation + 1e-9:
            return projection, False

        final_point = self.plan.points[-1]
        destination = self.plan.destination
        destination_x = float(destination["x"])
        destination_y = float(destination["y"])
        dx = destination_x - float(final_point.x)
        dy = destination_y - float(final_point.y)
        extension_squared = dx * dx + dy * dy
        if extension_squared <= 1e-12:
            return projection, False

        raw_ratio = (
            (float(ego_x) - float(final_point.x)) * dx
            + (float(ego_y) - float(final_point.y)) * dy
        ) / extension_squared
        ratio = min(1.0, max(0.0, raw_ratio))
        projected_x = float(final_point.x) + ratio * dx
        projected_y = float(final_point.y) + ratio * dy
        return (
            RouteProjection(
                segment_index=max(0, len(self.plan.points) - 2),
                ratio=ratio,
                distance_m=math.hypot(
                    float(ego_x) - projected_x,
                    float(ego_y) - projected_y,
                ),
                along_m=float(self.plan.total_distance_m),
                heading_deg=math.degrees(math.atan2(dy, dx)),
                projected_x=projected_x,
                projected_y=projected_y,
            ),
            True,
        )

    def _finite_horizon_geometry(
        self,
        segment_index: int,
        *,
        origin_distance_m: float,
        ego_yaw_deg: float,
    ) -> tuple[float, float]:
        """Return the current tangent and finite-horizon route curvature.

        Steering follows the segment containing the current route projection.
        Future segments contribute only to curvature, so a bend later in the
        lookahead can reduce speed without becoming an early steering command.
        """
        horizon = float(self.config.tangent_lookahead_m)
        if not math.isfinite(horizon) or horizon <= 0.0:
            raise ValueError("tangent_lookahead_m must be positive")
        points = self.plan.points
        start = max(0, min(int(segment_index), len(points) - 2))
        headings: list[tuple[float, float]] = []
        for index in range(start, len(points) - 1):
            first = points[index]
            second = points[index + 1]
            dx = float(second.x) - float(first.x)
            dy = float(second.y) - float(first.y)
            length = math.hypot(dx, dy)
            if length <= 1e-9:
                continue
            midpoint = (
                0.5 * (float(first.cumulative_distance_m)
                        + float(second.cumulative_distance_m))
                - float(origin_distance_m)
            )
            if midpoint < -1e-6:
                continue
            if midpoint > horizon + 1e-6:
                break
            unit_x = dx / length
            unit_y = dy / length
            headings.append((math.atan2(unit_y, unit_x), length))

        current = points[start]
        following = points[start + 1]
        current_dx = float(following.x) - float(current.x)
        current_dy = float(following.y) - float(current.y)
        if math.hypot(current_dx, current_dy) <= 1e-9:
            world_yaw_rad = math.radians(float(current.yaw_deg))
        else:
            world_yaw_rad = math.atan2(current_dy, current_dx)
        curvature = 0.0
        for (left_yaw, left_len), (right_yaw, _) in zip(
            headings, headings[1:]
        ):
            delta = math.atan2(
                math.sin(right_yaw - left_yaw),
                math.cos(right_yaw - left_yaw),
            )
            curvature = max(curvature, abs(delta) / max(left_len, 1e-6))
        local_yaw = math.radians(
            wrap_degrees(math.degrees(world_yaw_rad) - float(ego_yaw_deg))
        )
        return local_yaw, curvature

    def update_from_carla_transform(
        self,
        ego_transform,
        speed_mps: float,
        **state_flags,
    ) -> RouteState:
        """Update from a CARLA Transform without importing the CARLA package."""
        return self.update(
            ego_x=float(ego_transform.location.x),
            ego_y=float(ego_transform.location.y),
            ego_yaw_deg=float(ego_transform.rotation.yaw),
            speed_mps=speed_mps,
            **state_flags,
        )
