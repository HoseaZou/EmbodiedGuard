from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
import math
from typing import Tuple

from .coordinate_adapter import wrap_degrees
from .route_types import RoutePlan, RoutePoint


@dataclass(frozen=True)
class WaypointQueueConfig:
    backtrack_window: int = 2
    forward_search_window: int = 40
    waypoint_reached_radius_m: float = 2.0
    lookahead_base_m: float = 3.0
    lookahead_speed_gain: float = 0.8
    lookahead_min_m: float = 5.0
    lookahead_max_m: float = 15.0
    navigation_search_distance_m: float = 30.0
    projection_continuity_tolerance_m: float = 0.25


@dataclass(frozen=True)
class RouteProjection:
    segment_index: int
    ratio: float
    distance_m: float
    along_m: float
    heading_deg: float
    projected_x: float
    projected_y: float


def _distance(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(float(x1) - float(x0), float(y1) - float(y0))


def _point_segment_distance(
    x: float,
    y: float,
    start: RoutePoint,
    end: RoutePoint,
) -> Tuple[float, float]:
    vx = end.x - start.x
    vy = end.y - start.y
    denom = vx * vx + vy * vy
    if denom <= 1e-9:
        return _distance(x, y, start.x, start.y), 0.0
    t = ((x - start.x) * vx + (y - start.y) * vy) / denom
    clamped = min(1.0, max(0.0, t))
    px = start.x + clamped * vx
    py = start.y + clamped * vy
    return _distance(x, y, px, py), t


class WaypointQueue:
    """Immutable route points plus a monotonic runtime cursor."""

    def __init__(self, plan: RoutePlan, config: WaypointQueueConfig | None = None):
        self.plan = plan
        self.config = config or WaypointQueueConfig()
        self.cursor = 0
        self._cumulative = [point.cumulative_distance_m for point in plan.points]

    @property
    def points(self) -> Tuple[RoutePoint, ...]:
        return self.plan.points

    @property
    def current(self) -> RoutePoint:
        return self.points[self.cursor]

    @property
    def remaining_points(self) -> Tuple[RoutePoint, ...]:
        return self.points[self.cursor :]

    def find_nearest_forward(self, ego_x: float, ego_y: float) -> int:
        start = max(0, self.cursor - self.config.backtrack_window)
        end = min(len(self.points), self.cursor + self.config.forward_search_window + 1)
        return min(
            range(start, end),
            key=lambda index: _distance(ego_x, ego_y, self.points[index].x, self.points[index].y),
        )

    def advance(self, ego_x: float, ego_y: float) -> int:
        projection = self.project(ego_x, ego_y)
        next_cursor = projection.segment_index
        if projection.ratio >= 0.5:
            next_cursor += 1
        self.cursor = max(self.cursor, min(next_cursor, len(self.points) - 1))
        return self.cursor

    def project(self, ego_x: float, ego_y: float) -> RouteProjection:
        if len(self.points) == 1:
            point = self.points[0]
            return RouteProjection(
                segment_index=0,
                ratio=0.0,
                distance_m=_distance(ego_x, ego_y, point.x, point.y),
                along_m=float(point.cumulative_distance_m),
                heading_deg=float(point.yaw_deg),
                projected_x=float(point.x),
                projected_y=float(point.y),
            )
        start = max(0, self.cursor - self.config.backtrack_window)
        end = min(
            len(self.points) - 1,
            self.cursor + self.config.forward_search_window,
        )
        candidates = []
        for index in range(start, end):
            first = self.points[index]
            second = self.points[index + 1]
            if _distance(first.x, first.y, second.x, second.y) <= 1e-9:
                continue
            distance_m, raw_ratio = _point_segment_distance(
                ego_x,
                ego_y,
                first,
                second,
            )
            ratio = min(1.0, max(0.0, raw_ratio))
            projected_x = first.x + ratio * (second.x - first.x)
            projected_y = first.y + ratio * (second.y - first.y)
            segment_length = max(
                0.0,
                float(second.cumulative_distance_m)
                - float(first.cumulative_distance_m),
            )
            candidates.append(
                RouteProjection(
                    segment_index=index,
                    ratio=ratio,
                    distance_m=distance_m,
                    along_m=(
                        float(first.cumulative_distance_m)
                        + ratio * segment_length
                    ),
                    heading_deg=math.degrees(
                        math.atan2(second.y - first.y, second.x - first.x)
                    ),
                    projected_x=projected_x,
                    projected_y=projected_y,
                )
            )
        if not candidates:
            nearest_index = self.find_nearest_forward(ego_x, ego_y)
            point = self.points[nearest_index]
            return RouteProjection(
                segment_index=min(nearest_index, len(self.points) - 2),
                ratio=0.0,
                distance_m=_distance(ego_x, ego_y, point.x, point.y),
                along_m=float(point.cumulative_distance_m),
                heading_deg=float(point.yaw_deg),
                projected_x=float(point.x),
                projected_y=float(point.y),
            )
        nearest_distance = min(item.distance_m for item in candidates)
        continuity_tolerance = max(
            0.0,
            float(self.config.projection_continuity_tolerance_m),
        )
        near_candidates = [
            item
            for item in candidates
            if item.distance_m <= nearest_distance + continuity_tolerance
        ]
        cursor_progress = float(self.points[self.cursor].cumulative_distance_m)
        continuous = min(
            near_candidates,
            key=lambda item: (
                abs(item.along_m - cursor_progress),
                item.segment_index,
            ),
        )
        # The continuity tolerance disambiguates crossing route branches; it
        # must not pin the projection to the *endpoint* of the previous
        # segment when the ego has entered its immediate neighbour. Otherwise
        # longitudinal travel is counted as CTE (up to the tolerance) and the
        # heading sticks, then jumps at each waypoint. Refine only adjacent
        # interiors, preserving the branch selection at route crossings.
        if continuous.ratio <= 1e-9 or continuous.ratio >= 1.0 - 1e-9:
            local_interiors = [
                item for item in near_candidates
                if abs(item.segment_index - continuous.segment_index) == 1
                and 1e-9 < item.ratio < 1.0 - 1e-9
                and item.distance_m < continuous.distance_m - 1e-9
            ]
            if local_interiors:
                return min(local_interiors, key=lambda item: (
                    item.distance_m, abs(item.along_m - cursor_progress), item.segment_index,
                ))
        return continuous

    def lookahead_distance(self, speed_mps: float) -> float:
        raw = self.config.lookahead_base_m + self.config.lookahead_speed_gain * max(0.0, float(speed_mps))
        return min(self.config.lookahead_max_m, max(self.config.lookahead_min_m, raw))

    def lookahead_target(
        self,
        speed_mps: float,
        *,
        origin_distance_m: float | None = None,
    ) -> RoutePoint:
        origin = (
            float(self.current.cumulative_distance_m)
            if origin_distance_m is None
            else float(origin_distance_m)
        )
        target_distance = origin + self.lookahead_distance(speed_mps)
        index = bisect_left(self._cumulative, target_distance, lo=self.cursor)
        return self.points[min(index, len(self.points) - 1)]

    def preview_targets(
        self,
        spacing_m: float = 5.0,
        count: int = 6,
        *,
        origin_distance_m: float | None = None,
    ) -> Tuple[RoutePoint, ...]:
        """Sample a short forward route preview for Safety and control."""
        if spacing_m <= 0.0 or count <= 0:
            return ()
        selected = []
        previous_index = None
        origin = (
            float(self.current.cumulative_distance_m)
            if origin_distance_m is None
            else float(origin_distance_m)
        )
        for step in range(1, int(count) + 1):
            target_distance = origin + float(spacing_m) * step
            index = min(
                bisect_left(self._cumulative, target_distance, lo=self.cursor),
                len(self.points) - 1,
            )
            if index != previous_index:
                selected.append(self.points[index])
                previous_index = index
            if index == len(self.points) - 1:
                break
        return tuple(selected)

    def cross_track_error(self, ego_x: float, ego_y: float) -> float:
        return self.project(ego_x, ego_y).distance_m

    def heading_error_deg(self, ego_yaw_deg: float) -> float:
        return abs(
            wrap_degrees(
                self.current.yaw_deg - float(ego_yaw_deg)
            )
        )

    def route_progress(self) -> float:
        if self.plan.total_distance_m <= 1e-9:
            return 1.0 if self.cursor == len(self.points) - 1 else 0.0
        return min(1.0, max(0.0, self.current.cumulative_distance_m / self.plan.total_distance_m))

    def distance_to_goal(self, ego_x: float, ego_y: float) -> float:
        goal = self.plan.destination
        return _distance(
            ego_x,
            ego_y,
            float(goal["x"]),
            float(goal["y"]),
        )

    def next_navigation_action(self) -> str:
        distance_limit = self.current.cumulative_distance_m + self.config.navigation_search_distance_m
        for point in self.points[self.cursor :]:
            if point.cumulative_distance_m > distance_limit:
                break
            if point.road_option not in {"LANEFOLLOW", "VOID"}:
                return point.road_option
        return "LANEFOLLOW"
