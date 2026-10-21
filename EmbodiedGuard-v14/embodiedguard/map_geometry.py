"""Static public-grid adapter and independent continuous polyline audit.

No EmbodiedGuard imports here. Occupied squares and whole-map bounds remain
authoritative. Circles supplied to the old planner conservatively cover squares.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


EPS = 1e-9
MAX_EXCURSION_BOUND_M = 4.8


@dataclass
class Grid:
    name: str
    rows: tuple[str, ...]

    def __post_init__(self):
        self.height = len(self.rows)
        self.width = len(self.rows[0])
        self.occupied = np.array([[c not in ".G" for c in row] for row in self.rows], dtype=bool)
        row, col = np.nonzero(self.occupied)
        self.blocked_cells = np.column_stack((col, row))

    def cell_center(self, cell, scale):
        col, row = cell
        return np.array(((col + .5) * scale, (self.height - row - .5) * scale), dtype=float)

    def centers(self, scale):
        cells = self.blocked_cells
        return np.column_stack(((cells[:, 0] + .5) * scale, (self.height - cells[:, 1] - .5) * scale))


def parse_map(text, name):
    lines = text.splitlines()
    if len(lines) < 5 or lines[0].strip() != "type octile" or lines[3].strip() != "map":
        raise ValueError("Unsupported map header")
    if not lines[1].startswith("height ") or not lines[2].startswith("width "):
        raise ValueError("Missing map dimensions")
    height, width = int(lines[1].split()[1]), int(lines[2].split()[1])
    rows = tuple(lines[4:])
    if min(height, width) <= 0 or len(rows) != height or any(len(row) != width for row in rows):
        raise ValueError("Map dimensions disagree with actual rows")
    if set("".join(rows)) - set(".G@OT"):
        raise ValueError("Unsupported terrain semantics; do not guess passability")
    return Grid(name, rows)


def parse_scenarios(text, grid):
    lines = text.splitlines()
    if not lines or lines[0].strip() != "version 1":
        raise ValueError("Unsupported scenario version")
    records = []
    for file_line, line in enumerate(lines[1:], start=2):
        fields = line.split()
        if len(fields) != 9:
            raise ValueError(f"Wrong scenario field count at line {file_line}")
        bucket, name, width, height, sx, sy, gx, gy, length = fields
        width, height, sx, sy, gx, gy, bucket = map(int, (width, height, sx, sy, gx, gy, bucket))
        length = float(length)
        if name != grid.name + ".map" or (width, height) != (grid.width, grid.height):
            raise ValueError("Scenario map/dimensions require unsupported rescaling")
        if not math.isfinite(length) or length < 0:
            raise ValueError("Invalid official optimal grid length")
        for x, y in ((sx, sy), (gx, gy)):
            if not (0 <= x < width and 0 <= y < height) or grid.occupied[y, x]:
                raise ValueError("Scenario endpoint is not a passable point")
        records.append({"file_line": file_line, "bucket": bucket, "start_cell": [sx, sy],
                        "goal_cell": [gx, gy], "official_grid_optimal_cells": length})
    return records


def select_records(records, buckets, first_rows_per_bucket):
    selected = []
    for bucket in buckets:
        group = [r for r in records if r["bucket"] == bucket]
        if len(group) < first_rows_per_bucket:
            raise ValueError(f"Insufficient rows in frozen bucket {bucket}; no replacement")
        selected.extend(group[:first_rows_per_bucket])
    return selected


@dataclass
class RigidFrame:
    origin: np.ndarray
    cos: float
    sin: float
    distance: float

    @classmethod
    def from_pair(cls, start, goal):
        delta = np.asarray(goal) - np.asarray(start)
        distance = float(np.linalg.norm(delta))
        if not math.isfinite(distance) or distance <= EPS:
            raise ValueError("Coincident/nonfinite scenario endpoints")
        return cls(np.asarray(start, dtype=float), float(delta[0] / distance), float(delta[1] / distance), distance)

    def to_local(self, xy):
        delta = np.asarray(xy, dtype=float) - self.origin
        return np.stack((self.cos * delta[..., 0] + self.sin * delta[..., 1],
                         -self.sin * delta[..., 0] + self.cos * delta[..., 1]), axis=-1)

    def to_world(self, xy):
        xy = np.asarray(xy, dtype=float)
        return np.stack((self.cos * xy[..., 0] - self.sin * xy[..., 1],
                         self.sin * xy[..., 0] + self.cos * xy[..., 1]), axis=-1) + self.origin


def circle_obstacles(grid, frame, scale, footprint, reserve=.35):
    radius = math.sqrt(2) * scale / 2
    local = frame.to_local(grid.centers(scale))
    margin = radius + footprint + reserve + EPS
    keep = ((local[:, 0] >= -margin) & (local[:, 0] <= frame.distance + margin)
            & (np.abs(local[:, 1]) <= MAX_EXCURSION_BOUND_M + margin))
    indices = np.flatnonzero(keep)
    obstacles = []
    for index in indices:
        col, row = grid.blocked_cells[index]
        x, y = local[index]
        obstacles.append({"id": f"cell-{int(col)}-{int(row)}", "x": float(x), "y": float(y),
                          "radius": radius, "vx": 0., "vy": 0.})
    return obstacles


def point_boxes_distance_sq(point, centers, half):
    delta = np.maximum(np.abs(centers - point) - half, 0)
    return np.sum(delta * delta, axis=1)


def segment_boxes_distance_sq(a, b, centers, half):
    """Exact distance between one continuous segment and axis-aligned squares.

If disjoint, the minimum is at a segment endpoint or a square vertex. Clip
the segment against each box to detect intersections independently of that.
"""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    direction = b - a
    length_sq = float(np.dot(direction, direction))
    if length_sq <= EPS * EPS:
        return point_boxes_distance_sq(a, centers, half)
    best = np.minimum(point_boxes_distance_sq(a, centers, half), point_boxes_distance_sq(b, centers, half))
    for dx in (-half, half):
        for dy in (-half, half):
            corner = centers + (dx, dy)
            t = np.clip(((corner - a) @ direction) / length_sq, 0., 1.)
            delta = corner - (a + t[:, None] * direction)
            best = np.minimum(best, np.sum(delta * delta, axis=1))
    enter = np.zeros(len(centers))
    leave = np.ones(len(centers))
    possible = np.ones(len(centers), dtype=bool)
    for axis in range(2):
        if abs(direction[axis]) <= EPS:
            possible &= np.abs(a[axis] - centers[:, axis]) <= half
        else:
            t1 = (centers[:, axis] - half - a[axis]) / direction[axis]
            t2 = (centers[:, axis] + half - a[axis]) / direction[axis]
            enter = np.maximum(enter, np.minimum(t1, t2))
            leave = np.minimum(leave, np.maximum(t1, t2))
    best[possible & (enter <= leave)] = 0.
    return best


def endpoint_clearance(grid, point, scale, footprint):
    point = np.asarray(point, dtype=float)
    boundary = min(point[0], point[1], grid.width * scale - point[0], grid.height * scale - point[1])
    centers = grid.centers(scale)
    wall = float(np.sqrt(point_boxes_distance_sq(point, centers, scale / 2).min())) if len(centers) else math.inf
    return {"wall_clearance_m": wall - footprint, "boundary_clearance_m": float(boundary - footprint),
            "valid": bool(wall > footprint + EPS and boundary > footprint + EPS)}


def audit_polyline(grid, points_world, scale, footprint):
    points = np.asarray(points_world, dtype=float)
    if len(points) < 2 or not np.isfinite(points).all():
        raise ValueError("Invalid polyline")
    if (np.linalg.norm(np.diff(points, axis=0), axis=1) <= EPS).any():
        raise ValueError("Repeated adjacent path vertices")
    boundary = float(min(points[:, 0].min(), points[:, 1].min(),
                         grid.width * scale - points[:, 0].max(), grid.height * scale - points[:, 1].max()))
    centers = grid.centers(scale)
    if not len(centers):
        wall_distance, witness = math.inf, None
    else:
        wall_best_sq = float(point_boxes_distance_sq(points[0], centers, scale / 2).min())
        witness = None
        # A lower bound from the polyline AABB lets us discard only cells that
        # provably cannot improve the current minimum. The original grid is
        # never cropped for either clearance or boundary evaluation.
        path_lo, path_hi = points.min(axis=0), points.max(axis=0)
        gap = np.maximum(np.maximum(path_lo - (centers + scale / 2), (centers - scale / 2) - path_hi), 0)
        mask = np.sum(gap * gap, axis=1) <= wall_best_sq + EPS
        relevant_indices = np.flatnonzero(mask)
        relevant = centers[mask]
        for segment_index, (a, b) in enumerate(zip(points, points[1:])):
            lo, hi = np.minimum(a, b), np.maximum(a, b)
            gap = np.maximum(np.maximum(lo - (relevant + scale / 2), (relevant - scale / 2) - hi), 0)
            local_mask = np.sum(gap * gap, axis=1) <= wall_best_sq + EPS
            if not local_mask.any():
                continue
            distances = segment_boxes_distance_sq(a, b, relevant[local_mask], scale / 2)
            position = int(distances.argmin())
            value = float(distances[position])
            if value <= wall_best_sq + EPS:
                wall_best_sq = min(wall_best_sq, value)
                global_index = relevant_indices[np.flatnonzero(local_mask)[position]]
                witness = {"segment_index": segment_index, "occupied_cell": grid.blocked_cells[global_index].tolist()}
        wall_distance = math.sqrt(wall_best_sq)
    wall_clearance = wall_distance - footprint
    boundary_clearance = boundary - footprint
    safe = wall_clearance > EPS and boundary_clearance > EPS
    return {"safe": bool(safe), "minimum_wall_clearance_m": float(wall_clearance),
            "minimum_boundary_clearance_m": boundary_clearance, "nearest_wall_witness": witness,
            "checker": "continuous segment vs original square cells plus full-map boundary"}


def polyline_metrics(points):
    points = np.asarray(points, dtype=float)
    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    if (lengths <= EPS).any():
        raise ValueError("Repeated path vertex")
    curvatures = []
    for a, b, c in zip(points, points[1:], points[2:]):
        ab, bc, ac = b - a, c - b, c - a
        numerator = 2 * abs(float(ab[0] * bc[1] - ab[1] * bc[0]))
        denominator = float(np.linalg.norm(ab) * np.linalg.norm(bc) * np.linalg.norm(ac))
        curvatures.append(numerator / denominator if denominator > EPS else math.inf)
    return {"length_m": float(lengths.sum()), "maximum_numeric_curvature_per_m": max(curvatures, default=0.)}


def footprint_grid_components(grid, scale, footprint):
    """4-neighbor disk-center connectivity diagnostic; no Ackermann claim."""
    blocked = grid.occupied.copy()
    n = math.ceil(footprint / scale + .5)
    for dy in range(-n, n + 1):
        for dx in range(-n, n + 1):
            distance_sq = (max(abs(dx) * scale - scale / 2, 0)**2
                           + max(abs(dy) * scale - scale / 2, 0)**2)
            if distance_sq > footprint * footprint + EPS:
                continue
            ys = slice(max(0, -dy), min(grid.height, grid.height - dy))
            xs = slice(max(0, -dx), min(grid.width, grid.width - dx))
            yt = slice(max(0, dy), min(grid.height, grid.height + dy))
            xt = slice(max(0, dx), min(grid.width, grid.width + dx))
            blocked[yt, xt] |= grid.occupied[ys, xs]
    x = (np.arange(grid.width) + .5) * scale
    y = (np.arange(grid.height) + .5) * scale
    blocked |= ((x <= footprint + EPS) | (grid.width * scale - x <= footprint + EPS))[None, :]
    blocked |= ((y <= footprint + EPS) | (grid.height * scale - y <= footprint + EPS))[:, None]
    labels = np.full((grid.height, grid.width), -1, dtype=np.int32)
    component = 0
    for y0 in range(grid.height):
        for x0 in range(grid.width):
            if blocked[y0, x0] or labels[y0, x0] >= 0:
                continue
            labels[y0, x0] = component
            queue = deque([(x0, y0)])
            while queue:
                x0, y0 = queue.popleft()
                for x1, y1 in ((x0 - 1, y0), (x0 + 1, y0), (x0, y0 - 1), (x0, y0 + 1)):
                    if (0 <= x1 < grid.width and 0 <= y1 < grid.height
                            and not blocked[y1, x1] and labels[y1, x1] < 0):
                        labels[y1, x1] = component
                        queue.append((x1, y1))
            component += 1
    return labels, component
