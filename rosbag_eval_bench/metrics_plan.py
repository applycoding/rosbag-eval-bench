# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import math

from rosbag_eval_bench.types import EvalError, Grid, Path2D, PlanMetrics, Pose2D, wrap_angle


def planning(
    path: Path2D,
    goal: Pose2D,
    grid: Grid,
    robot_radius: float = 0.25,
    goal_tol_xy: float = 0.50,
    goal_tol_yaw: float = 0.25,
) -> PlanMetrics:
    if not path.poses:
        raise EvalError("empty planning path")
    occupied = _occupied_cells(grid)
    if not occupied:
        raise EvalError("map has no occupied cells")
    length = _path_length(path.poses)
    clearance, collision = _clearance_and_collision(path.poses, grid, occupied)
    success = _success(path.poses, goal, clearance, collision, robot_radius, goal_tol_xy, goal_tol_yaw)
    ttg = _time_to_goal(path.poses, goal, success, goal_tol_xy, goal_tol_yaw)
    smooth = _smoothness(path.poses, length)
    return PlanMetrics(
        path_length=length,
        clearance=clearance,
        success=success,
        time_to_goal=ttg,
        smoothness=smooth,
        collision=collision,
        path_poses=list(path.poses),
    )


def _path_length(poses: list[Pose2D]) -> float:
    total = 0.0
    for i in range(1, len(poses)):
        dx = poses[i].x - poses[i - 1].x
        dy = poses[i].y - poses[i - 1].y
        seg = math.hypot(dx, dy)
        if seg >= 1e-12:
            total += seg
    return total


def _occupied_cells(grid: Grid) -> list[tuple[float, float, float]]:
    h = grid.resolution / 2.0
    out: list[tuple[float, float, float]] = []
    for j in range(grid.height):
        for i in range(grid.width):
            idx = j * grid.width + i
            if grid.data[idx] >= 50:
                cx = grid.origin_x + (i + 0.5) * grid.resolution
                cy = grid.origin_y + (j + 0.5) * grid.resolution
                out.append((cx, cy, h))
    return out


def _point_square_dist(px: float, py: float, cx: float, cy: float, h: float) -> float:
    return math.hypot(max(abs(px - cx) - h, 0.0), max(abs(py - cy) - h, 0.0))


def _segment_square_dist(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    cx: float,
    cy: float,
    h: float,
) -> float:
    samples = 16
    best = math.inf
    for k in range(samples + 1):
        t = k / samples
        px = x0 + t * (x1 - x0)
        py = y0 + t * (y1 - y0)
        d = _point_square_dist(px, py, cx, cy, h)
        best = min(best, d)
    # corners of square
    corners = [
        (cx - h, cy - h),
        (cx + h, cy - h),
        (cx - h, cy + h),
        (cx + h, cy + h),
    ]
    for sx, sy in corners:
        d = _point_segment_dist(sx, sy, x0, y0, x1, y1)
        best = min(best, d)
    return best


def _point_segment_dist(px: float, py: float, x0: float, y0: float, x1: float, y1: float) -> float:
    dx = x1 - x0
    dy = y1 - y0
    if dx == 0 and dy == 0:
        return math.hypot(px - x0, py - y0)
    t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)))
    qx = x0 + t * dx
    qy = y0 + t * dy
    return math.hypot(px - qx, py - qy)


def _in_map(px: float, py: float, grid: Grid) -> bool:
    min_x = grid.origin_x
    min_y = grid.origin_y
    max_x = grid.origin_x + grid.width * grid.resolution
    max_y = grid.origin_y + grid.height * grid.resolution
    return min_x <= px < max_x and min_y <= py < max_y


def _clearance_and_collision(
    poses: list[Pose2D],
    grid: Grid,
    occupied: list[tuple[float, float, float]],
) -> tuple[float, bool]:
    best = math.inf
    collision = False
    for p in poses:
        if not _in_map(p.x, p.y, grid):
            raise EvalError("path vertex outside map")
        for cx, cy, h in occupied:
            d = _point_square_dist(p.x, p.y, cx, cy, h)
            best = min(best, d)
            if d <= 0.0:
                collision = True
    for i in range(1, len(poses)):
        p0, p1 = poses[i - 1], poses[i]
        mid_x = (p0.x + p1.x) / 2
        mid_y = (p0.y + p1.y) / 2
        if not _in_map(mid_x, mid_y, grid):
            raise EvalError("path segment outside map")
        for cx, cy, h in occupied:
            d = _segment_square_dist(p0.x, p0.y, p1.x, p1.y, cx, cy, h)
            best = min(best, d)
            if d <= 0.0:
                collision = True
    return float(best), collision


def _success(
    poses: list[Pose2D],
    goal: Pose2D,
    clearance: float,
    collision: bool,
    robot_radius: float,
    tol_xy: float,
    tol_yaw: float,
) -> float:
    if collision or clearance < robot_radius:
        return 0.0
    for p in poses:
        if math.hypot(p.x - goal.x, p.y - goal.y) <= tol_xy and abs(wrap_angle(p.yaw - goal.yaw)) <= tol_yaw:
            return 1.0
    return 0.0


def _time_to_goal(
    poses: list[Pose2D],
    goal: Pose2D,
    success: float,
    tol_xy: float,
    tol_yaw: float,
) -> float | None:
    if success < 0.5:
        return None
    t0 = poses[0].t_ns
    for p in poses:
        if math.hypot(p.x - goal.x, p.y - goal.y) <= tol_xy and abs(wrap_angle(p.yaw - goal.yaw)) <= tol_yaw:
            return (p.t_ns - t0) / 1e9
    return None


def _smoothness(poses: list[Pose2D], length: float) -> float | None:
    if len(poses) < 2:
        return None
    headings: list[float] = []
    for i in range(1, len(poses)):
        dx = poses[i].x - poses[i - 1].x
        dy = poses[i].y - poses[i - 1].y
        if math.hypot(dx, dy) < 1e-12:
            continue
        headings.append(math.atan2(dy, dx))
    if len(headings) < 2:
        return None
    if length < 1e-6:
        return None
    total = 0.0
    for i in range(1, len(headings)):
        total += abs(wrap_angle(headings[i] - headings[i - 1]))
    return total / length
