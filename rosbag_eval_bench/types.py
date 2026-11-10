# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal


def wrap_angle(angle: float) -> float:
    """Wrap angle into (-pi, pi]."""
    import math

    two_pi = 2.0 * math.pi
    a = (angle + math.pi) % two_pi
    if a <= 0:
        a += two_pi
    return a - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    import math

    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    import math

    hz = math.sin(yaw / 2.0)
    wz = math.cos(yaw / 2.0)
    return (0.0, 0.0, hz, wz)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float
    t_ns: int


@dataclass
class Path2D:
    poses: list[Pose2D]
    frame_id: str


@dataclass
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: list[int]


@dataclass(frozen=True)
class Detection:
    class_id: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float


@dataclass
class DetectionFrame:
    t_ns: int
    detections: list[Detection]


@dataclass
class MatchedPoses:
    gt: list[Pose2D]
    est: list[Pose2D]
    match_ratio: float
    n_matched: int
    n_gt: int


@dataclass
class MetricValue:
    name: str
    value: float | None
    unit: str
    direction: Literal["higher", "lower"]
    null_ok: bool = True


@dataclass
class CompareRow:
    name: str
    baseline: float | None
    candidate: float | None
    delta: float | None
    passed: bool
    reason: str
    gated: bool = False


@dataclass
class CompareResult:
    passed: bool
    rows: list[CompareRow]
    invalid: bool
    invalid_reason: str = ""


@dataclass
class TrajMetrics:
    ate_unaligned_rmse: float
    ate_unaligned_mean: float
    ate_unaligned_median: float
    ate_unaligned_std: float
    ate_unaligned_max: float
    ate_rmse: float
    ate_rot_rmse: float
    alignment_yaw: float
    alignment_tx: float
    alignment_ty: float
    rpe_trans_rmse: float
    rpe_rot_rmse: float
    match_ratio: float
    n_matched: int
    n_gt: int
    ate_series: list[dict[str, float]] = field(default_factory=list)


@dataclass
class PlanMetrics:
    path_length: float
    clearance: float
    success: float
    time_to_goal: float | None
    smoothness: float | None
    collision: bool
    path_poses: list[Pose2D] = field(default_factory=list)
    gt_path_poses: list[Pose2D] = field(default_factory=list)


@dataclass
class DetMetrics:
    precision_50: float
    recall_50: float
    map_50: float
    map_75: float
    map: float
    per_class: dict[str, dict[str, float]] = field(default_factory=dict)


class EvalError(Exception):
    """Invalid run (exit 2)."""

    def __init__(self, message: str) -> None:
        super().__init__(message)
        self.message = message
