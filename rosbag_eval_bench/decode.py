# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import math
from typing import Any

from rosbag_eval_bench.types import (
    Detection,
    DetectionFrame,
    EvalError,
    Grid,
    Path2D,
    Pose2D,
    yaw_from_quaternion,
)


def _get(obj: Any, *path: str) -> Any:
    cur = obj
    for key in path:
        if isinstance(cur, dict):
            cur = cur[key]
        else:
            cur = getattr(cur, key)
    return cur


def _validate_planar_pose(z: float, qx: float, qy: float) -> None:
    if abs(z) > 1e-6 or abs(qx) > 1e-6 or abs(qy) > 1e-6:
        raise EvalError("non-planar pose")


def _normalize_quaternion(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm < 1e-12:
        raise EvalError("invalid quaternion")
    if abs(norm - 1.0) > 1e-3:
        qx, qy, qz, qw = (qx / norm, qy / norm, qz / norm, qw / norm)
    return qx, qy, qz, qw


def decode_odometry(msg: Any, t_ns: int) -> Pose2D:
    x = float(_get(msg, "pose", "pose", "position", "x"))
    y = float(_get(msg, "pose", "pose", "position", "y"))
    z = float(_get(msg, "pose", "pose", "position", "z"))
    qx = float(_get(msg, "pose", "pose", "orientation", "x"))
    qy = float(_get(msg, "pose", "pose", "orientation", "y"))
    qz = float(_get(msg, "pose", "pose", "orientation", "z"))
    qw = float(_get(msg, "pose", "pose", "orientation", "w"))
    _validate_planar_pose(z, qx, qy)
    qx, qy, qz, qw = _normalize_quaternion(qx, qy, qz, qw)
    yaw = yaw_from_quaternion(qx, qy, qz, qw)
    return Pose2D(x, y, yaw, t_ns)


def _stamp_ns(header: Any) -> int:
    sec = int(_get(header, "stamp", "sec"))
    nanosec = int(_get(header, "stamp", "nanosec"))
    if nanosec < 0 or nanosec >= 1_000_000_000:
        raise EvalError("invalid header stamp nanosec")
    return sec * 1_000_000_000 + nanosec


def decode_pose_stamped(msg: Any, t_ns: int) -> Pose2D:
    x = float(_get(msg, "pose", "position", "x"))
    y = float(_get(msg, "pose", "position", "y"))
    z = float(_get(msg, "pose", "position", "z"))
    qx = float(_get(msg, "pose", "orientation", "x"))
    qy = float(_get(msg, "pose", "orientation", "y"))
    qz = float(_get(msg, "pose", "orientation", "z"))
    qw = float(_get(msg, "pose", "orientation", "w"))
    _validate_planar_pose(z, qx, qy)
    qx, qy, qz, qw = _normalize_quaternion(qx, qy, qz, qw)
    yaw = yaw_from_quaternion(qx, qy, qz, qw)
    stamp = _stamp_ns(msg.header)
    return Pose2D(x, y, yaw, stamp if stamp else t_ns)


def decode_path(msg: Any, log_t_ns: int) -> Path2D:
    frame_id = str(_get(msg, "header", "frame_id"))
    poses: list[Pose2D] = []
    for pst in _get(msg, "poses"):
        t_stamp = _stamp_ns(pst.header)
        x = float(_get(pst, "pose", "position", "x"))
        y = float(_get(pst, "pose", "position", "y"))
        z = float(_get(pst, "pose", "position", "z"))
        qx = float(_get(pst, "pose", "orientation", "x"))
        qy = float(_get(pst, "pose", "orientation", "y"))
        qz = float(_get(pst, "pose", "orientation", "z"))
        qw = float(_get(pst, "pose", "orientation", "w"))
        _validate_planar_pose(z, qx, qy)
        qx, qy, qz, qw = _normalize_quaternion(qx, qy, qz, qw)
        yaw = yaw_from_quaternion(qx, qy, qz, qw)
        poses.append(Pose2D(x, y, yaw, t_stamp if t_stamp else log_t_ns))
    if poses:
        last_t = poses[0].t_ns
        for p in poses[1:]:
            if p.t_ns < last_t:
                raise EvalError("path pose timestamps not monotonic")
            last_t = p.t_ns
    return Path2D(poses=poses, frame_id=frame_id)


def decode_grid(msg: Any) -> Grid:
    width = int(_get(msg, "info", "width"))
    height = int(_get(msg, "info", "height"))
    res = float(_get(msg, "info", "resolution"))
    origin_x = float(_get(msg, "info", "origin", "position", "x"))
    origin_y = float(_get(msg, "info", "origin", "position", "y"))
    qx = float(_get(msg, "info", "origin", "orientation", "x"))
    qy = float(_get(msg, "info", "origin", "orientation", "y"))
    qz = float(_get(msg, "info", "origin", "orientation", "z"))
    qw = float(_get(msg, "info", "origin", "orientation", "w"))
    qx, qy, qz, qw = _normalize_quaternion(qx, qy, qz, qw)
    origin_yaw = yaw_from_quaternion(qx, qy, qz, qw)
    if abs(origin_yaw) > 1e-6:
        raise EvalError("OccupancyGrid origin yaw must be 0")
    if width <= 0 or height <= 0 or res <= 0 or not math.isfinite(res):
        raise EvalError("invalid grid dimensions")
    data_raw = _get(msg, "data")
    data = [int(v) for v in data_raw]
    if width * height != len(data):
        raise EvalError("grid data length mismatch")
    return Grid(width, height, res, origin_x, origin_y, origin_yaw, data)


def decode_detection_array(msg: Any, t_ns: int) -> DetectionFrame:
    dets: list[Detection] = []
    for det in _get(msg, "detections"):
        theta = float(_get(det, "bbox", "center", "theta"))
        if abs(theta) > 1e-6:
            raise EvalError("detection bbox theta must be 0")
        results = _get(det, "results")
        if not results:
            raise EvalError("detection results empty")
        best_score = -1.0
        best_class = ""
        for hyp in results:
            class_id = str(_get(hyp, "hypothesis", "class_id"))
            score = float(_get(hyp, "hypothesis", "score"))
            if score < 0.0 or score > 1.0:
                raise EvalError("detection score out of range")
            if score > best_score + 1e-15:
                best_score = score
                best_class = class_id
        cx = float(_get(det, "bbox", "center", "position", "x"))
        cy = float(_get(det, "bbox", "center", "position", "y"))
        sx = float(_get(det, "bbox", "size_x"))
        sy = float(_get(det, "bbox", "size_y"))
        if sx < 0 or sy < 0 or not all(math.isfinite(v) for v in (cx, cy, sx, sy)):
            raise EvalError("invalid detection box size")
        x1 = cx - sx / 2.0
        x2 = cx + sx / 2.0
        y1 = cy - sy / 2.0
        y2 = cy + sy / 2.0
        dets.append(Detection(best_class, best_score, x1, y1, x2, y2))
    return DetectionFrame(t_ns=t_ns, detections=dets)
