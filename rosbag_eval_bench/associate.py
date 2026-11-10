# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import math

from rosbag_eval_bench.types import (
    DetectionFrame,
    EvalError,
    MatchedPoses,
    Pose2D,
    wrap_angle,
)


def associate_poses(
    gt: list[Pose2D],
    est: list[Pose2D],
    max_dt_ns: int = 50_000_000,
) -> MatchedPoses:
    if not gt:
        raise EvalError("no GT poses")
    if not est:
        raise EvalError("no estimate poses")

    est_sorted = sorted(est, key=lambda p: p.t_ns)
    matched_gt: list[Pose2D] = []
    matched_est: list[Pose2D] = []
    for g in gt:
        matched = _match_one(gt_pose=g, est=est_sorted, max_dt_ns=max_dt_ns)
        if matched is not None:
            matched_gt.append(g)
            matched_est.append(matched)

    n_gt = len(gt)
    n_matched = len(matched_gt)
    ratio = n_matched / n_gt
    return MatchedPoses(
        gt=matched_gt,
        est=matched_est,
        match_ratio=ratio,
        n_matched=n_matched,
        n_gt=n_gt,
    )


def _match_one(gt_pose: Pose2D, est: list[Pose2D], max_dt_ns: int) -> Pose2D | None:
    t = gt_pose.t_ns
    for p in est:
        if p.t_ns == t:
            return p
    # find bracket
    lo: Pose2D | None = None
    hi: Pose2D | None = None
    for p in est:
        if p.t_ns < t:
            lo = p
        elif p.t_ns > t:
            hi = p
            break
    if lo is None or hi is None:
        return None
    if not (lo.t_ns < t < hi.t_ns):
        return None
    if (t - lo.t_ns) > max_dt_ns or (hi.t_ns - t) > max_dt_ns:
        return None
    alpha = (t - lo.t_ns) / (hi.t_ns - lo.t_ns)
    x = lo.x + alpha * (hi.x - lo.x)
    y = lo.y + alpha * (hi.y - lo.y)
    dyaw = wrap_angle(hi.yaw - lo.yaw)
    if abs(abs(dyaw) - math.pi) < 1e-12:
        return None
    yaw = wrap_angle(lo.yaw + alpha * dyaw)
    return Pose2D(x, y, yaw, t)


def require_match_ratio(mp: MatchedPoses, min_ratio: float = 0.90) -> None:
    if mp.n_gt == 0:
        raise EvalError("n_gt is zero")
    if mp.match_ratio + 1e-15 < min_ratio:
        raise EvalError(f"match_ratio {mp.match_ratio} below {min_ratio}")


def associate_detection_frames(
    gt_frames: list[DetectionFrame],
    cand_frames: list[DetectionFrame],
    max_dt_ns: int = 50_000_000,
) -> list[tuple[DetectionFrame, DetectionFrame | None]]:
    gt_sorted = sorted(gt_frames, key=lambda f: f.t_ns)
    cand_sorted = sorted(cand_frames, key=lambda f: f.t_ns)
    used = [False] * len(cand_sorted)
    pairs: list[tuple[DetectionFrame, DetectionFrame | None]] = []

    for gf in gt_sorted:
        best_i: int | None = None
        best_dt: int | None = None
        for i, cf in enumerate(cand_sorted):
            if used[i]:
                continue
            dt = abs(cf.t_ns - gf.t_ns)
            if dt <= max_dt_ns and (
                best_dt is None or dt < best_dt or (dt == best_dt and i < (best_i or 0))
            ):
                best_dt = dt
                best_i = i
        if best_i is None:
            pairs.append((gf, None))
        else:
            used[best_i] = True
            pairs.append((gf, cand_sorted[best_i]))

    for i, cf in enumerate(cand_sorted):
        if not used[i]:
            # unmatched candidate FP frame — represented as None GT
            pairs.append(
                (
                    DetectionFrame(t_ns=cf.t_ns, detections=[]),
                    cf,
                )
            )
    return pairs
