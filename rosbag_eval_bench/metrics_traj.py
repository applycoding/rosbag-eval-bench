# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import math

import numpy as np

from rosbag_eval_bench.types import EvalError, MatchedPoses, Pose2D, TrajMetrics, wrap_angle


def ate_rpe(mp: MatchedPoses, rpe_delta_s: float = 1.0, min_rpe_pairs: int = 3) -> TrajMetrics:
    if len(mp.gt) < 3:
        raise EvalError("fewer than 3 matched poses for alignment")
    gt_pts = np.array([[p.x, p.y] for p in mp.gt], dtype=np.float64)
    est_pts = np.array([[p.x, p.y] for p in mp.est], dtype=np.float64)
    if _all_coincident(gt_pts):
        raise EvalError("coincident pose set")
    unaligned_err = np.linalg.norm(est_pts - gt_pts, axis=1)
    unaligned_rmse = float(np.sqrt(np.mean(unaligned_err**2)))
    unaligned_mean = float(np.mean(unaligned_err))
    unaligned_median = float(np.median(unaligned_err))
    unaligned_std = float(np.sqrt(np.mean((unaligned_err - unaligned_mean) ** 2)))
    unaligned_max = float(np.max(unaligned_err))

    r_mat, t_vec = umeyama_se2(est_pts, gt_pts)
    aligned_est = (r_mat @ est_pts.T).T + t_vec
    aligned_err = np.linalg.norm(aligned_est - gt_pts, axis=1)
    ate_rmse = float(np.sqrt(np.mean(aligned_err**2)))
    alignment_yaw = math.atan2(r_mat[1, 0], r_mat[0, 0])
    alignment_tx = float(t_vec[0])
    alignment_ty = float(t_vec[1])

    yaw_err = []
    for g, e in zip(mp.gt, mp.est, strict=True):
        est_yaw = wrap_angle(e.yaw + alignment_yaw)
        yaw_err.append(wrap_angle(est_yaw - g.yaw))
    ate_rot_rmse = float(np.sqrt(np.mean(np.array(yaw_err) ** 2)))

    rpe_trans, rpe_rot = _compute_rpe(mp, rpe_delta_s, min_rpe_pairs)

    series = []
    for i, (g, e) in enumerate(zip(mp.gt, mp.est, strict=True)):
        ae = aligned_est[i]
        series.append(
            {
                "t_s": g.t_ns / 1e9,
                "err_m": float(np.linalg.norm(ae - gt_pts[i])),
            }
        )

    return TrajMetrics(
        ate_unaligned_rmse=unaligned_rmse,
        ate_unaligned_mean=unaligned_mean,
        ate_unaligned_median=unaligned_median,
        ate_unaligned_std=unaligned_std,
        ate_unaligned_max=unaligned_max,
        ate_rmse=ate_rmse,
        ate_rot_rmse=ate_rot_rmse,
        alignment_yaw=alignment_yaw,
        alignment_tx=alignment_tx,
        alignment_ty=alignment_ty,
        rpe_trans_rmse=rpe_trans,
        rpe_rot_rmse=rpe_rot,
        match_ratio=mp.match_ratio,
        n_matched=mp.n_matched,
        n_gt=mp.n_gt,
        ate_series=series,
    )


def umeyama_se2(est: np.ndarray, gt: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Estimate-to-GT SE(2), column vectors, no scale."""
    mean_e = np.mean(est, axis=0)
    mean_g = np.mean(gt, axis=0)
    ec = est - mean_e
    gc = gt - mean_g
    h_mat = ec.T @ gc
    u_mat, _, vt = np.linalg.svd(h_mat)
    d_mat = np.diag([1.0, np.linalg.det(vt.T @ u_mat.T)])
    rot = vt.T @ d_mat @ u_mat.T
    trans = mean_g - rot @ mean_e
    return rot, trans


def _all_coincident(pts: np.ndarray) -> bool:
    if len(pts) < 2:
        return True
    return bool(np.allclose(pts, pts[0]))


def _se2_from_pose(p: Pose2D) -> np.ndarray:
    c = math.cos(p.yaw)
    s = math.sin(p.yaw)
    mat = np.array([[c, -s, p.x], [s, c, p.y], [0.0, 0.0, 1.0]], dtype=np.float64)
    return mat


def _inv_se2(t_mat: np.ndarray) -> np.ndarray:
    rot = t_mat[:2, :2]
    trans = t_mat[:2, 2]
    inv_rot = rot.T
    inv_t = -inv_rot @ trans
    out = np.eye(3)
    out[:2, :2] = inv_rot
    out[:2, 2] = inv_t
    return out


def _rel_error(gt_i: Pose2D, gt_j: Pose2D, est_i: Pose2D, est_j: Pose2D) -> tuple[float, float]:
    t_gt_i = _se2_from_pose(gt_i)
    t_gt_j = _se2_from_pose(gt_j)
    t_est_i = _se2_from_pose(est_i)
    t_est_j = _se2_from_pose(est_j)
    delta_gt = _inv_se2(t_gt_i) @ t_gt_j
    delta_est = _inv_se2(t_est_i) @ t_est_j
    err = _inv_se2(delta_gt) @ delta_est
    trans = math.hypot(err[0, 2], err[1, 2])
    rot = wrap_angle(math.atan2(err[1, 0], err[0, 0]))
    return trans, math.degrees(rot)


def _compute_rpe(mp: MatchedPoses, delta_s: float, min_pairs: int) -> tuple[float, float]:
    delta_ns = int(delta_s * 1e9)
    max_dt = 50_000_000
    trans_errs: list[float] = []
    rot_errs: list[float] = []
    for i, g_i in enumerate(mp.gt):
        target_t = g_i.t_ns + delta_ns
        best_j: int | None = None
        best_dt: int | None = None
        for j in range(i + 1, len(mp.gt)):
            g_j = mp.gt[j]
            dt = abs(g_j.t_ns - target_t)
            if dt <= max_dt and (best_dt is None or dt < best_dt):
                best_dt = dt
                best_j = j
        if best_j is None:
            continue
        te, re = _rel_error(mp.gt[i], mp.gt[best_j], mp.est[i], mp.est[best_j])
        trans_errs.append(te)
        rot_errs.append(re)
    if len(trans_errs) < min_pairs:
        raise EvalError("fewer than minimum RPE pairs")
    trans_rmse = float(np.sqrt(np.mean(np.array(trans_errs) ** 2)))
    rot_rmse = float(np.sqrt(np.mean(np.array(rot_errs) ** 2)))
    return trans_rmse, rot_rmse
