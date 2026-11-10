# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import math

import numpy as np
import pytest

from rosbag_eval_bench.associate import associate_poses
from rosbag_eval_bench.metrics_det import _ap_one_class, iou
from rosbag_eval_bench.metrics_plan import _point_square_dist, planning
from rosbag_eval_bench.metrics_traj import ate_rpe, umeyama_se2
from rosbag_eval_bench.types import Detection, DetectionFrame, EvalError, Grid, Path2D, Pose2D


def test_iou_seven():
    a = Detection("c", 1.0, 0, 0, 2, 2)
    b = Detection("c", 1.0, 1, 1, 3, 3)
    assert abs(iou(a, b) - 1 / 7) < 1e-12


def test_ate_constant_offset():
    ts = [k * 1_000_000_000 for k in range(4)]
    gt = [Pose2D(k, 0, 0, ts[k]) for k in range(4)]
    est = [Pose2D(k + 0.1, 0.1, 0, ts[k]) for k in range(4)]
    mp = associate_poses(gt, est)
    tm = ate_rpe(mp)
    assert abs(tm.ate_unaligned_rmse - math.sqrt(0.02)) < 1e-12
    assert tm.ate_rmse < 1e-12
    assert abs(tm.alignment_tx + 0.1) < 1e-12
    assert abs(tm.alignment_ty + 0.1) < 1e-12
    assert abs(tm.alignment_yaw) < 1e-12


def test_ate_ninety_degree():
    ts = [k * 1_000_000_000 for k in range(4)]
    gt = [Pose2D(k, 0, 0, ts[k]) for k in range(4)]
    est = [Pose2D(0, k, 0, ts[k]) for k in range(4)]
    mp = associate_poses(gt, est)
    tm = ate_rpe(mp)
    assert abs(tm.alignment_yaw + math.pi / 2) < 1e-12
    assert tm.ate_rmse < 1e-12


def test_ate_drift():
    gt_pts = np.array([[0, 0], [1, 0], [2, 0]], dtype=float)
    est_pts = np.array([[0, 0], [1.15, 0], [2.30, 0]], dtype=float)
    r, t = umeyama_se2(est_pts, gt_pts)
    aligned = (r @ est_pts.T).T + t
    rmse = float(np.sqrt(np.mean(np.sum((aligned - gt_pts) ** 2, axis=1))))
    assert abs(rmse - math.sqrt(0.015)) < 1e-12


def test_rpe_cases():
    def poses(xs, ys, ts):
        return [Pose2D(x, y, 0, t) for x, y, t in zip(xs, ys, ts, strict=True)]

    t = [0, 1_000_000_000, 2_000_000_000, 3_000_000_000]
    gt = poses([0, 0.1, 0.2, 0.3], [0, 0, 0, 0], t)
    est = poses([0, 0.1, 0.2, 0.3], [0, 0, 0, 0], t)
    mp = associate_poses(gt, est)
    tm = ate_rpe(mp)
    assert tm.rpe_trans_rmse < 1e-12
    assert tm.rpe_rot_rmse < 1e-12

    est2 = poses([0, 0.2, 0.4, 0.6], [0, 0, 0, 0], t)
    mp2 = associate_poses(gt, est2)
    tm2 = ate_rpe(mp2)
    assert abs(tm2.rpe_trans_rmse - 0.1) < 1e-12

    # Same body-frame translations as GT; each 1 s pair has a 1° relative yaw error.
    yaw = math.radians(1.0)
    est3 = [Pose2D(0.0, 0.0, 0.0, t[0])]
    x = y = th = 0.0
    for i in range(1, 4):
        x += 0.1 * math.cos(th)
        y += 0.1 * math.sin(th)
        th += yaw
        est3.append(Pose2D(x, y, th, t[i]))
    mp3 = associate_poses(gt, est3)
    tm3 = ate_rpe(mp3)
    assert abs(tm3.rpe_rot_rmse - 1.0) < 1e-12
    assert tm3.rpe_trans_rmse < 1e-12


def test_ap_hand():
    gt_f = DetectionFrame(
        0,
        [
            Detection("c", 1.0, 0, 0, 2, 2),
            Detection("c", 1.0, 4, 0, 6, 2),
        ],
    )
    cand_f = DetectionFrame(
        0,
        [
            Detection("c", 0.9, 0, 0, 2, 2),
            Detection("c", 0.8, 10, 10, 12, 12),
            Detection("c", 0.7, 4, 0, 6, 2),
        ],
    )
    ap = _ap_one_class([(gt_f, cand_f)], "c", 0.5)
    assert abs(ap - 5 / 6) < 1e-12


def test_smoothness_l_path():
    goal = Pose2D(2, 0, 0, 0)
    path = Path2D(
        [Pose2D(0, 0, 0, 0), Pose2D(0, 2, 0, 1), Pose2D(2, 2, 0, 2)],
        "map",
    )
    data = [0] * (40 * 40)
    data[20 * 40 + 20] = 100
    grid = Grid(40, 40, 0.1, 0.0, 0.0, 0.0, data)
    pm = planning(path, goal, grid, goal_tol_xy=1.0, goal_tol_yaw=1.0)
    assert pm.smoothness is not None
    assert abs(pm.smoothness - (math.pi / 2) / 4) < 1e-12


def test_clearance_corner():
    res = 0.05
    h = res / 2
    cx, cy = 1.0, 1.0
    eps = 1e-6
    d = _point_square_dist(cx + h + eps, cy + h + eps, cx, cy, h)
    assert abs(d - eps * math.sqrt(2)) < 1e-9


def test_umeyama_mirrored_not_zero():
    gt = np.array([[0, 0], [1, 0], [0, 1]], dtype=float)
    est = np.array([[0, 0], [1, 0], [0, -1]], dtype=float)
    r, t = umeyama_se2(est, gt)
    aligned = (r @ est.T).T + t
    rmse = float(np.sqrt(np.mean(np.sum((aligned - gt) ** 2, axis=1))))
    assert rmse > 1e-6

def test_rpe_two_pairs_is_invalid():
    # Three poses at 1 s spacing yield only two RPE pairs; plan requires exit 2.
    t = [0, 1_000_000_000, 2_000_000_000]
    gt = [Pose2D(0.1 * i, 0, 0, t[i]) for i in range(3)]
    mp = associate_poses(gt, gt)
    with pytest.raises(EvalError, match="RPE"):
        ate_rpe(mp)


def test_ap_no_prediction():
    gt_f = DetectionFrame(0, [Detection("c", 1.0, 0, 0, 2, 2)])
    cand_f = DetectionFrame(0, [])
    ap = _ap_one_class([(gt_f, cand_f)], "c", 0.5)
    assert ap == 0.0
