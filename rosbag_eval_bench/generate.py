# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import math
import random
from pathlib import Path

from rosbag_eval_bench.eval_runner import evaluate_suite, write_results
from rosbag_eval_bench.mcap_io import McapFixtureWriter
from rosbag_eval_bench.mcap_messages import (
    detection_array,
    occupancy_grid,
    odometry,
    path_msg,
    pose_stamped,
)
from rosbag_eval_bench.schema import load_suite
from rosbag_eval_bench.types import Detection, Grid, Pose2D

T0_NS = 1_000_000_000_000
HZ20 = 50_000_000
HZ10 = 100_000_000
EPISODE_S = 5.0


def _make_map() -> Grid:
    w, h = 80, 80
    res = 0.05
    data = [0] * (w * h)
    # obstacle block in the center forcing detour (2m x 2m region)
    for j in range(30, 50):
        for i in range(30, 50):
            data[j * w + i] = 100
    return Grid(w, h, res, 0.0, 0.0, 0.0, data)


def _gt_trajectory(t0: int) -> list[Pose2D]:
    poses = []
    n = int(EPISODE_S * 20)
    for k in range(n + 1):
        t_ns = t0 + k * HZ20
        x = min(2.0, k * 0.02)
        poses.append(Pose2D(x, 0.0, 0.0, t_ns))
    return poses


def _nominal_est(gt: list[Pose2D]) -> list[Pose2D]:
    return [Pose2D(g.x + 0.05, g.y + 0.05, g.yaw, g.t_ns) for g in gt]


def _degraded_est(gt: list[Pose2D]) -> list[Pose2D]:
    t0 = gt[0].t_ns
    out = []
    for g in gt:
        t_s = (g.t_ns - t0) / 1e9
        out.append(Pose2D(g.x + 0.15 * t_s, g.y, g.yaw, g.t_ns))
    return out


def _nominal_path(t0: int, goal: Pose2D) -> list[Pose2D]:
    """Collision-free route north of the center obstacle."""
    t_end = t0 + int(EPISODE_S * 1e9)
    return [
        Pose2D(0.0, 0.0, 0.0, t0),
        Pose2D(0.0, 2.8, math.pi / 2, t0 + 800_000_000),
        Pose2D(3.0, 2.8, 0.0, t0 + 1_600_000_000),
        Pose2D(3.0, 3.0, 0.0, t_end),
    ]


def _degraded_path(t0: int, goal: Pose2D) -> list[Pose2D]:
    """Collision-free detour with >=1 m extra length versus nominal."""
    t_end = t0 + int(EPISODE_S * 1e9)
    return [
        Pose2D(0.0, 0.0, 0.0, t0),
        Pose2D(0.0, 2.8, math.pi / 2, t0 + 800_000_000),
        Pose2D(0.0, 3.4, math.pi / 2, t0 + 1_200_000_000),
        Pose2D(3.4, 3.4, 0.0, t0 + 2_000_000_000),
        Pose2D(3.4, 2.8, -math.pi / 2, t0 + 2_600_000_000),
        Pose2D(3.0, 3.0, 0.0, t_end),
    ]


def _detection_frames(rng: random.Random, t0: int, degraded: bool) -> tuple[list, list]:
    gt_frames = []
    cand_frames = []
    n_frames = int(EPISODE_S * 10) + 1
    person_count = 0
    for k in range(n_frames):
        t_ns = t0 + k * HZ10
        gt_dets: list[Detection] = []
        cand_dets: list[Detection] = []
        # vehicle
        gt_dets.append(Detection("vehicle", 0.95, 100.0, 80.0, 200.0, 160.0))
        cand_dets.append(Detection("vehicle", 0.92, 102.0, 82.0, 202.0, 162.0))
        # persons (>=10 total across frames)
        for _ in range(2):
            person_count += 1
            px = 300.0 + rng.uniform(-20, 20)
            py = 200.0 + rng.uniform(-20, 20)
            gt_dets.append(Detection("person", 0.9, px, py, px + 40, py + 80))
            if not degraded:
                cand_dets.append(Detection("person", 0.88, px + 2, py + 2, px + 42, py + 82))
        gt_frames.append((t_ns, gt_dets))
        cand_frames.append((t_ns, cand_dets))
    assert person_count >= 10
    return gt_frames, cand_frames


def write_scenario_bag(path: Path, degraded: bool, seed: int) -> None:
    rng = random.Random(seed)
    grid = _make_map()
    goal = Pose2D(3.0, 3.0, 0.0, T0_NS)
    gt_poses = _gt_trajectory(T0_NS)
    est_poses = _degraded_est(gt_poses) if degraded else _nominal_est(gt_poses)
    path_poses = _degraded_path(T0_NS, goal) if degraded else _nominal_path(T0_NS, goal)
    gt_dets, cand_dets = _detection_frames(rng, T0_NS, degraded)

    w = McapFixtureWriter(path)
    try:
        for p in gt_poses:
            w.write_odometry("/gt/pose", odometry(p), p.t_ns)
        for p in est_poses:
            w.write_odometry("/candidate/pose", odometry(p), p.t_ns)
        w.write_path("/candidate/path", path_msg(path_poses, path_poses[-1].t_ns), path_poses[-1].t_ns)
        w.write_pose("/gt/goal", pose_stamped(goal), T0_NS)
        w.write_grid("/gt/map", occupancy_grid(grid, T0_NS), T0_NS)
        gt_overlay = _nominal_path(T0_NS, goal)
        w.write_path("/gt/path", path_msg(gt_overlay, T0_NS), T0_NS)
        for t_ns, dets in gt_dets:
            w.write_detections("/gt/detections", detection_array(dets, t_ns), t_ns)
        for t_ns, dets in cand_dets:
            w.write_detections("/candidate/detections", detection_array(dets, t_ns), t_ns)
    finally:
        w.close()


DEFAULT_SUITE = {
    "name": "synthetic_demo",
    "metric_version": 1,
    "gate_policy_version": 1,
    "thresholds": {},
    "runs": [
        {
            "name": "full_stack",
            "bag": "bags/nominal.mcap",
            "metrics": ["planning", "localization", "detection"],
        }
    ],
}

DEGRADED_SUITE = {
    "name": "synthetic_demo",
    "metric_version": 1,
    "gate_policy_version": 1,
    "thresholds": {},
    "runs": [
        {
            "name": "full_stack",
            "bag": "bags/degraded.mcap",
            "metrics": ["planning", "localization", "detection"],
        }
    ],
}


def generate_fixtures(out: Path, seed: int, write_baseline: bool) -> None:
    out.mkdir(parents=True, exist_ok=True)
    bags = out / "bags"
    bags.mkdir(exist_ok=True)
    write_scenario_bag(bags / "nominal.mcap", degraded=False, seed=seed)
    write_scenario_bag(bags / "degraded.mcap", degraded=True, seed=seed)
    (out / "suite.yaml").write_text(
        __import__("yaml").dump(DEFAULT_SUITE, sort_keys=False), encoding="utf-8"
    )
    (out / "suite_degraded.yaml").write_text(
        __import__("yaml").dump(DEGRADED_SUITE, sort_keys=False), encoding="utf-8"
    )
    suite = load_suite(out / "suite.yaml")
    results = evaluate_suite(suite)
    if write_baseline:
        baseline_dir = out / "baselines"
        baseline_dir.mkdir(exist_ok=True)
        write_results(baseline_dir / "suite.json", results)
