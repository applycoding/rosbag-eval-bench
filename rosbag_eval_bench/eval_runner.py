# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from rosbag_eval_bench.associate import (
    associate_detection_frames,
    associate_poses,
    require_match_ratio,
)
from rosbag_eval_bench.bag_loader import load_run_bag
from rosbag_eval_bench.compare import compare_runs
from rosbag_eval_bench.metrics_det import detection
from rosbag_eval_bench.metrics_plan import planning
from rosbag_eval_bench.metrics_traj import ate_rpe
from rosbag_eval_bench.schema import RunConfig, SuiteConfig, Thresholds
from rosbag_eval_bench.types import CompareResult, EvalError

METRIC_VERSION = 1
GATE_POLICY_VERSION = 1


def evaluate_run(run: RunConfig) -> dict[str, Any]:
    data = load_run_bag(run)
    metrics: dict[str, Any] = {}
    series: dict[str, Any] = {}

    if "localization" in run.metrics:
        mp = associate_poses(data.gt_poses, data.cand_poses, run.max_dt_ns)
        require_match_ratio(mp, run.min_match_ratio)
        tm = ate_rpe(mp, run.rpe_delta_s, run.min_rpe_pairs)
        metrics.update(
            {
                "ate_unaligned_rmse": tm.ate_unaligned_rmse,
                "ate_rmse": tm.ate_rmse,
                "ate_rot_rmse": tm.ate_rot_rmse,
                "rpe_trans_rmse": tm.rpe_trans_rmse,
                "rpe_rot_rmse": tm.rpe_rot_rmse,
                "match_ratio": tm.match_ratio,
            }
        )
        series["ate_series"] = tm.ate_series
        series["alignment"] = {
            "yaw": tm.alignment_yaw,
            "tx": tm.alignment_tx,
            "ty": tm.alignment_ty,
        }

    if "planning" in run.metrics:
        if data.cand_path is None or data.goal is None or data.grid is None:
            raise EvalError("incomplete planning inputs")
        pm = planning(
            data.cand_path,
            data.goal,
            data.grid,
            run.robot_radius,
            run.goal_tol_xy,
            run.goal_tol_yaw,
        )
        metrics.update(
            {
                "path_length": pm.path_length,
                "clearance": pm.clearance,
                "success": pm.success,
                "time_to_goal": pm.time_to_goal,
                "smoothness": pm.smoothness,
            }
        )
        series["path_poses"] = [{"x": p.x, "y": p.y, "t_s": p.t_ns / 1e9} for p in pm.path_poses]
        if data.gt_path:
            series["gt_path_poses"] = [{"x": p.x, "y": p.y} for p in data.gt_path.poses]

    if "detection" in run.metrics:
        pairs = associate_detection_frames(data.gt_detections, data.cand_detections, run.max_dt_ns)
        dm = detection(pairs)
        metrics.update(
            {
                "precision_50": dm.precision_50,
                "recall_50": dm.recall_50,
                "map_50": dm.map_50,
                "map_75": dm.map_75,
                "map": dm.map,
            }
        )

    return {"name": run.name, "metrics": metrics, "series": series}


def evaluate_suite(suite: SuiteConfig) -> dict[str, Any]:
    runs_out = []
    for run in suite.runs:
        runs_out.append(evaluate_run(run))
    return {
        "metric_version": METRIC_VERSION,
        "gate_policy_version": GATE_POLICY_VERSION,
        "suite": suite.name,
        "thresholds": suite.thresholds.model_dump(),
        "runs": runs_out,
    }


def load_baseline(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def compare_to_baseline(
    candidate: dict[str, Any],
    baseline: dict[str, Any],
    thresholds: Thresholds | None = None,
) -> tuple[dict[str, Any], CompareResult]:
    result = compare_runs(baseline, candidate, thresholds)
    rows = [
        {
            "name": r.name,
            "baseline": r.baseline,
            "candidate": r.candidate,
            "delta": r.delta,
            "passed": r.passed,
            "reason": r.reason,
            "gated": r.gated,
        }
        for r in result.rows
    ]
    candidate["compare_rows"] = rows
    candidate["comparison_passed"] = result.passed
    candidate["comparison_invalid"] = result.invalid
    if result.invalid_reason:
        candidate["comparison_invalid_reason"] = result.invalid_reason
    return candidate, result


def canonicalize_json(obj: Any) -> Any:
    """Normalize -0.0 to 0.0 for metrics."""
    if isinstance(obj, dict):
        return {k: canonicalize_json(v) for k, v in sorted(obj.items())}
    if isinstance(obj, list):
        return [canonicalize_json(v) for v in obj]
    if isinstance(obj, float):
        if not math.isfinite(obj):
            raise EvalError("non-finite metric")
        if obj == 0.0:
            return 0.0
        return obj
    return obj


def write_results(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = canonicalize_json(data)
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
