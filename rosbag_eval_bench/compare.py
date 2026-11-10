# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import math
from typing import Any

from rosbag_eval_bench.schema import Thresholds
from rosbag_eval_bench.types import CompareResult, CompareRow

GATED_METRICS: dict[str, tuple[str, str]] = {
    "path_length": ("lower", "m"),
    "clearance": ("higher", "m"),
    "success": ("higher", ""),
    "time_to_goal": ("lower", "s"),
    "smoothness": ("lower", "rad/m"),
    "ate_rmse": ("lower", "m"),
    "rpe_trans_rmse": ("lower", "m"),
    "rpe_rot_rmse": ("lower", "deg"),
    "map": ("higher", ""),
    "map_50": ("higher", ""),
    "map_75": ("higher", ""),
}


def _is_invalid_number(v: Any) -> bool:
    if v is None:
        return False
    if not isinstance(v, (int, float)):
        return True
    f = float(v)
    return not math.isfinite(f)


def compare_runs(
    baseline: dict[str, Any],
    candidate: dict[str, Any],
    thresholds: Thresholds | None = None,
) -> CompareResult:
    thr = thresholds or Thresholds()
    if baseline.get("metric_version") != candidate.get("metric_version"):
        return CompareResult(False, [], True, "metric_version mismatch")
    if baseline.get("gate_policy_version") != candidate.get("gate_policy_version"):
        return CompareResult(False, [], True, "gate_policy_version mismatch")
    if baseline.get("suite") != candidate.get("suite"):
        return CompareResult(False, [], True, "suite name mismatch")

    base_runs = {r["name"]: r for r in baseline.get("runs", [])}
    cand_runs = {r["name"]: r for r in candidate.get("runs", [])}
    if set(base_runs) != set(cand_runs):
        return CompareResult(False, [], True, "run name mismatch")

    rows: list[CompareRow] = []
    any_fail = False
    for name in sorted(base_runs):
        br = base_runs[name]
        cr = cand_runs[name]
        b_metrics = br.get("metrics", {})
        c_metrics = cr.get("metrics", {})
        for metric_name, (direction, _unit) in GATED_METRICS.items():
            if metric_name not in b_metrics and metric_name not in c_metrics:
                continue
            if metric_name not in b_metrics or metric_name not in c_metrics:
                return CompareResult(False, [], True, f"missing gated key {metric_name} on {name}")
            bv = b_metrics[metric_name]
            cv = c_metrics[metric_name]
            if _is_invalid_number(bv) or _is_invalid_number(cv):
                return CompareResult(False, [], True, f"invalid numeric metric {metric_name}")
            row = _compare_one(metric_name, bv, cv, direction, thr)
            row = CompareRow(
                name=f"{name}.{metric_name}",
                baseline=_norm(bv),
                candidate=_norm(cv),
                delta=row.delta,
                passed=row.passed,
                reason=row.reason,
                gated=True,
            )
            rows.append(row)
            if not row.passed:
                any_fail = True
        for metric_name in sorted(b_metrics):
            if metric_name in GATED_METRICS:
                continue
            bv = b_metrics[metric_name]
            cv = c_metrics.get(metric_name)
            if cv is None:
                return CompareResult(False, [], True, f"missing metric {metric_name}")
            if _is_invalid_number(bv) or _is_invalid_number(cv):
                return CompareResult(False, [], True, f"invalid metric {metric_name}")
            delta = None if bv is None or cv is None else float(cv) - float(bv)
            rows.append(
                CompareRow(
                    name=f"{name}.{metric_name}",
                    baseline=_norm(bv),
                    candidate=_norm(cv),
                    delta=delta,
                    passed=True,
                    reason="reported only",
                    gated=False,
                )
            )

    return CompareResult(passed=not any_fail, rows=rows, invalid=False)


def _norm(v: Any) -> float | None:
    if v is None:
        return None
    f = float(v)
    if f == 0.0:
        return 0.0
    return f


def _compare_one(
    name: str,
    baseline: Any,
    candidate: Any,
    direction: str,
    thr: Thresholds,
) -> CompareRow:
    bv = baseline
    cv = candidate
    if cv is None and bv is not None:
        return CompareRow(name, _norm(bv), None, None, False, "candidate null vs finite baseline", True)
    if cv is not None and bv is None:
        return CompareRow(name, None, _norm(cv), None, True, "newly available", True)
    if cv is None and bv is None:
        return CompareRow(name, None, None, None, True, "both null", True)
    bf = float(bv)
    cf = float(cv)
    delta = cf - bf
    fail = False
    reason = "PASS"
    if name == "path_length":
        fail = cf > bf + thr.path_length_allowance_m
        reason = "regression" if fail else "PASS"
    elif name == "clearance":
        fail = cf < bf - thr.clearance_allowance_m
    elif name == "success":
        fail = cf < bf
    elif name == "time_to_goal":
        fail = (cv is None and bv is not None) or (
            cv is not None and bv is not None and cf > bf + thr.time_to_goal_allowance_s
        )
    elif name == "smoothness":
        fail = (cv is None and bv is not None) or (
            cv is not None and bv is not None and cf > bf + thr.smoothness_allowance_rad_m
        )
    elif name == "ate_rmse":
        fail = cf > bf + thr.ate_rmse_allowance_m
    elif name == "rpe_trans_rmse":
        fail = cf > bf + thr.rpe_trans_allowance_m
    elif name == "rpe_rot_rmse":
        fail = cf > bf + thr.rpe_rot_allowance_deg
    elif name in ("map", "map_50", "map_75"):
        fail = cf < bf - thr.map_allowance
    elif direction == "lower":
        fail = cf > bf
    else:
        fail = cf < bf
    return CompareRow(name, _norm(bv), _norm(cv), delta, not fail, reason, True)
