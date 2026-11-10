# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from rosbag_eval_bench.compare import compare_runs


def _run(metrics: dict) -> dict:
    return {
        "metric_version": 1,
        "gate_policy_version": 1,
        "suite": "s",
        "runs": [{"name": "r", "metrics": metrics}],
    }


def test_path_length_boundary():
    base = _run({"path_length": 4.0})
    ok = compare_runs(base, _run({"path_length": 4.5}))
    assert ok.passed
    bad = compare_runs(base, _run({"path_length": 4.5 + 1e-6}))
    assert not bad.passed
    improve = compare_runs(base, _run({"path_length": 3.5}))
    assert improve.passed
