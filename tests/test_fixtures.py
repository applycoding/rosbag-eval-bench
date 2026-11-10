# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_suite_eval_exit_zero():
    proc = subprocess.run(
        [
            sys.executable,
            "-m",
            "rosbag_eval_bench.cli",
            "eval",
            "--suite",
            str(ROOT / "fixtures/suite.yaml"),
            "--baseline",
            str(ROOT / "fixtures/baselines/suite.json"),
            "--out",
            str(ROOT / "reports/pytest_nominal"),
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr


def test_degraded_exit_one():
    proc = subprocess.run(
        [
            sys.executable,
            "-m",
            "rosbag_eval_bench.cli",
            "eval",
            "--suite",
            str(ROOT / "fixtures/suite_degraded.yaml"),
            "--baseline",
            str(ROOT / "fixtures/baselines/suite.json"),
            "--out",
            str(ROOT / "reports/pytest_degraded"),
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 1, proc.stderr
    data = json.loads((ROOT / "reports/pytest_degraded/results.json").read_text())
    fails = {r["name"] for r in data["compare_rows"] if not r["passed"] and r.get("gated")}
    assert "full_stack.ate_rmse" in fails
    assert "full_stack.path_length" in fails
    assert "full_stack.map" in fails


def test_bag_size_under_1mb():
    for p in (ROOT / "fixtures/bags").glob("*.mcap"):
        assert p.stat().st_size < 1_048_576

def test_identical_replay_matches_baseline(tmp_path: Path):
    """Criterion 6: re-evaluating the nominal suite matches stored baseline floats."""
    out = tmp_path / "replay"
    proc = subprocess.run(
        [
            sys.executable,
            "-m",
            "rosbag_eval_bench.cli",
            "eval",
            "--suite",
            str(ROOT / "fixtures/suite.yaml"),
            "--baseline",
            str(ROOT / "fixtures/baselines/suite.json"),
            "--out",
            str(out),
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr
    baseline = json.loads((ROOT / "fixtures/baselines/suite.json").read_text(encoding="utf-8"))
    candidate = json.loads((out / "results.json").read_text(encoding="utf-8"))
    base_runs = {r["name"]: r for r in baseline["runs"]}
    cand_runs = {r["name"]: r for r in candidate["runs"]}
    assert set(base_runs) == set(cand_runs)
    mismatches = []
    for name, br in base_runs.items():
        cr = cand_runs[name]
        for metric, bv in br["metrics"].items():
            assert metric in cr["metrics"], f"missing metric {metric}"
            cv = cr["metrics"][metric]
            if bv is None or cv is None:
                if bv is not cv:
                    mismatches.append((name, metric, bv, cv))
                continue
            if abs(float(cv) - float(bv)) >= 1e-12:
                mismatches.append((name, metric, bv, cv))
    assert mismatches == [], mismatches
    gated = [r for r in candidate["compare_rows"] if r.get("gated")]
    assert gated, "expected gated compare rows"
    failed = [r["name"] for r in gated if not r["passed"]]
    assert failed == [], failed
