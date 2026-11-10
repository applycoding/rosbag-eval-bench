# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from rosbag_eval_bench.eval_runner import (
    compare_to_baseline,
    evaluate_suite,
    load_baseline,
    write_results,
)
from rosbag_eval_bench.generate import generate_fixtures
from rosbag_eval_bench.report import write_figures, write_html, write_markdown
from rosbag_eval_bench.schema import EvalInvalidError, load_suite
from rosbag_eval_bench.types import EvalError


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="reb", description="rosbag-eval-bench CLI")
    sub = parser.add_subparsers(dest="command", required=True)

    p_gen = sub.add_parser("generate-fixtures", help="Write synthetic MCAP fixtures")
    p_gen.add_argument("--seed", type=int, default=20261110)
    p_gen.add_argument("--out", type=Path, default=Path("fixtures"))
    p_gen.add_argument("--write-baseline", action="store_true")

    p_eval = sub.add_parser("eval", help="Evaluate a suite against a baseline")
    p_eval.add_argument("--suite", type=Path, required=True)
    p_eval.add_argument("--baseline", type=Path, required=True)
    p_eval.add_argument("--out", type=Path, default=Path("reports"))

    p_cmp = sub.add_parser("compare", help="Compare two result JSON files")
    p_cmp.add_argument("--baseline", type=Path, required=True)
    p_cmp.add_argument("--candidate", type=Path, required=True)
    p_cmp.add_argument("--out", type=Path, default=Path("reports"))

    args = parser.parse_args(argv)
    try:
        if args.command == "generate-fixtures":
            generate_fixtures(args.out, args.seed, args.write_baseline)
            return 0
        if args.command == "eval":
            return _cmd_eval(args.suite, args.baseline, args.out)
        if args.command == "compare":
            return _cmd_compare(args.baseline, args.candidate, args.out)
    except (EvalError, EvalInvalidError) as exc:
        print(exc.message if hasattr(exc, "message") else str(exc), file=sys.stderr)
        return 2
    return 2


def _cmd_eval(suite_path: Path, baseline_path: Path, out: Path) -> int:
    suite = load_suite(suite_path)
    baseline = load_baseline(baseline_path)
    candidate = evaluate_suite(suite)
    candidate, cmp = compare_to_baseline(candidate, baseline, suite.thresholds)
    out.mkdir(parents=True, exist_ok=True)
    write_results(out / "results.json", candidate)
    write_markdown(candidate, out / "report.md")
    fig_dir = out / "figures"
    write_figures(candidate, fig_dir)
    write_html(candidate, out / "report.html", fig_dir)
    if cmp.invalid:
        return 2
    return 1 if not cmp.passed else 0


def _cmd_compare(baseline_path: Path, candidate_path: Path, out: Path) -> int:
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    candidate, cmp = compare_to_baseline(candidate, baseline)
    out.mkdir(parents=True, exist_ok=True)
    write_results(out / "results.json", candidate)
    write_markdown(candidate, out / "report.md")
    fig_dir = out / "figures"
    write_figures(candidate, fig_dir)
    write_html(candidate, out / "report.html", fig_dir)
    if cmp.invalid:
        return 2
    return 1 if not cmp.passed else 0


if __name__ == "__main__":
    raise SystemExit(main())
