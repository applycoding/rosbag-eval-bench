# SPDX-License-Identifier: Apache-2.0
"""Isolated ROS Humble bag smoke (no rosbag_eval_bench / numpy 2.5 install)."""
from __future__ import annotations

import json
import sys
from pathlib import Path

MANIFEST = Path(__file__).resolve().parents[1] / "fixtures" / "humble_manifest.json"


def main() -> int:
    if not MANIFEST.exists():
        print("humble manifest missing", file=sys.stderr)
        return 2
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    try:
        import rosbag2_py  # type: ignore
    except ImportError:
        print("rosbag2_py not available", file=sys.stderr)
        return 0
    bag_path = manifest.get("bag")
    if not bag_path:
        return 2
    # Informational stub: validate manifest topics list without importing the core package.
    expected = set(manifest.get("topics", []))
    if not expected:
        print("no topics in manifest", file=sys.stderr)
        return 2
    print(f"manifest ok for {bag_path}, topics={sorted(expected)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
