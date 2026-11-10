# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from rosbag_eval_bench.reader import McapRos2BagReader

FIX = Path(__file__).resolve().parents[1] / "fixtures" / "bags" / "nominal.mcap"
DIGEST_PATH = Path(__file__).resolve().parents[1] / "fixtures" / "bags" / "nominal.semantic.json"


def _digest(path: Path) -> str:
    rows = []
    for topic, t_ns, msg in McapRos2BagReader(path).iter_messages():
        rows.append({"topic": topic, "log_time_ns": t_ns, "type": type(msg).__name__})
    blob = json.dumps(rows, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode()).hexdigest()


def test_semantic_digest_matches_committed():
    assert FIX.exists()
    assert DIGEST_PATH.exists(), "committed semantic digest is missing"
    expected = json.loads(DIGEST_PATH.read_text(encoding="utf-8"))["sha256"]
    assert _digest(FIX) == expected
