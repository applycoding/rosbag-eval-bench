# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from rosbag_eval_bench.decode import (
    decode_detection_array,
    decode_grid,
    decode_odometry,
    decode_path,
)
from rosbag_eval_bench.generate import write_scenario_bag
from rosbag_eval_bench.reader import McapRos2BagReader

FIX = Path(__file__).resolve().parents[1] / "fixtures" / "bags" / "nominal.mcap"


def test_round_trip_topics(tmp_path: Path):
    p = tmp_path / "t.mcap"
    write_scenario_bag(p, degraded=False, seed=1)
    topics = set()
    for topic, t_ns, msg in McapRos2BagReader(p).iter_messages():
        topics.add(topic)
        if topic == "/gt/pose":
            decode_odometry(msg, t_ns)
        elif topic == "/candidate/path":
            decode_path(msg, t_ns)
        elif topic == "/gt/map":
            decode_grid(msg)
        elif topic == "/gt/detections":
            decode_detection_array(msg, t_ns)
    assert "/gt/pose" in topics
    assert "/candidate/path" in topics
    assert "/gt/map" in topics
    assert "/gt/detections" in topics


def test_deterministic_bytes(tmp_path: Path):
    a = tmp_path / "a.mcap"
    b = tmp_path / "b.mcap"
    write_scenario_bag(a, degraded=False, seed=99)
    write_scenario_bag(b, degraded=False, seed=99)
    assert hashlib.sha256(a.read_bytes()).hexdigest() == hashlib.sha256(b.read_bytes()).hexdigest()


@pytest.mark.skipif(not FIX.exists(), reason="fixtures not built")
def test_committed_bag_matches_generator(tmp_path: Path):
    regen = tmp_path / "regen.mcap"
    write_scenario_bag(regen, degraded=False, seed=20261110)
    assert hashlib.sha256(FIX.read_bytes()).hexdigest() == hashlib.sha256(regen.read_bytes()).hexdigest()
