# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from collections import defaultdict
from pathlib import Path

from rosbag_eval_bench.decode import (
    decode_detection_array,
    decode_grid,
    decode_odometry,
    decode_path,
    decode_pose_stamped,
)
from rosbag_eval_bench.reader import McapRos2BagReader, check_topic_monotonic
from rosbag_eval_bench.schema import RunConfig, resolve_topics
from rosbag_eval_bench.types import DetectionFrame, EvalError, Path2D, Pose2D


class BagData:
    def __init__(self) -> None:
        self.gt_poses: list[Pose2D] = []
        self.cand_poses: list[Pose2D] = []
        self.cand_path: Path2D | None = None
        self.gt_path: Path2D | None = None
        self.goal: Pose2D | None = None
        self.grid = None
        self.gt_detections: list[DetectionFrame] = []
        self.cand_detections: list[DetectionFrame] = []


def load_run_bag(run: RunConfig) -> BagData:
    topics = resolve_topics(run)
    path = Path(run.bag)
    if not path.exists():
        raise EvalError(f"bag not found: {path}")
    by_topic: dict[str, list[tuple[int, object]]] = defaultdict(list)
    reader = McapRos2BagReader(path)
    for topic, t_ns, msg in reader.iter_messages():
        by_topic[topic].append((t_ns, msg))

    data = BagData()
    fam = set(run.metrics)

    if "localization" in fam:
        _require(by_topic, topics["gt_pose"])
        _require(by_topic, topics["candidate_pose"])
        check_topic_monotonic(by_topic[topics["gt_pose"]])
        check_topic_monotonic(by_topic[topics["candidate_pose"]])
        for t_ns, msg in by_topic[topics["gt_pose"]]:
            data.gt_poses.append(decode_odometry(msg, t_ns))
        for t_ns, msg in by_topic[topics["candidate_pose"]]:
            data.cand_poses.append(decode_odometry(msg, t_ns))

    if "planning" in fam:
        _require(by_topic, topics["candidate_path"])
        _require(by_topic, topics["gt_goal"])
        _require(by_topic, topics["gt_map"])
        if topics["gt_path"] in by_topic:
            records = by_topic[topics["gt_path"]]
            if records:
                data.gt_path = decode_path(records[-1][1], records[-1][0])
        path_records = by_topic[topics["candidate_path"]]
        if not path_records:
            raise EvalError("missing candidate path")
        data.cand_path = decode_path(path_records[-1][1], path_records[-1][0])
        goal_records = by_topic[topics["gt_goal"]]
        data.goal = decode_pose_stamped(goal_records[-1][1], goal_records[-1][0])
        grid_records = by_topic[topics["gt_map"]]
        data.grid = decode_grid(grid_records[-1][1])

    if "detection" in fam:
        _require(by_topic, topics["gt_detections"])
        _require(by_topic, topics["candidate_detections"])
        check_topic_monotonic(by_topic[topics["gt_detections"]])
        check_topic_monotonic(by_topic[topics["candidate_detections"]])
        for t_ns, msg in by_topic[topics["gt_detections"]]:
            data.gt_detections.append(decode_detection_array(msg, t_ns))
        for t_ns, msg in by_topic[topics["candidate_detections"]]:
            data.cand_detections.append(decode_detection_array(msg, t_ns))

    return data


def _require(by_topic: dict, topic: str) -> None:
    if topic not in by_topic or not by_topic[topic]:
        raise EvalError(f"missing required topic {topic}")
