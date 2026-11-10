# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
from typing import Any, Protocol

from mcap_ros2.reader import read_ros2_messages

from rosbag_eval_bench.types import EvalError


class BagReader(Protocol):
    def iter_messages(self) -> Iterator[tuple[str, int, Any]]:
        ...


class McapRos2BagReader:
    def __init__(self, path: Path) -> None:
        self.path = path

    def iter_messages(self) -> Iterator[tuple[str, int, Any]]:
        with self.path.open("rb") as f:
            for msg in read_ros2_messages(f):
                yield msg.channel.topic, msg.log_time_ns, msg.ros_msg


def check_topic_monotonic(
    records: list[tuple[int, Any]],
) -> None:
    """Check log_time_ns is strictly increasing in file order."""
    last: int | None = None
    for t_ns, _ in records:
        if last is not None and t_ns <= last:
            raise MonotonicError(f"non-monotonic log_time_ns: {t_ns} after {last}")
        last = t_ns


class MonotonicError(EvalError):
    pass
