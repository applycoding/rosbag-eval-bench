# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from pathlib import Path
from typing import Any

from mcap.writer import CompressionType
from mcap_ros2.writer import Writer

from rosbag_eval_bench.msg_closure import (
    schema_detection_array,
    schema_occupancy,
    schema_odometry,
    schema_path,
    schema_pose_stamped,
)


class McapFixtureWriter:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._file = path.open("wb")
        self.writer = Writer(
            self._file,
            compression=CompressionType.NONE,
            enable_crcs=True,
        )
        self._schemas: dict[str, Any] = {}
        self._seq = 0

    def close(self) -> None:
        self.writer.finish()
        self._file.close()

    def _schema(self, datatype: str, text_fn) -> Any:
        if datatype not in self._schemas:
            self._schemas[datatype] = self.writer.register_msgdef(datatype, text_fn())
        return self._schemas[datatype]

    def write_odometry(self, topic: str, msg: dict, log_time: int) -> None:
        schema = self._schema("nav_msgs/Odometry", schema_odometry)
        self.writer.write_message(
            topic, schema, msg, log_time=log_time, publish_time=log_time, sequence=self._next_seq()
        )

    def write_path(self, topic: str, msg: dict, log_time: int) -> None:
        schema = self._schema("nav_msgs/Path", schema_path)
        self.writer.write_message(
            topic, schema, msg, log_time=log_time, publish_time=log_time, sequence=self._next_seq()
        )

    def write_grid(self, topic: str, msg: dict, log_time: int) -> None:
        schema = self._schema("nav_msgs/OccupancyGrid", schema_occupancy)
        self.writer.write_message(
            topic, schema, msg, log_time=log_time, publish_time=log_time, sequence=self._next_seq()
        )

    def write_pose(self, topic: str, msg: dict, log_time: int) -> None:
        schema = self._schema("geometry_msgs/PoseStamped", schema_pose_stamped)
        self.writer.write_message(
            topic, schema, msg, log_time=log_time, publish_time=log_time, sequence=self._next_seq()
        )

    def write_detections(self, topic: str, msg: dict, log_time: int) -> None:
        schema = self._schema("vision_msgs/Detection2DArray", schema_detection_array)
        self.writer.write_message(
            topic, schema, msg, log_time=log_time, publish_time=log_time, sequence=self._next_seq()
        )

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq
