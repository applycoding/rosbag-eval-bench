# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from rosbag_eval_bench.msg_closure import load_msg_text


def test_pose2d_nested():
    text = load_msg_text("vision_msgs/Pose2D")
    assert "position" in text
    assert "theta" in text
    assert "float64 x" not in text
