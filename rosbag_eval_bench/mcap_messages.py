# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from rosbag_eval_bench.types import Detection, Grid, Pose2D, quat_from_yaw


def header(stamp_ns: int, frame_id: str = "map") -> dict:
    sec = stamp_ns // 1_000_000_000
    nanosec = stamp_ns % 1_000_000_000
    return {"stamp": {"sec": int(sec), "nanosec": int(nanosec)}, "frame_id": frame_id}


def odometry(pose: Pose2D, child_frame: str = "base_link") -> dict:
    qx, qy, qz, qw = quat_from_yaw(pose.yaw)
    return {
        "header": header(pose.t_ns),
        "child_frame_id": child_frame,
        "pose": {
            "pose": {
                "position": {"x": pose.x, "y": pose.y, "z": 0.0},
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
            },
            "covariance": [0.0] * 36,
        },
        "twist": {
            "twist": {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            "covariance": [0.0] * 36,
        },
    }


def pose_stamped(pose: Pose2D, frame_id: str = "map") -> dict:
    qx, qy, qz, qw = quat_from_yaw(pose.yaw)
    return {
        "header": header(pose.t_ns, frame_id),
        "pose": {
            "position": {"x": pose.x, "y": pose.y, "z": 0.0},
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        },
    }


def path_msg(poses: list[Pose2D], log_ns: int) -> dict:
    return {
        "header": header(log_ns),
        "poses": [pose_stamped(p) for p in poses],
    }


def occupancy_grid(grid: Grid, stamp_ns: int) -> dict:
    qx, qy, qz, qw = quat_from_yaw(grid.origin_yaw)
    return {
        "header": header(stamp_ns),
        "info": {
            "map_load_time": {"sec": 0, "nanosec": 0},
            "resolution": grid.resolution,
            "width": grid.width,
            "height": grid.height,
            "origin": {
                "position": {"x": grid.origin_x, "y": grid.origin_y, "z": 0.0},
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
            },
        },
        "data": list(grid.data),
    }


def detection_array(dets: list[Detection], t_ns: int) -> dict:
    detections = []
    for i, d in enumerate(dets):
        cx = (d.x1 + d.x2) / 2.0
        cy = (d.y1 + d.y2) / 2.0
        sx = d.x2 - d.x1
        sy = d.y2 - d.y1
        detections.append(
            {
                "header": header(t_ns, "camera"),
                "results": [
                    {
                        "hypothesis": {"class_id": d.class_id, "score": d.score},
                        "pose": {
                            "pose": {
                                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                            },
                            "covariance": [0.0] * 36,
                        },
                    }
                ],
                "bbox": {
                    "center": {
                        "position": {"x": cx, "y": cy},
                        "theta": 0.0,
                    },
                    "size_x": sx,
                    "size_y": sy,
                },
                "id": str(i),
            }
        )
    return {"header": header(t_ns, "camera"), "detections": detections}
