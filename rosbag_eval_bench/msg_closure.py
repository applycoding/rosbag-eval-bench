# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from functools import lru_cache
from pathlib import Path

MSG_ROOT = Path(__file__).resolve().parent / "msgs"


@lru_cache(maxsize=128)
def load_msg_text(relative: str) -> str:
    pkg, name = relative.split("/", 1)
    path = MSG_ROOT / pkg / f"{name}.msg"
    if not path.exists():
        raise FileNotFoundError(path)
    return path.read_text(encoding="utf-8")


def _parse_fields(text: str, pkg: str) -> list[str]:
    deps: list[str] = []
    for line in text.splitlines():
        stripped = line.split("#", 1)[0].strip()
        if not stripped or stripped.startswith("="):
            continue
        tokens = stripped.split()
        if not tokens:
            continue
        field_type = tokens[0].rstrip("[]")
        if field_type in (
            "bool",
            "byte",
            "char",
            "float32",
            "float64",
            "int8",
            "uint8",
            "int16",
            "uint16",
            "int32",
            "uint32",
            "int64",
            "uint64",
            "string",
            "wstring",
        ):
            continue
        if "[" in tokens[0]:
            base = field_type
            if "/" in base:
                deps.append(base)
            elif base[0].isupper():
                deps.append(f"{pkg}/{base}")
            continue
        if "/" in field_type:
            deps.append(field_type)
        elif field_type[0].isupper():
            deps.append(f"{pkg}/{field_type}")
    return deps


def ros2msg_closure(root_type: str) -> str:
    queue = [root_type]
    seen: set[str] = set()
    ordered: list[str] = []

    while queue:
        t = queue.pop(0)
        if t in seen:
            continue
        seen.add(t)
        ordered.append(t)
        pkg, _ = t.split("/", 1)
        text = load_msg_text(t)
        for dep in _parse_fields(text, pkg):
            if dep not in seen:
                queue.append(dep)

    # dependencies first, root last (writer wants root first actually - _for_each_msgdef splits by ===)
    # Order: dependencies before use — put root at end, emit reversed topological
    parts: list[str] = []
    for t in ordered:
        body = load_msg_text(t).strip()
        parts.append(f"MSG: {t}\n{body}")

    return "\n====\n".join(parts)


def schema_odometry() -> str:
    return ros2msg_closure("nav_msgs/Odometry")


def schema_path() -> str:
    return ros2msg_closure("nav_msgs/Path")


def schema_occupancy() -> str:
    return ros2msg_closure("nav_msgs/OccupancyGrid")


def schema_pose_stamped() -> str:
    return ros2msg_closure("geometry_msgs/PoseStamped")


def schema_detection_array() -> str:
    return ros2msg_closure("vision_msgs/Detection2DArray")
