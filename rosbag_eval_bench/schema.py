# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class Thresholds(BaseModel):
    model_config = ConfigDict(extra="forbid")

    path_length_allowance_m: float = 0.50
    clearance_allowance_m: float = 0.05
    time_to_goal_allowance_s: float = 1.0
    smoothness_allowance_rad_m: float = 0.05
    ate_rmse_allowance_m: float = 0.05
    rpe_trans_allowance_m: float = 0.05
    rpe_rot_allowance_deg: float = 1.0
    map_allowance: float = 0.05


class RunConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    name: str
    bag: str
    metrics: list[Literal["planning", "localization", "detection"]]
    topics: dict[str, str] = Field(default_factory=dict)
    robot_radius: float = 0.25
    goal_tol_xy: float = 0.50
    goal_tol_yaw: float = 0.25
    max_dt_ns: int = 50_000_000
    min_match_ratio: float = 0.90
    rpe_delta_s: float = 1.0
    min_rpe_pairs: int = 3


class SuiteConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    name: str
    metric_version: int = 1
    gate_policy_version: int = 1
    runs: list[RunConfig]
    thresholds: Thresholds = Field(default_factory=Thresholds)

    @field_validator("runs")
    @classmethod
    def nonempty_runs(cls, v: list[RunConfig]) -> list[RunConfig]:
        if not v:
            raise ValueError("runs must not be empty")
        return v

    @model_validator(mode="after")
    def unique_names(self) -> SuiteConfig:
        names = [r.name for r in self.runs]
        if len(names) != len(set(names)):
            raise ValueError("duplicate run names")
        return self


def load_suite(path: Path) -> SuiteConfig:
    import yaml

    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise EvalInvalidError("suite YAML root must be a mapping")
    try:
        suite = SuiteConfig.model_validate(raw)
    except Exception as exc:
        raise EvalInvalidError(f"invalid suite: {exc}") from exc
    base = path.parent
    for run in suite.runs:
        bag_path = Path(run.bag)
        if not bag_path.is_absolute():
            run.bag = str((base / bag_path).resolve())
    return suite


class EvalInvalidError(Exception):
    def __init__(self, message: str) -> None:
        super().__init__(message)
        self.message = message


def default_topics() -> dict[str, str]:
    return {
        "gt_pose": "/gt/pose",
        "candidate_pose": "/candidate/pose",
        "candidate_path": "/candidate/path",
        "gt_goal": "/gt/goal",
        "gt_map": "/gt/map",
        "gt_detections": "/gt/detections",
        "candidate_detections": "/candidate/detections",
        "gt_path": "/gt/path",
    }


def resolve_topics(run: RunConfig) -> dict[str, str]:
    base = default_topics()
    base.update(run.topics)
    return base
