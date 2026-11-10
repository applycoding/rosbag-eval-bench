# References

Permissively licensed GitHub projects used as **references only**. Do not copy source. Credit any conceptually derived idea in the README. Licenses were checked on 2026-11-10 with `gh api repos/OWNER/REPO --jq .license.spdx_id`, and with the raw LICENSE, package.xml, or LICENSE.md when GitHub reported `NOASSERTION`. All listed URLs returned HTTP 200 on 2026-11-10.

**Outbound license for this demo repo is Apache-2.0**, not MIT. A LICENSE file with the official Apache-2.0 text, a NOTICE file crediting Arpan Bom, and SPDX-License-Identifier: Apache-2.0 headers are required at build time on *authored* files.

This demo must not duplicate existing applycoding repos (list checked 2026-11-10 with `gh repo list applycoding --limit 200`; `applycoding/rosbag-eval-bench` does not exist):

| Repo | What it already is | How this demo differs |
| --- | --- | --- |
| [applycoding/llm-eval-observability](https://github.com/applycoding/llm-eval-observability), [applycoding/rag-llm-eval-demo](https://github.com/applycoding/rag-llm-eval-demo) | LLM / RAG evaluation and observability | Robot MCAP bags; planning, localization, and detection metrics. |
| [applycoding/teleop-console-demo](https://github.com/applycoding/teleop-console-demo) | Browser teleop with record/replay of control messages | Offline bag evaluation against ground truth, not a teleop console. |
| [applycoding/ros2-nav2-slam-sim](https://github.com/applycoding/ros2-nav2-slam-sim) | Gazebo + slam_toolbox + Nav2 simulation | Offline eval bench, no Gazebo, no live Nav2. |
| [applycoding/search-ranking-recsys](https://github.com/applycoding/search-ranking-recsys) | NDCG / MRR ranking evals | Different domain. |
| [applycoding/nav2-coverage-planner-compare](https://github.com/applycoding/nav2-coverage-planner-compare) | Planner comparison in simulation | Bag replay and CI regression versus a stored baseline, not a planner implementation. |
| [applycoding/sim-gimbal-stabilizer](https://github.com/applycoding/sim-gimbal-stabilizer), ekf-gnss-localization (in planning) | Estimators / controllers | This repo is a metric harness, not an estimator. |

## Selected references (8)

### 1. foxglove/mcap

- URL: https://github.com/foxglove/mcap
- License: **MIT** (`gh api` SPDX: MIT; root LICENSE is the MIT text)
- Docs (HTTP 200 on 2026-11-10): https://mcap.dev and https://mcap.dev/guides/python/ros2
- PyPI, verified 2026-11-10: `mcap==1.6.0`, `mcap-ros2-support==0.6.0` (MIT classifiers; no ROS 2 dependency)
- Borrow: the MCAP format and the Python reader/writer that work without a ROS install (`read_ros2_messages`, `mcap_ros2.writer.Writer`). Use the public APIs; do not copy examples beyond ordinary use of the library (this is a declared runtime dependency, not a vendored fork). Prefer `log_time_ns` over the datetime `log_time` property.

### 2. ros2/rosbag2

- URL: https://github.com/ros2/rosbag2
- License: **Apache-2.0** (`gh api` SPDX: Apache-2.0)
- LICENSE: https://github.com/ros2/rosbag2/blob/master/LICENSE
- Borrow: the idea of an optional ROS 2 Humble reader path (`rosbag2_py`, MCAP storage plugin). Do not vendor the package. The isolated Humble smoke may *use* the distro packages; it must not copy rosbag2 sources into this tree.

### 3. uzh-rpg/rpg_trajectory_evaluation

- URL: https://github.com/uzh-rpg/rpg_trajectory_evaluation
- License: **MIT** (`gh api` SPDX: MIT; LICENSE file checked, copyright 2018 Zichao Zhang, Robotics and Perception Group, University of Zurich)
- LICENSE: https://github.com/uzh-rpg/rpg_trajectory_evaluation/blob/master/LICENSE
- Paper (idea only, not code): Zichao Zhang and Davide Scaramuzza, “A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry,” IEEE/RSJ IROS 2018, DOI [10.1109/IROS.2018.8593941](https://doi.org/10.1109/iros.2018.8593941). Author PDF (HTTP 200, 2026-11-10): https://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf
- Also cite: Jürgen Sturm, Nikolas Engelhard, Felix Endres, Wolfram Burgard, and Daniel Cremers, “A Benchmark for the Evaluation of RGB-D SLAM Systems,” IROS 2012, for the time-based ATE / RPE problem statement.
- Borrow: alignment choices and the ATE vs relative-error distinction. **Honesty:** this demo’s RPE is Sturm-style time-based relative pose error at Δt = 1 s. Zhang and Scaramuzza also discuss KITTI-style distance-based relative error, which we do **not** implement in v1. Write our own Umeyama and RPE; do not copy toolbox code or figures.

### 4. cartographer-project/cartographer

- URL: https://github.com/cartographer-project/cartographer
- License: **Apache-2.0** (`gh api` SPDX: Apache-2.0)
- Borrow: the *idea* of reporting trajectory metrics in a SLAM/evaluation setting. Far too large to vendor. Do not copy evaluation scripts or proto.

### 5. tier4/driving_log_replayer_v2

- URL: https://github.com/tier4/driving_log_replayer_v2
- License: **Apache-2.0** (`gh api` SPDX: Apache-2.0; LICENSE is the Apache 2.0 text)
- Related: [tier4/driving_log_replayer](https://github.com/tier4/driving_log_replayer) is also Apache-2.0 but its README (checked 2026-11-10) states it is **no longer maintained** and points to v2.
- Borrow: the *idea* of a scenario plus a bag plus pass/fail thresholds, in the context of Autoware / TIER IV evaluation. Do not copy scenarios, launch files, metric code, or CLI structure. Do not claim Autoware compatibility or that this bench is a driving_log_replayer substitute.

### 6. ros-navigation/navigation2

- URL: https://github.com/ros-navigation/navigation2
- License: **mixed**. The root LICENSE (checked 2026-11-10) lists LGPL-2.1-or-later, Apache-2.0, BSD-3-Clause, and combinations, and says unmarked files are Apache-2.0. GitHub SPDX is `NOASSERTION`. `nav2_core/package.xml` declares **Apache-2.0**.
- LICENSE: https://github.com/ros-navigation/navigation2/blob/main/LICENSE
- Borrow: planner metric *ideas* (path length, goal tolerance, costmap clearance) from Apache-2.0 packages only, after reading the relevant `package.xml`. **Do not read or translate LGPL-2.1-or-later packages to implement our metrics.** Implement our own polyline-to-square clearance; do not vendor Nav2 costmap code.

### 7. facebookresearch/detectron2 and open-mmlab/mmdetection

- URLs: https://github.com/facebookresearch/detectron2 and https://github.com/open-mmlab/mmdetection
- License: both **Apache-2.0** (`gh api` SPDX: Apache-2.0)
- Borrow: the *idea* of IoU matching and average precision over a set of IoU thresholds. Do not vendor either project, and do not import their dependency trees to “validate AP” (that can pull excluded cocoapi / pycocotools). Our numeric target is all-point interpolated AP (see PLAN.md), **not** official COCO 101-recall-point AP.

### 8. NVIDIA-ISAAC-ROS/isaac_ros_benchmark

- URL: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark
- License: **Apache-2.0** (`gh api` SPDX: Apache-2.0)
- Borrow: the *idea* of a CI job that fails when a bench regresses. **Honesty:** that project is mostly latency and throughput, not planning or perception accuracy regression. Cite it only as a CI-shape idea. Do not copy launch files or benchmark configs.

## Message-definition sources (vendored texts, not metric code)

These are copied as `.msg` texts at the commits in PLAN.md, Apache-2.0, credited in NOTICE, not relicensed:

- [ros2/common_interfaces](https://github.com/ros2/common_interfaces) `humble` @ `08434490d3e4abd37ccc5462cdde69b88853b5b8` — geometry_msgs, nav_msgs, std_msgs
- [ros-perception/vision_msgs](https://github.com/ros-perception/vision_msgs) tag `4.1.1` @ `e77fa2d97d69abaaa546592892d2d5d16cec9977` — Apache-2.0 (`package.xml`); include `Point2D` and nested `Pose2D`
- [ros2/rcl_interfaces](https://github.com/ros2/rcl_interfaces) `humble` @ `82776fc9068d1e0cd4af11c2e2700195994b2eb1` — `builtin_interfaces` 1.2.3, Apache-2.0

## Looked at, not used as primary references or dependencies

- [MichaelGrupp/evo](https://github.com/MichaelGrupp/evo) — **GPL-3.0**. Idea-only. Do not depend on it in runtime, dev, or optional extras. Do not read its sources to implement ATE/RPE. Only repository license metadata was used for this list.
- [cocodataset/cocoapi](https://github.com/cocodataset/cocoapi) / pycocotools — GitHub SPDX `NOASSERTION`; mixed data vs API history. Do not depend on cocoapi or pycocotools in any extra. The all-point AP contract in PLAN.md is specified independently. The 101-recall-point COCO rule was noted in review only as a reason *not* to claim official COCO AP; do not vendor that code.
- [ternaris/rosbags](https://pypi.org/project/rosbags/) (`rosbags` 0.11.9, Apache-2.0 on PyPI) — a fine alternative bag reader. **Not a v1 dependency**; we already use `mcap-ros2-support`.
- Official COCO evaluation site / pycocotools parameter comments — methodology contrast only.

## Credit rule

Credit idea sources and their licenses in the README. If a later change would copy code from a reference, stop and get a license review first. Permissive licenses still require notices when code is redistributed.

## Compatibility with our Apache-2.0 outbound license

MIT, BSD-3-Clause, and Apache-2.0 references can be used as idea sources and, for foxglove/mcap and the official ROS interface packages, as a declared dependency or vendored `.msg` text with attribution. GPL-3.0 (evo) cannot appear in the tree or the lockfile. LGPL Nav2 packages cannot be read to implement metrics. cocoapi / pycocotools stay out of all dependency groups.
