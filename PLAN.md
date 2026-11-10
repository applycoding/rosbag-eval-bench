# rosbag-eval-bench — v1 plan

Checked 2026-11-10. Outbound license is **Apache-2.0**. This document incorporates an independent review of `PLAN_draft.md` and additional verification (pinned-library MCAP round-trip, ATE/IoU/AP/RPE/smoothness oracles, and regression-gate arithmetic). Planning only; no public repo or implementation is created by this document.

## Goal

Public portfolio demo for Arpan Bom: an offline regression bench for robot software. It reads small **synthetic** ROS 2 MCAP bags, scores planning, localization, and detection outputs against ground truth in the same bag, writes an HTML and Markdown report, and fails CI when gated metrics regress past absolute thresholds versus a committed baseline.

TIER IV and Lely have asked for this kind of testing, and 24 tracked roles mention evals. This repo is a portfolio bench on synthetic fixtures. It does not claim that Arpan has shipped Autoware evaluation, TIER IV `driving_log_replayer`, Lely production HIL, or certified robot-software QA.

Commit author and committer are Arpan Bom <applycoding@gmail.com>; no co-author trailers.

## Scope

Python package `rosbag_eval_bench`, console script `reb`. Default path reads and writes MCAP with `mcap==1.6.0` and `mcap-ros2-support==0.6.0` and does not import ROS. An optional Humble adapter (`tools/humble_smoke.py`) uses `rosbag2_py` only, is isolated from the numpy 2.5 / Python 3.12 stack, and does not gate core CI. Jazzy is a later port.

v1 metric families:

- Planning: path length, clearance, success, time to goal, smoothness.
- Localization: ATE (aligned translation RMSE is the gate; unaligned RMSE is reported) and RPE (translation and yaw).
- Detection: precision/recall at IoU 0.5 (reported, not gated), `map_50`, `map_75`, and all-point `map` averaged over IoU 0.50:0.05:0.95.

Output of `reb eval`: `report.md`, `report.html`, `results.json` (metrics, compare rows, and plot series), matplotlib PNGs. GitHub Pages is a Vite + TypeScript app at base `/rosbag-eval-bench/` that displays a sample report and an interactive baseline-vs-candidate view by **loading committed JSON** (no second metric engine in TypeScript).

Differentiate existing applycoding repos (list checked 2026-11-10, `gh repo list applycoding --limit 200`; `applycoding/rosbag-eval-bench` does not exist):

- `llm-eval-observability`, `rag-llm-eval-demo`: LLM/RAG evals.
- `teleop-console-demo`: browser teleop record/replay of control messages.
- `ros2-nav2-slam-sim`: Gazebo + slam_toolbox + Nav2, online sim.
- `search-ranking-recsys`: NDCG/MRR.
- `nav2-coverage-planner-compare`: planner comparison in simulation, no bag replay and no CI gate against a stored baseline.
- `ekf-gnss-localization` (in planning) and `sim-gimbal-stabilizer`: estimators and controllers.

Reimplement Umeyama, ATE/RPE, planning scores, and all-point AP from first principles. Do not copy or vendor evo, driving_log_replayer, Cartographer, Detectron2, Nav2, or pycocotools sources.

## Architecture

**Decision:** a pure-Python metric core plus a thin MCAP adapter. Messages are decoded to internal dataclasses before any metric runs. Comparison policy lives in `compare.py` only. Reports and the Pages app consume the same `results.json` schema, including precomputed `CompareRow`s and path / ATE series. The Pages overlay does not recompute gates.

**Why:** Interview readers can run pytest without a ROS install. One bag per scenario avoids pairing two files. Gating outside the metric functions keeps formulas free of threshold policy. `metric_version: 1` and `gate_policy_version: 1` are stored in every result JSON; a formula or gate change bumps the matching version and regenerates baselines.

### Reader

`BagReader` is a protocol that yields `(topic: str, timestamp_ns: int, message)`. `McapRos2BagReader` calls `mcap_ros2.reader.read_ros2_messages` and **must use `McapROS2Message.log_time_ns`** (int nanoseconds). The helper property `.log_time` is a naive `datetime` in the local timezone with only microsecond resolution; it is **forbidden** in metrics and association (verified 2026-11-10: a stamp of `1_000_000_000` ns appeared as `1969-12-31 17:00:01` under America/Denver). Convert decoded `ros_msg` objects into the internal dataclasses. `publish_time` is written equal to `log_time` by the generator.

`Rosbag2PyBagReader` is used only by the isolated Humble smoke and is never imported by the core package at import time.

Monotonicity is checked on the **file order** of records for each topic (not after the reader's optional log-time sort). Duplicate or non-monotonic `log_time_ns` on a required topic is exit 2.

### Frames and geometry

Planar `map` frame, REP-103-like ENU: +x East, +y North, +z up. Poses are SE(2) body-to-map transforms: `z = 0`, roll = pitch = 0, quaternion is yaw about +z: `(x, y, z, w) = (0, 0, sin(yaw/2), cos(yaw/2))`. Yaw from a unit z-yaw quaternion is `atan2(2*w*z, 1 - 2*z*z)`. `q` and `-q` must give the same yaw. Quaternion norm error `< 1e-3` is normalized; otherwise exit 2. Non-zero roll or pitch (`|position.z|`, or `|orientation.x|` / `|orientation.y|`, above `1e-6`) exits 2.

OccupancyGrid is in `map`. Index is ROS row-major: cell `(i, j)` (i along x, j along y) is `data[j * width + i]`. Cell `(i, j)` covers the half-open square `[origin_x + i*res, origin_x + (i+1)*res) × [origin_y + j*res, origin_y + (j+1)*res)` when origin yaw is 0. **Origin yaw must be 0** in v1; a non-zero `info.origin.orientation` yaw exits 2. Occupied if value `>= 50`, unknown is `-1`, free is `0`. Fixtures use `0 / 100 / -1` (Nav2 / map_server style), **not** the “0/1/-1” comment in the official msg. Resolution in fixtures is `0.05 m`. A non-square grid unit test must fail if width and height are swapped in the index formula.

Detection boxes are axis-aligned in a synthetic 640×480 image. Image convention (vision_msgs `Point2D`): +x right, +y down, distinct from map ENU. The image itself is not stored. `bbox.center.theta` must be `0` on every detection in a detection run; non-zero is exit 2. Boxes may extend outside 640×480; they are **not** clipped.

### Time association

Timestamps stay integer nanoseconds until a ratio is formed.

Poses: an exact timestamp match (`t_est == t_gt`) is always valid, including the first and last samples. Otherwise the candidate must **bracket** `t` with `t_lo < t < t_hi` and `(t - t_lo) <= max_dt` **and** `(t_hi - t) <= max_dt`, where `max_dt = 50_000_000` ns (each side, not a 50 ms total span). This permits a 100 ms bracket and matches 20 Hz fixtures (50 ms period). Interpolate `x` and `y` linearly. Interpolate yaw as `wrap(yaw_lo + alpha * wrap(yaw_hi - yaw_lo))` with `alpha = (t - t_lo) / (t_hi - t_lo)` computed in float after integer subtraction, and `wrap` into `(-π, π]`. If `abs(wrap(yaw_hi - yaw_lo))` is exactly `π` (antipodal, short arc undefined), that GT sample is unmatched. A gap that fails the bracket test is unmatched.

`match_ratio = n_matched / n_gt`, computed before dropping unmatched GT from the arrays. Exactly `0.90` passes. `n_gt == 0` or `match_ratio < 0.90` is exit 2. Report `n_matched`, `n_gt`, and the ratio.

RPE pairs: for each matched GT index `i`, look for a matched GT pose whose timestamp is nearest to `t_i + 1.0 s` within 50 ms, with `j > i`. The corresponding estimate poses are the already-matched estimates at `i` and `j` (a common GT index). Do not invent estimate-only timestamps. Fewer than 3 pairs is exit 2.

Detections: one-to-one nearest-time matching of frames within 50 ms. Sort GT frames by time; assign each the unused candidate frame with smallest `|Δt|` if `<= 50 ms`; ties take the earlier candidate. An unmatched GT frame contributes its boxes as false negatives and **remains in the AP GT count**. An unmatched candidate frame (including those outside the GT time range) contributes its boxes as false positives. An empty `Detection2DArray` is a valid frame. A missing required topic is exit 2, not an empty frame.

### Localization metrics

Correspondences are the matched `(x, y, yaw)` pairs after association. Poses are body-to-map SE(2) transforms.

- **Unaligned ATE:** RMSE of Euclidean translation error, plus mean, median, **population** standard deviation (`sqrt(mean((e - mean(e))²))`, divide by `n`), and max, in meters. Not a gate.
- **Aligned ATE (the `ate_rmse` gate):** rigid SE(2) Umeyama / Horn, **no scale**, transform **estimate → GT**, column vectors. Let `e`, `g` be the 2-D points, `ē`, `ḡ` their means, `H = Σ (e−ē)(g−ḡ)ᵀ`, `H = U S Vᵀ` (SVD), `D = diag(1, det(V Uᵀ))`, `R = V D Uᵀ`, `t = ḡ − R ē`. Apply `R, t` to the estimate positions. Report alignment yaw `atan2(R_10, R_00)` and translation `t`. Rotate estimate headings by that yaw before any reported yaw-error stats. Rotation ATE is reported and is not a gate. Fewer than 3 non-coincident points is exit 2.
- **RPE:** `delta_t = 1.0 s`. `ΔT_gt = inv(T_gt_i) T_gt_j`, `ΔT_est = inv(T_est_i) T_est_j`, `E = inv(ΔT_gt) ΔT_est` in SE(2). Translational error is `hypot(E_02, E_12)` (body-frame relative translation, not a world-frame position subtraction). Rotational error is `wrap(atan2(E_10, E_00))` converted to degrees before RMSE. No alignment. Report `rpe_trans_rmse` (m) and `rpe_rot_rmse` (deg).

Hand oracles (tests must hit these values within `1e-12`):

- GT `(0,0), (1,0), (2,0)` vs est `(0.1,0.1), (1.1,0.1), (2.1,0.1)`: unaligned RMSE `sqrt(0.02) ≈ 0.1414213562373095` m. Estimate-to-GT yaw `0`, translation `(-0.1, -0.1)`, aligned RMSE `0`.
- Same GT vs est `(0,0), (0,1), (0,2)`: estimate-to-GT yaw **−π/2** (not +π/2), translation `0`, aligned RMSE `0`.
- Linear drift (unalignable by a rigid motion): GT `(0,0), (1,0), (2,0)` at t = 0, 1, 2 s vs est `(0,0), (1.15,0), (2.30,0)` (offset `0.15 t` m). After SE(2) alignment, ATE RMSE is `sqrt(0.015) ≈ 0.1224744871391589` m. This is the model for the degraded localization fixture.
- RPE, four poses at t = 0, 1, 2, 3 s (three pairs at Δt = 1 s): identical motion ⇒ trans RMSE `0` and rot RMSE `0`. GT relative `+0.1 m` in x vs estimate relative `+0.2 m` in x ⇒ trans RMSE `0.1`. Identical translations with a relative yaw **error** of `1°` on each pair ⇒ rot RMSE `1`. Two pairs only ⇒ exit 2.

### Planning metrics

Computed on the **candidate** `nav_msgs/Path` against `/gt/goal` and `/gt/map`. `/gt/path`, if present, is for report overlay only and is not required and not scored. Vertices are `PoseStamped`s in `map`. One Path message at episode end; if several Path messages exist, use the last by `log_time_ns`. `time_to_goal` uses the **pose header stamps inside the path**, not the Path message `log_time`. Those stamps are planned times, not proof of a real robot arrival; the README says so.

Clearance is the minimum Euclidean distance from the **polyline** (every vertex and every closed segment) to every occupied cell **square**, not to an inscribed disk. For a point `p` and a cell with center `c` and half-width `h = res/2`:

`dist(p, cell) = hypot(max(|p_x−c_x|−h, 0), max(|p_y−c_y|−h, 0))`

which is `0` on or inside the square. Segment-to-square distance is the min distance from the closed segment to the square (intersection ⇒ `0`). Contact counts as clearance `0` and `collision = true`.

A vertex or segment that leaves the half-open map is exit 2. Unknown cells are **passable** (not obstacles); the README says so. A planning run whose map has no occupied cell is exit 2 (fixture restriction, so clearance stays finite). `width * height` must equal `len(data)`.

- **path_length** `L`: sum of Euclidean segment lengths (m). Zero-length segments are skipped for heading. Lower is better.
- **clearance:** the minimum polyline-to-occupied-square distance, in meters. Lower is worse.
- **success:** `1.0` iff `collision` is false and `clearance >= robot_radius` (`0.25 m`) and some path pose is within `goal_tol_xy = 0.50 m` and `goal_tol_yaw = 0.25 rad` (wrapped yaw error) of `/gt/goal`. Else `0.0`. Footprint interpretation: clearance is centerline-to-obstacle distance; `clearance == robot_radius` just grazes and **passes**.
- **time_to_goal:** stamp of the first pose that meets the goal tolerance, minus the stamp of the first pose (s). Unsuccessful runs store JSON `null`. Lower is better.
- **smoothness:** `(1/L) * sum_i |wrap(θ_{i+1} − θ_i)|` with headings of **positive-length** segments, wrap into `(-π, π]`, unit rad/m. A straight line is `0`. Two legs of 2 m at a right angle are `(π/2)/4`.

Empty or missing path on a planning-enabled run is **exit 2**. A single valid pose is allowed: `L = 0`, smoothness `null`, `time_to_goal = 0` if it meets the goal and clearance rules, else success `0` and `time_to_goal` null. `L < 1e-6` with two or more coincident poses is treated the same as a single pose (smoothness null), not exit 2.

Hand oracles: smoothness L-path `(π/2)/4`; a point on the face of an occupied square at distance `res/2` reports `res/2`; a point just outside a square **corner** at `(c_x+h+ε, c_y+h+ε)` reports ≈ `ε√2`, **not** the inscribed-disk value `(√2 − 0.5)*res + ε√2`. A segment that crosses an occupied cell without a vertex inside it has clearance `0` and `collision` true.

### Detection metrics

Internal box is continuous `(x1, y1, x2, y2)` from `BoundingBox2D`: `cx = center.position.x`, `cy = center.position.y`, `x1 = cx − size_x/2`, `x2 = cx + size_x/2`, same for `y`. Reject negative or non-finite sizes (exit 2). IoU is intersection area / union area, `0` if union is `0`. Intersection width is `max(0, min(x2)−max(x1))`, likewise y. No `+1` pixel. `[0,0,2,2]` vs `[1,1,3,3]` is `1/7`.

Class of a detection is the hypothesis with the **maximum score**; ties take the earlier index in `results`. Empty `results` on a detection is exit 2. Score should lie in `[0, 1]`; values outside are exit 2.

Matching at one IoU threshold: greedy by descending score (ties: earlier frame time, then earlier index). Assign to the unmatched GT of the **same `class_id`** with the highest IoU if that IoU `>=` threshold. IoU ties take the earlier GT index. Unmatched GT are FN; unmatched candidates are FP. A wrong-class prediction is an FP **only for its predicted class** and leaves the true-class GT unmatched (FN). It is not replicated across other classes. Prediction-only classes are excluded from the GT-class macro mean but still appear in micro precision and in the report.

Precision and recall at IoU `0.50` use every candidate (no extra score cutoff). Report per class and micro-average. Fixture classes: `vehicle`, `person`.

**AP is all-point interpolated AP**, not official COCO 101-recall-point AP. We do use COCO’s ten IoU thresholds. For one class with `N > 0` GT boxes, after per-frame matching at a fixed IoU, sort that class’s detections globally by descending score and form `r_i = TP_i / N`, `p_i = TP_i / (TP_i + FP_i)`. Build

`r = [0, r_1, …, r_n, 1]`, `p = [0, p_1, …, p_n, 0]`,

then from the right replace each `p[j]` by `max(p[j], p[j+1], …)`. Then

`AP = Σ_{j=0…n} (r[j+1] − r[j]) * p[j+1]`.

Empty predictions with GT present give AP `0`. `map_50` is the mean of per-class AP at IoU 0.50 over classes present in GT. `map_75` uses IoU 0.75. `map` is the mean of AP at IoU `0.50, 0.55, …, 0.95` (10 values), then the mean over GT classes. A GT-empty detection run is exit 2.

Hand AP (one class, IoU 0.5) must equal `5/6`: two GT; detections in score order TP (r=0.5, p=1), FP (r=0.5, p=0.5), TP (r=1, p=2/3). The locked formula gives `1/2 + 1/3 = 5/6`. Omitting the recall-0 sentinel, or shifting the interval endpoint, can yield `1/3`; tests include the sentinel arrays, a no-prediction case (AP 0), and a trailing FP.

### Baseline and gates

`fixtures/baselines/suite.json` holds the baseline candidate’s metrics as **JSON numbers** (IEEE-754 finite floats), not formatted strings. `-0.0` is canonicalized to `0.0` before write. `null` is JSON `null`, never `0`. Identical-replay tests compare with absolute tolerance `1e-12`. Gates compare the raw floats with the inequalities below (no prior rounding, no “almost equal” slack). Key order is not semantically significant; writers may sort keys for diffs.

Join baseline and candidate by stable run `name` and metric name. Missing or extra required runs, mismatched `metric_version` / `gate_policy_version` / units / suite name, or a missing gated key is **exit 2**, never a silent pass. A degraded bag may differ in sha256 but uses the **same run names and the same GT**.

Null matrix for gated metrics: candidate `null` vs finite baseline fails; finite candidate vs `null` baseline is reported as newly available and **passes**; both `null` pass; a missing key is invalid (exit 2), not `null`. NaN and Infinity are invalid (exit 2). Exit 2 takes precedence over exit 1 if any run is invalid. Exit 1 requires a completed, valid comparison that failed a gate. Reports are still written on exit 1.

A metric fails only when the inequality is true (equality passes):

| Metric | Fail when |
| --- | --- |
| path_length | candidate > baseline + 0.50 m |
| clearance | candidate < baseline − 0.05 m |
| success | candidate < baseline |
| time_to_goal | candidate is null and baseline is not, or both finite and candidate > baseline + 1.0 s |
| smoothness | candidate is null and baseline is not, or both finite and candidate > baseline + 0.05 rad/m |
| ate_rmse (aligned translation) | candidate > baseline + 0.05 m |
| rpe_trans_rmse | candidate > baseline + 0.05 m |
| rpe_rot_rmse | candidate > baseline + 1.0 deg |
| map, map_50, map_75 | candidate < baseline − 0.05 |

Higher-is-better: success, map, map_50, map_75, precision_50, recall_50, clearance. Lower-is-better: path_length, time_to_goal, smoothness, ate_rmse, rpe_trans_rmse, rpe_rot_rmse. Precision, recall, and unaligned ATE are reported and are not gates. `reb compare` uses the same function as `reb eval`.

Path-length gate tests use a representable baseline (e.g. `4.0`): `baseline + 0.50` passes, `baseline + 0.50 + 1e-6` fails, `baseline − 0.50` passes (one-sided). The same inclusive-boundary idea applies to higher-is-better gates.

### Degraded fixture (must fail named gates)

`fixtures/suite_degraded.yaml` evaluates the same GT with a degraded candidate in the same topic layout. All outputs remain valid (exit 1, never 2). Locked construction, with margin beyond each allowance:

1. **Localization.** Candidate = GT pose plus an x-offset `0.15 * t` meters (`t` in seconds from the first GT stamp). Aligned ATE is on the order of the 3-point oracle `sqrt(0.015) ≈ 0.122` m versus a baseline near 0, so `ate_rmse` fails the +0.05 m gate. A rigid constant offset is **not** sufficient (Umeyama removes it).
2. **Planning.** Collision-free detour with `path_length >= baseline + 1.0 m` (allowance is 0.50 m). Clearance still `>= robot_radius`. `path_length` is marked FAIL.
3. **Detection.** At least 10 GT `person` boxes; the degraded candidate emits **no** `person` detections (vehicle detections may remain). `map` (and typically `map_50`) drops by more than 0.05 and is marked FAIL.

The degraded report must name `ate_rmse`, `path_length`, and `map` as FAIL. pytest asserts exit code **1** (not 2) and those three row names.

### Messages

Vendor the minimal humble `.msg` texts under `rosbag_eval_bench/msgs/`, Apache-2.0, credited in NOTICE with URL, tag or commit, and path. Do not relicense. Do not trim fields. Each registered MCAP schema is a complete ROS 2 `ros2msg` closure (parent plus `====` / `MSG:` dependencies), as in the official `mcap` `ros2-noenv` writer example.

Sources (checked 2026-11-10):

- `ros2/common_interfaces` branch `humble`, commit `08434490d3e4abd37ccc5462cdde69b88853b5b8` — `geometry_msgs`, `nav_msgs`, `std_msgs`.
- `ros-perception/vision_msgs` tag `4.1.1` (package version 4.1.1, commit `e77fa2d97d69abaaa546592892d2d5d16cec9977`; humble branch `5f35e557cfb5e085ac5c6dc0210c5736c2c362f7` has the same field layout) — **Apache-2.0**.
- `ros2/rcl_interfaces` branch `humble`, commit `82776fc9068d1e0cd4af11c2e2700195994b2eb1` — `builtin_interfaces` (Apache-2.0, version 1.2.3). This source was missing from the draft and is required for `Time`.

Locked humble `vision_msgs` fields (the draft’s flat `Pose2D.{x,y,theta}` was **wrong**):

- `vision_msgs/Point2D`: `x`, `y`
- `vision_msgs/Pose2D`: `position` (`Point2D`), `theta`
- `BoundingBox2D`: `center` (`Pose2D`), `size_x`, `size_y` — use `center.position.x` / `center.position.y`
- `ObjectHypothesis`: `class_id` (string), `score`
- `ObjectHypothesisWithPose`: `hypothesis`, `pose`
- `Detection2D`: `header`, `results`, `bbox`, `id` (string)
- `Detection2DArray`: `header`, `detections`

Also vendor: `std_msgs/Header`; `builtin_interfaces/Time` (`sec`, `nanosec`); `geometry_msgs` Point, Quaternion, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Twist, TwistWithCovariance, Vector3; `nav_msgs` Odometry, Path, OccupancyGrid, MapMetaData.

Writer path: `mcap_ros2.writer.Writer` with `CompressionType.NONE`, `enable_crcs=True`, `register_msgdef`, dict payloads, and explicit integer `log_time` / `publish_time`. The generator never calls `time.time_ns()`. Round-trip tests cover Odometry, Path, OccupancyGrid, and Detection2DArray, not only a pose.

### Topics

Defaults (a scenario prefix is allowed only via suite remaps):

- `/gt/pose`, `/candidate/pose`: `nav_msgs/Odometry` at 20 Hz. Metrics use `pose.pose` (x, y, yaw). Twist may be zero.
- `/candidate/path`: one `nav_msgs/Path` at episode end.
- `/gt/goal`: `geometry_msgs/PoseStamped`.
- `/gt/map`: `nav_msgs/OccupancyGrid`.
- `/gt/detections`, `/candidate/detections`: `vision_msgs/Detection2DArray` at 10 Hz.
- `/gt/path`: optional, overlay only.

A run that does not enable a family may omit that family’s **required** topics. A required topic for an enabled family that is missing exits 2. For a static map/goal, if several messages exist, use the last by `log_time_ns`.

### Fixtures

`reb generate-fixtures --seed N` is deterministic. v1 committed bags use seed `20261110`. Each bag is well under 1 MiB. **Byte-identical sha256 is promised only under the pinned writer lock** (`mcap==1.6.0`, `mcap-ros2-support==0.6.0`, `CompressionType.NONE`, CRC on, fixed schema text, registration order, message order, sequences, and chunk settings). The ROS2 writer embeds package versions in `Header.library`, so a dependency bump can change bytes without changing decoded messages. CI checks (a) bag sha256 against the current lock and (b) a decoded semantic digest (topics, `log_time_ns`, and payload fields). A lock bump updates the sha256 with review; metric baselines are refreshed only if decoded metrics change. No vehicle, farm, or customer logs.

### Report, results schema, and Pages

Jinja2 renders Markdown and a self-contained HTML report. The baseline-diff table shows signed delta and PASS/FAIL (green/red). Null deltas display as `n/a`, not a fake PASS. Matplotlib PNGs (path overlay, ATE error vs time, PR sketch) sit beside the HTML.

`results.json` is versioned and includes: `metric_version`, `gate_policy_version`, suite name, effective thresholds, per-run metrics and counts, alignment transform, compare rows, and the path / ATE series the Canvas app needs. `reb eval` and `reb compare` both write this file.

Pages (`web/`): Vite `base: '/rosbag-eval-bench/'`. Asset URLs use `import.meta.env.BASE_URL` (no root-relative `/baseline.json`). The app loads **committed** `results.json` files (baseline and degraded) including `CompareRow`s; it does not reimplement gates in TypeScript. Playwright (Chromium) previews the **built** `web/dist` at the Pages base path, asserts the sample report text and a FAIL row for the degraded sample, and writes `docs/demo.png` via a local command. CI uploads a screenshot artifact and does not commit it. No pixel-perfect visual assertion.

### CLI

Stdlib `argparse`. No network and no telemetry after install.

- `reb generate-fixtures [--seed 20261110] [--write-baseline] [--out fixtures/]`
- `reb eval --suite fixtures/suite.yaml --baseline fixtures/baselines/suite.json --out reports/`
- `reb compare --baseline results_a.json --candidate results_b.json --out reports/`

`--write-baseline` is for local refresh. CI does not rewrite the baseline. Exit codes: 0 pass, 1 regression, 2 invalid. Exit 2 prints a one-line error to stderr; a traceback is not the only output. Reports are produced on exit 1.

### Optional Humble smoke

NumPy 2.5.4 requires Python `>= 3.12`; Humble’s image Python is 3.10 (EOL upstream since 2026-10-01). The optional job therefore **must not install the main package**. `tools/humble_smoke.py` uses only the stdlib plus `rosbag2_py` (and `rosbag2_storage_mcap` as needed) to open a committed fixture and check topic names and `log_time_ns` against a precomputed manifest. Pin `ros:humble-ros-base@sha256:…` and platform **before** the first run, not after. The job is `continue-on-error` / informational and does not gate core tests or Pages.

## Pinned stack

Exact pins, verified 2026-11-10 on PyPI and the npm registry. CI does not float `latest`. First build commits `requirements.lock.txt` (`pip-compile --generate-hashes`, including every transitive: pydantic typing packages, matplotlib, pytest, lz4, zstandard) and `web/package-lock.json`. CI installs with `pip install --require-hashes` and `npm ci`. Pin the lock generator version in the README. Record wheel sha256 in the lock. Choose `pip-tools`, not a mixed uv/pip story, so `--require-hashes` is native.

- Python 3.12 (security support through 2028-10). Log the exact patch on the runner. numpy 2.5 needs `>= 3.12`.
- Core CI: `ubuntu-24.04`, Python 3.12 via `actions/setup-python`.
- Optional Humble job: `ros:humble-ros-base` **digest + platform pinned in the workflow before it is enabled**. Informational only. Jazzy is not v1.
- `mcap==1.6.0` (MIT; wheel `mcap-1.6.0-py3-none-any.whl`; uploaded 2026-10-19).
- `mcap-ros2-support==0.6.0` (MIT; import `mcap_ros2`; requires `mcap>=1.6.0`; uploaded 2026-10-19). README: no ROS 2 dependency. Do not depend on `mcap-protobuf-support`.
- `numpy==2.5.4` (BSD-3-Clause and others).
- `matplotlib==3.11.2` (PSF-style). Host and CI report images only.
- `Jinja2==3.1.6` (BSD), `MarkupSafe==3.0.4` (BSD-3-Clause).
- `PyYAML==6.0.3` (MIT).
- `pydantic==2.14.3` (MIT) and `pydantic-core==2.50.4` (MIT; the exact companion of 2.14.3).
- `pytest==9.2.0` (MIT). Lock `iniconfig`, `packaging`, `pluggy`, `pygments`.
- `ruff==0.17.4` (MIT).
- mcap transitive: `lz4==4.4.5`, `zstandard==0.25.0` (BSD-family).
- Node **26.13.0** Active LTS (Lithium, released 2026-10-28). This release **bundles npm 11.20.0**; that is the npm we use (npm latest on 2026-11-10 was 12.2.2; we are not floating npm independently). Node 20 is EOL.
- TypeScript 7.0.2, Vite 8.3.7, Vitest 5.1.0, `@playwright/test` 1.65.0 with its matching `playwright` 1.65.0 and Chromium revision. “No C++ / WASM core” means this demo has no authored C++ metric engine; it does not describe the TypeScript 7 native compiler.
- No Eigen, no GoogleTest, no chart library.

## File layout

```
rosbag_eval_bench/          # installable package
  cli.py
  reader.py                 # BagReader, McapRos2BagReader (uses log_time_ns)
  schema.py                 # SuiteConfig, RunConfig, Thresholds, result models
  types.py
  associate.py
  metrics_traj.py
  metrics_plan.py
  metrics_det.py
  compare.py
  report.py
  generate.py
  msgs/                     # vendored humble .msg texts
tools/humble_smoke.py       # isolated, no numpy 2.5
fixtures/bags/*.mcap
fixtures/suite.yaml
fixtures/suite_degraded.yaml
fixtures/baselines/suite.json
fixtures/web/               # committed results JSON for Pages
tests/
web/                        # Vite app + package-lock.json
docs/demo.png
.github/workflows/ci.yml    # test, web, comment, pages jobs in one file
.github/workflow-pins.md
requirements.lock.txt
LICENSE
NOTICE
README.md
```

## Interfaces

Pydantic models:

- `Thresholds`: the absolute allowances in the table above, as defaults.
- `RunConfig`: `name`, `bag`, `metrics` (`planning` | `localization` | `detection`), `topics` remaps, `robot_radius=0.25`, `goal_tol_xy=0.50`, `goal_tol_yaw=0.25`, `max_dt_ns=50_000_000`, `min_match_ratio=0.90`, `rpe_delta_s=1.0`, `min_rpe_pairs=3`.
- `SuiteConfig`: `name`, `metric_version=1`, `gate_policy_version=1`, `runs`, `thresholds`.
- Result / `CompareRow` models as written to `results.json`.

Dataclasses:

- `Pose2D(x, y, yaw, t_ns)`
- `Path2D(poses, frame_id)`
- `Grid(width, height, resolution, origin_x, origin_y, origin_yaw, data)`
- `Detection(class_id, score, x1, y1, x2, y2)`
- `DetectionFrame(t_ns, detections)`
- `MatchedPoses(gt, est, match_ratio, n_matched, n_gt)`
- `MetricValue(name, value: float | None, unit, direction: Literal['higher','lower'], null_ok)`
- `CompareRow(name, baseline, candidate, delta, passed, reason)`
- `CompareResult(passed: bool, rows, invalid: bool)`

Functions: `associate_poses`, `ate_rpe`, `planning`, `detection`, `compare`, `write_markdown`, `write_html`. Exit codes 0 / 1 / 2.

Suite YAML is loaded with `safe_load`, rejects unknown keys, and resolves bag paths relative to the suite file.

## Edge cases

- Yaw interpolation across ±π stays on the short arc; a unit test uses samples near `+π` and `-π`. Antipodal difference of exactly `π` is unmatched.
- Exact timestamp matches are valid without interpolation.
- File-order non-monotonic or duplicate `log_time_ns` on a required topic: exit 2.
- Quaternion sign invariance; zero/NaN quaternion: exit 2.
- Zero-area boxes: IoU 0, still FP/FN.
- Score ties and IoU ties use the stated deterministic keys.
- `time_to_goal` null is never formatted as `0`.
- Path pose timestamps must be monotonic; mixed `frame_id` is exit 2.
- Reaching the goal and later colliding still yields success 0 (collision is global).
- Fork PRs: the check runs; the comment step is skipped when `github.event.pull_request.head.repo.full_name != github.repository`.

## Test plan

Unit (no ROS, no wall-clock asserts):

- IoU `1/7`, plus disjoint, containment, identical, and zero-area cases.
- Umeyama SE(2) 90° toy (yaw **−π/2**, translation 0, aligned RMSE 0) and a non-collinear mirrored set that cannot align to zero with a proper rotation.
- Unaligned ATE `sqrt(0.02)` and aligned RMSE 0 on the constant offset; drifted 3-point ATE `sqrt(0.015)`.
- RPE identity 0, 0.1 m relative-translation **error**, 1° relative-yaw **error**, and <3 pairs ⇒ error.
- Smoothness L-path and straight line.
- Clearance face, corner, and a **segment that crosses a cell without a vertex inside**.
- Success true/false, including graze at `clearance == robot_radius`.
- Association: exact match, wrap-around yaw, each-side `max_dt`, `match_ratio == 0.90` passes, below fails.
- Detection AP hand example `5/6`, no-prediction AP 0, wrong-class not counted as FP on other classes.

Fixture: write/read Odometry, Path, OccupancyGrid, Detection2DArray with the pinned writer; `log_time_ns` matches. Committed bag sha256 equals `reb generate-fixtures --seed 20261110` under the current lock. Semantic digest also matches.

Regression: default suite exits 0. Identical replay vs `suite.json` within `1e-12`. Degraded suite exits 1 and marks `ate_rmse`, `path_length`, and `map` FAIL. Path-length boundary cases as specified.

Invalid: missing topic, empty planning path, low match ratio, bad YAML, NaN metric → exit 2 and a one-line stderr message.

Web: Playwright on the built site at `/rosbag-eval-bench/` as above. No TS reimplementation of gates to unit-test.

## Numbered success criteria

1. `pytest` on ubuntu-24.04 / Python 3.12 passes with no ROS installed, using pinned `mcap==1.6.0` and `mcap-ros2-support==0.6.0`.
2. A generated MCAP round-trips topic names, integer `log_time_ns`, and payloads for Odometry, Path, OccupancyGrid, and Detection2DArray through `read_ros2_messages`.
3. Unit oracles hold within `1e-12`: IoU `1/7`; unaligned ATE `sqrt(0.02)` and aligned `0` on the constant offset; 90° estimate-to-GT yaw `−π/2`; drifted ATE `sqrt(0.015)`; hand AP `5/6`; L-path smoothness `(π/2)/4`; RPE identity `0` and 0.1 m / 1° error cases.
4. `reb eval --suite fixtures/suite.yaml --baseline fixtures/baselines/suite.json` exits 0.
5. The degraded suite exits 1 (not 2) and the report marks `ate_rmse`, `path_length`, and `map` FAIL.
6. An identical replay matches the stored baseline within `1e-12` and does not fail a gate. The path_length boundary cases around `+0.50 m` behave as specified.
7. `reb generate-fixtures --seed 20261110` reproduces committed bag sha256 under the pinned writer lock. Bags stay under 1 MiB. Compression is none and CRC is on. A decoded semantic digest is also checked.
8. `report.md` and `report.html` include a baseline-diff table with signed deltas and PASS/FAIL. `results.json` includes compare rows and plot series.
9. Playwright loads `/rosbag-eval-bench/` from the built `web/dist`, shows the sample report and a FAIL row for the degraded JSON, and a local command has produced `docs/demo.png`.
10. The optional Humble smoke is isolated from numpy 2.5 / Python 3.12, and does not gate the core CI or Pages deploy.
11. `LICENSE` is the verbatim text from https://www.apache.org/licenses/LICENSE-2.0.txt. `NOTICE` starts with `rosbag-eval-bench, Copyright 2026 Arpan Bom` and lists vendored message packages (including `builtin_interfaces`) with URL, commit or tag, and license. Authored files carry `SPDX-License-Identifier: Apache-2.0`.
12. README states synthetic fixtures and simulation only, names Arpan Bom, describes the clearance, AP, and time-to-goal honesty notes, and does not claim Autoware, driving_log_replayer, Lely HIL, or certified QA.
13. The tree does not vendor or depend on evo, pycocotools, Detectron2, driving_log_replayer, Cartographer, or Nav2 metric sources, including optional and dev extras.
14. Commit author and committer are Arpan Bom `<applycoding@gmail.com>`; no co-author trailers.
15. Fixtures contain no real or proprietary bag data. After the documented pip/npm/Actions bootstrap, evaluation and tests make no network calls.

## Out of scope

Real robot or customer bags. Proprietary logs. Autoware or driving_log_replayer compatibility. Live `ros2 bag play` closed-loop control. Training detectors or planners. 3D or rotated boxes. SE(3) or Sim(3) scale alignment as a gate. Official COCO 101-recall-point AP as a numeric target. A GPL evo dependency. Foxglove cloud. SaaS or accounts. Windows CI. Jazzy CI. Copying Nav2, Cartographer, Detectron2, or TIER IV sources. Certified functional-safety evaluation. `mcap-protobuf-support`. `ternaris/rosbags` as a v1 dependency. A TypeScript reimplementation of the metric gates.

## Documentation and honesty

README credits idea sources and states the implementation is original. It must say:

- Synthetic fixtures, simulation only.
- Aligned ATE uses a rigid SE(2) estimate-to-GT transform; a constant offset will not fail the gate.
- RPE is Sturm-style time-based relative pose error at Δt = 1 s, not KITTI distance-based relative error.
- AP is all-point interpolation at COCO IoU thresholds, not official COCO 101-point AP.
- Clearance is polyline-to-occupied-square distance; unknown cells are passable; time-to-goal uses path pose stamps, not a live robot arrival.
- A 2.5 / Python 3.12 stack is not installed in the Humble smoke job.

Idea references are listed in `REFERENCES.md`.

## License

Apache-2.0. `LICENSE` is the official text fetched verbatim from https://www.apache.org/licenses/LICENSE-2.0.txt. `NOTICE` begins with `rosbag-eval-bench, Copyright 2026 Arpan Bom` and lists vendored message packages and redistributed third-party notices (MIT/BSD Python deps, NumPy’s multi-license statement). SPDX headers on files this repo authors. Vendored `.msg` files keep their upstream copyright comments and are not given an Apache SPDX that claims we authored them.

## Build and CI

One workflow file `.github/workflows/ci.yml` so Pages can `needs:` the test and web jobs of the **same commit**. Jobs:

1. **core:** ruff, pytest (unit, fixture sha256 + semantic digest, default suite exit 0, degraded suite exit 1 asserted in the shell without `|| true`). `ubuntu-24.04`, Python 3.12, no ROS. Upload `reports/` always when the comparison ran.
2. **web:** Node 26.13.0, `npm ci`, `tsc`, vitest, Playwright against the built dist at the Pages base. Upload the screenshot artifact.
3. **comment** (same-repo `pull_request` only, not `pull_request_target`): `pull-requests: write` only on this job. Runs even when core exits 1, so a valid regression still gets a comment. Does not execute candidate code; takes the Markdown artifact as data. Updates one comment with a stable bot marker. Skip for forks. Failure to comment must not mask a metric failure.
4. **pages:** only on `main`, after core and web succeed. `pages: write` and `id-token: write` confined to this job. Deploys the tested `web/dist` artifact.

Every Action is pinned to a full commit SHA, recorded in `.github/workflow-pins.md` when first written (checkout, setup-python, setup-node, upload/download-artifact, configure-pages, deploy-pages, github-script). Hashed Python lock is installed with `--require-hashes`.

Tests must not compare wall-clock durations. Matplotlib is confined to report generation.

## Risks

- Writer bytes change if the library identifier or chunk layout changes. Pin hashes, assert sha256 under the current lock, and keep a semantic digest so header-only changes do not silently refresh metric baselines.
- Humble `Pose2D` is nested (`position.x/y`, `theta`). Encode/decode fixtures must use that layout.
- `read_ros2_messages().log_time` is a local datetime; only `log_time_ns` is safe.
- Collinear Umeyama toys do not fully test the reflection prohibition; the mirrored non-collinear case is mandatory.
- A constant localization offset will not trip aligned ATE; the degraded fixture must use non-rigid drift.
- Humble’s Python 3.10 cannot install this package’s numpy 2.5 stack; the smoke job stays isolated or it will fail for an uninteresting reason.
- Fork PR tokens cannot comment upstream. The skip is explicit; the fail check still runs.

## Verification already done (do not re-litigate)

Performed 2026-11-10 with `mcap==1.6.0`, `mcap-ros2-support==0.6.0`, and `numpy==2.5.4` (see `_verify_math.py`):

- IoU of `[0,0,2,2]` vs `[1,1,3,3]` is exactly `1/7`.
- Unaligned ATE on the constant offset is `sqrt(0.02)`; aligned RMSE is 0 with yaw 0 and translation `(-0.1,-0.1)`.
- The 90° toy recovers estimate-to-GT yaw **−π/2**, not +π/2, and aligned RMSE 0.
- RPE of identical relatives is 0; a 0.1 m relative-translation error and a 1° relative-yaw error produce RMSE 0.1 and 1 respectively.
- L-path smoothness is `(π/2)/4`.
- The locked all-point AP formula on the hand example is `5/6`. A missing recall-0 sentinel can yield `1/3`.
- Distance to an occupied-cell **center minus res/2** overestimates clearance at corners; the square formula does not. Adopt the square (and segment) formula.
- Path-length gates: `baseline + 0.50` passes, `baseline + 0.50 + 1e-6` fails. A degraded path +0.8 m, ATE +0.20 m, or map drop of 0.15 fails the corresponding gate.
- `format(x, '.17g')` of `sqrt(0.02)` is not unique across `math.sqrt` vs `numpy` reductions (1 ULP). Do not treat `.17g` strings as a reproducibility scheme; compare JSON numbers with `1e-12`.
- A tiny Odometry MCAP written twice with `CompressionType.NONE` is bit-identical (sha256 `1d5282c7d30e81fd0c5786fedc81cf7a1a6a919a111faa41b7fc68c43faafcec` on this machine, size 7033). `log_time` on the wrapper is a local datetime; `log_time_ns` is the integer 1e9, 1.05e9, …
- Humble `vision_msgs` 4.1.1 `Pose2D` is `position` + `theta`, not flat `x,y,theta`.
- No existing `applycoding/rosbag-eval-bench` repo. All listed documentation and GitHub URLs returned HTTP 200 on 2026-11-10.
