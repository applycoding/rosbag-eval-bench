# Build tasks — rosbag-eval-bench

Execute in order. Follow PLAN.md. Deviate only if a pin or test proves a step broken, and record why. License is **Apache-2.0**, not MIT. Commit author and committer are Arpan Bom <applycoding@gmail.com>; no co-author trailers.

Each task has acceptance checks. A task is done only when those checks pass.

## 1. Repository skeleton and license

- Create the tree in PLAN.md (empty dirs may hold `.gitkeep`).
- Fetch `https://www.apache.org/licenses/LICENSE-2.0.txt` into `LICENSE` and confirm byte equality (including the appendix).
- Write `NOTICE` starting with `rosbag-eval-bench, Copyright 2026 Arpan Bom`, then vendored-message provenance and a dependency-notice section (filled as locks land).
- README stub: portfolio demo, synthetic fixtures only, Apache-2.0, author Arpan Bom, no Autoware / driving_log_replayer / Lely HIL / certified-QA claims.
- `pyproject.toml` for package `rosbag_eval_bench`, console script `reb`, Python `>=3.12`, pinned dependencies as in PLAN.md. `pytest` and `ruff` as dev extras.
- Add `SPDX-License-Identifier: Apache-2.0` on every authored source file from here on (after shebangs). Do not put that SPDX on vendored `.msg` files.

**Accept:** `cmp LICENSE` against a fresh curl of the official text exits 0. NOTICE and README name Apache-2.0 and Arpan Bom. `pip install -e .` (or equivalent) exposes `reb --help`.

## 2. Hashed Python lock and NOTICE for dependencies

- Commit `requirements.lock.txt` from `pip-compile --generate-hashes` covering every transitive package named in PLAN.md. Pin the `pip-tools` version used in the README.
- Append MIT/BSD/PSF/NumPy notices to `NOTICE`. Do not relicense them.

**Accept:** `pip install --require-hashes -r requirements.lock.txt` succeeds on Python 3.12. NOTICE lists each direct dependency and its license.

## 3. Vendored humble message definitions

- Copy the `.msg` texts listed in PLAN.md from the locked commits/tags (`common_interfaces` humble `08434490…`, `vision_msgs` 4.1.1 `e77fa2d…`, `rcl_interfaces` humble `82776fc…`). Keep upstream copyright comments. Record URL, commit or tag, path, and Apache-2.0 in NOTICE.
- Build complete `ros2msg` closures for Odometry, Path, OccupancyGrid, PoseStamped, and Detection2DArray, including `Point2D` and nested `Pose2D`.

**Accept:** Closures contain `builtin_interfaces/Time` and `vision_msgs/Pose2D.position`. A unit test shows `Pose2D` has `position.x` / `position.y` / `theta`, not flat `x,y,theta`.

## 4. MCAP reader/writer and log_time_ns

- Implement `McapRos2BagReader` using `read_ros2_messages` and **`log_time_ns` only** (never `.log_time`).
- Implement the generator writer: `CompressionType.NONE`, CRC on, explicit integer timestamps, no `time.time_ns()`.

**Accept:** A pytest writes and reads Odometry, Path, OccupancyGrid, and Detection2DArray; topics, `log_time_ns`, and payload fields match. Two writes of the same fixture under the pin are sha256-identical.

## 5. Association

- Implement the exact-match and each-side `max_dt` bracket rule, yaw wrap interpolation, antipodal-π rejection, and `match_ratio` as in PLAN.md.
- Detection frames: one-to-one nearest-time matching with the stated tie breaks.

**Accept:** Tests for exact match, ±π wrap, each-side 50 ms, `match_ratio == 0.90` pass, below 0.90 raises the invalid-run error, and unused candidate frames become FPs.

## 6. Localization metrics

- Implement estimate-to-GT SE(2) Umeyama (no scale, det fix) and Sturm-style RPE as in PLAN.md. Population std. Report unaligned ATE but gate only aligned `ate_rmse`.

**Accept:** Oracles in criterion 3 pass within `1e-12`: `sqrt(0.02)` / aligned 0 / yaw 0 on the offset; yaw `−π/2` on the 90° toy; drifted `sqrt(0.015)`; RPE identity 0, 0.1 m error, 1° error; <3 RPE pairs is invalid. A mirrored non-collinear set does not align to zero with a proper rotation.

## 7. Planning metrics

- Implement path length, polyline-to-occupied-square clearance (vertices and segments), success, time_to_goal, and smoothness. Occupancy index `j * width + i`. Origin yaw must be 0. Fixtures use 0 / 100 / -1, threshold `>= 50`. Empty planning path is invalid (exit 2). Single-pose `L = 0` is allowed as in PLAN.md.

**Accept:** L-path smoothness `(π/2)/4`; face and corner clearance oracles; a segment that crosses an occupied cell without a vertex inside has `collision` true and clearance 0; a swapped width/height index test fails if the formula is transposed; success at `clearance == robot_radius`.

## 8. Detection metrics

- Implement continuous IoU, max-score class selection, greedy matching, and the locked all-point AP formula (sentinels and `(Δr) * p[j+1]`). `map_50`, `map_75`, `map` as specified. Wrong-class FPs stay on the predicted class only.

**Accept:** IoU `1/7`; hand AP `5/6`; no-prediction AP 0; a wrong-class box is not an FP on the other class; empty `results` is invalid.

## 9. Compare, gates, and CLI exits

- Implement `compare` with the null matrix, one-sided absolute gates, `metric_version` / `gate_policy_version` checks, and exit-code precedence (2 over 1). JSON numbers, `1e-12` replay tolerance, no `.17g` strings as payload.
- Wire `reb eval`, `reb compare`, and `reb generate-fixtures`.

**Accept:** `baseline + 0.50` path_length passes; `+ 0.50 + 1e-6` fails; `baseline - 0.50` passes. Missing key is exit 2. Candidate null vs finite baseline fails. `reb --help` lists the three commands.

## 10. Fixtures, baseline, and degraded suite

- Seed `20261110`. Nominal bag (good candidate) and degraded bag as specified in PLAN.md (non-rigid ATE drift, collision-free detour `>= baseline + 1.0 m`, ≥10 GT person boxes with zero person detections). Bags < 1 MiB.
- Commit `fixtures/baselines/suite.json` produced by the generator (not hand-edited floats).
- CI / pytest: default suite exits 0; identical replay within `1e-12`; degraded suite exits **1** and marks `ate_rmse`, `path_length`, and `map` FAIL.

**Accept:** Criteria 4–7 hold. `generate-fixtures` without `--write-baseline` does not change the committed baseline. Semantic digest is checked as well as sha256.

## 11. Reports

- Jinja2 Markdown and HTML with a baseline-diff table (signed delta, PASS/FAIL, `n/a` for null deltas). Matplotlib PNGs. `results.json` includes compare rows and path / ATE series.

**Accept:** Criterion 8 holds for both the default and degraded evals. Opening `report.html` without a server shows the table (relative image links work from the report directory).

## 12. Pages app and Playwright

- Vite app, `base: '/rosbag-eval-bench/'`, assets via `import.meta.env.BASE_URL`. Loads committed JSON; does **not** reimplement gates. Canvas 2D path and ATE overlay.
- `web/package-lock.json`. Playwright Chromium against the **built** dist at the Pages base path. Local command writes `docs/demo.png` once; CI uploads an artifact and does not commit it.

**Accept:** `npm ci && npx tsc --noEmit && npx vitest run && npm run build` on Node 26.13.0. Criterion 9 holds.

## 13. CI workflow (one file)

- `.github/workflows/ci.yml` with core, web, comment, and pages jobs as in PLAN.md. Pin Actions to full SHAs in `.github/workflow-pins.md`.
- Degraded exit 1 is asserted in a shell step without `|| true`. Comment job has `pull-requests: write` only, runs on a valid exit 1, skips forks, does not execute PR code. Pages deploys the tested `web/dist` from `main` only after core and web of the **same commit**.
- Optional Humble job, if added, is isolated (`tools/humble_smoke.py`, no numpy 2.5 install), digest-pinned, and non-gating.

**Accept:** Criterion 10 and the workflow shape in PLAN.md hold. A first branch CI shows core and web green, including the degraded-fail assertion.

## 14. README, credits, and honesty pass

- Finish README in the honesty order given in PLAN.md. Credit idea sources and licenses from REFERENCES.md. Flag evo (GPL) and cocoapi as excluded. Document `log_time_ns`, nested `Pose2D`, square clearance, all-point AP vs COCO 101-point, Sturm RPE vs KITTI relative error, and the isolated Humble smoke.
- Footer says Apache-2.0. Screenshot in README once `docs/demo.png` exists.

**Accept:** Criteria 11–13 and 15 can be checked off by reading the tree. No MIT license artifacts for *this* repo. No Autoware / production-HIL claims. No evo or pycocotools in any dependency extra.

## 15. Git history hygiene (when the public repo is created)

- Every commit: `GIT_AUTHOR_NAME="Arpan Bom"`, `GIT_AUTHOR_EMAIL="applycoding@gmail.com"`, and the same for committer. No co-author trailers. Do not rewrite already-pushed history without explicit approval.

**Accept:** `git log --format='%an <%ae> %cn <%ce> %s %b'` shows only Arpan Bom as author and committer and no co-author trailers.

---

Do not start Autoware compatibility, live bag play, a TypeScript metric engine, official COCO 101-point AP, SE(3) gates, or an Ideas Hub product in this repository.
