# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from rosbag_eval_bench.types import Detection, DetectionFrame, DetMetrics, EvalError


def iou(a: Detection, b: Detection) -> float:
    ix1 = max(a.x1, b.x1)
    iy1 = max(a.y1, b.y1)
    ix2 = min(a.x2, b.x2)
    iy2 = min(a.y2, b.y2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0.0, a.x2 - a.x1) * max(0.0, a.y2 - a.y1)
    area_b = max(0.0, b.x2 - b.x1) * max(0.0, b.y2 - b.y1)
    union = area_a + area_b - inter
    if union <= 0.0:
        return 0.0
    return inter / union


def detection(
    frame_pairs: list[tuple[DetectionFrame, DetectionFrame | None]],
) -> DetMetrics:
    gt_total = sum(len(gf.detections) for gf, _ in frame_pairs)
    if gt_total == 0:
        raise EvalError("detection run has no GT boxes")

    gt_classes = _gt_classes(frame_pairs)
    map50_vals = [_ap_one_class(frame_pairs, c, 0.50) for c in gt_classes]
    map75_vals = [_ap_one_class(frame_pairs, c, 0.75) for c in gt_classes]
    map50 = float(sum(map50_vals) / len(map50_vals)) if map50_vals else 0.0
    map75 = float(sum(map75_vals) / len(map75_vals)) if map75_vals else 0.0

    iou_thresholds = [round(0.50 + 0.05 * k, 2) for k in range(10)]
    macro_map_vals: list[float] = []
    for cls in gt_classes:
        aps = [_ap_one_class(frame_pairs, cls, thr) for thr in iou_thresholds]
        macro_map_vals.append(sum(aps) / len(aps))
    map_val = float(sum(macro_map_vals) / len(macro_map_vals)) if macro_map_vals else 0.0

    p50, r50 = _precision_recall_at(frame_pairs, 0.50)
    per_class_report: dict[str, dict[str, float]] = {}
    for cls in gt_classes:
        pc, rc = _precision_recall_at(frame_pairs, 0.50, class_filter=cls)
        per_class_report[cls] = {
            "ap50": _ap_one_class(frame_pairs, cls, 0.50),
            "precision_50": pc,
            "recall_50": rc,
        }

    return DetMetrics(
        precision_50=p50,
        recall_50=r50,
        map_50=map50,
        map_75=map75,
        map=map_val,
        per_class=per_class_report,
    )


def _gt_classes(frame_pairs: list[tuple[DetectionFrame, DetectionFrame | None]]) -> list[str]:
    classes: set[str] = set()
    for gf, _ in frame_pairs:
        for d in gf.detections:
            classes.add(d.class_id)
    return sorted(classes)


def _match_frame(
    gt: list[Detection],
    cand: list[Detection],
    thr: float,
) -> tuple[list[tuple[int, int]], set[int], set[int]]:
    cand_sorted = sorted(enumerate(cand), key=lambda x: (-x[1].score, x[0]))
    matched_gt: set[int] = set()
    matched_c: set[int] = set()
    pairs: list[tuple[int, int]] = []
    for ci, det in cand_sorted:
        best_gi: int | None = None
        best_iou = -1.0
        for gi, gt_det in enumerate(gt):
            if gi in matched_gt:
                continue
            if det.class_id != gt_det.class_id:
                continue
            v = iou(det, gt_det)
            tie = abs(v - best_iou) <= 1e-15 and (best_gi is None or gi < best_gi)
            if v >= thr and (v > best_iou + 1e-15 or tie):
                best_iou = v
                best_gi = gi
        if best_gi is not None:
            matched_gt.add(best_gi)
            matched_c.add(ci)
            pairs.append((best_gi, ci))
    return pairs, matched_gt, matched_c


def _collect_global_for_ap(
    frame_pairs: list[tuple[DetectionFrame, DetectionFrame | None]],
    class_id: str,
    thr: float,
) -> tuple[list[tuple[float, int, int, bool]], int]:
    gt_frames: list[list[Detection]] = []
    cand_frames: list[list[Detection]] = []
    for gf, cf in frame_pairs:
        gt_frames.append([d for d in gf.detections if d.class_id == class_id])
        cand_frames.append([] if cf is None else [d for d in cf.detections if d.class_id == class_id])
    n_gt = sum(len(g) for g in gt_frames)
    events: list[tuple[float, int, int, bool]] = []
    for fi, (gts, cands) in enumerate(zip(gt_frames, cand_frames, strict=True)):
        _, _, matched_c = _match_frame(gts, cands, thr)
        for ci, det in enumerate(cands):
            events.append((det.score, fi, ci, ci in matched_c))
    events.sort(key=lambda e: (-e[0], e[1], e[2]))
    return events, n_gt


def _ap_one_class(
    frame_pairs: list[tuple[DetectionFrame, DetectionFrame | None]],
    class_id: str,
    thr: float,
) -> float:
    events, n_gt = _collect_global_for_ap(frame_pairs, class_id, thr)
    if n_gt == 0:
        return 0.0
    if not events:
        return 0.0
    tp = 0
    fp = 0
    r_list = [0.0]
    p_list = [0.0]
    for _score, _fi, _di, is_tp in events:
        if is_tp:
            tp += 1
        else:
            fp += 1
        r_list.append(tp / n_gt)
        denom = tp + fp
        p_list.append(tp / denom if denom else 0.0)
    r_list.append(1.0)
    p_list.append(0.0)
    for j in range(len(p_list) - 2, -1, -1):
        p_list[j] = max(p_list[j], p_list[j + 1])
    ap = 0.0
    for j in range(len(r_list) - 1):
        ap += (r_list[j + 1] - r_list[j]) * p_list[j + 1]
    return ap


def _precision_recall_at(
    frame_pairs: list[tuple[DetectionFrame, DetectionFrame | None]],
    thr: float,
    class_filter: str | None = None,
) -> tuple[float, float]:
    tp = fp = fn = 0
    for gf, cf in frame_pairs:
        gts = [d for d in gf.detections if class_filter is None or d.class_id == class_filter]
        cands = [] if cf is None else [d for d in cf.detections if class_filter is None or d.class_id == class_filter]
        _, matched_g, matched_c = _match_frame(gts, cands, thr)
        tp += len(matched_c)
        fp += len(cands) - len(matched_c)
        fn += len(gts) - len(matched_g)
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    return float(precision), float(recall)
