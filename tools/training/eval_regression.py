#!/usr/bin/env python3
"""eval_regression — score a candidate model against the held-out regression set.

This is the G2 model gate: per-class precision/recall at IoU>=0.5, plus the
confusion matrix among the 5 LED buoy states (the classes the mission logic
actually branches on). Exits nonzero if any gated class misses its floor, so it
can gate a deploy the same way tests gate a merge.

The matching/metric functions have no ultralytics dependency and are unit-tested
in tests/test_eval_metrics.py; only main() needs the model runtime.

Usage (Jetson/CUDA):
  python3 eval_regression.py --weights best.pt \
      --regression ~/datasets/buoys_v1/regression --out report.json
"""
import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

GATED_CLASSES = ["buoy_off", "buoy_flash_red", "buoy_flash_green",
                 "buoy_flash_blue", "buoy_solid_blue"]
DEFAULT_FLOOR = {"precision": 0.80, "recall": 0.80}
IOU_MATCH = 0.5


# ---------- pure logic (unit-tested) ----------

def iou(a, b):
    """a, b = (x1, y1, x2, y2)."""
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter == 0:
        return 0.0
    area = ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)
    return inter / area


def greedy_match(preds, gts, iou_floor=IOU_MATCH):
    """preds: [(label, conf, bbox)] sorted desc by conf; gts: [(label, bbox)].
    Returns (matches [(pred_i, gt_i)], unmatched_pred_is, unmatched_gt_is).
    Matching is class-agnostic (localization first) so cross-class confusion is
    observable; classification is scored on the matched pairs."""
    matches, used_gt, unmatched_pred = [], set(), []
    for pi, (plabel, conf, pb) in sorted(enumerate(preds), key=lambda kv: -kv[1][1]):
        best_gi, best_iou = None, iou_floor
        for gi, (glabel, gb) in enumerate(gts):
            if gi in used_gt:
                continue
            v = iou(pb, gb)
            if v >= best_iou:
                best_gi, best_iou = gi, v
        if best_gi is None:
            unmatched_pred.append(pi)
        else:
            used_gt.add(best_gi)
            matches.append((pi, best_gi))
    unmatched_gt = [gi for gi in range(len(gts)) if gi not in used_gt]
    return matches, unmatched_pred, unmatched_gt


def score(frames):
    """frames: list of (preds, gts) per frame (formats as in greedy_match).
    Returns dict: per-class {tp, fp, fn, precision, recall} + confusion."""
    tp = defaultdict(int)
    fp = defaultdict(int)
    fn = defaultdict(int)
    confusion = defaultdict(int)          # (gt_label, pred_label) -> count
    for preds, gts in frames:
        matches, un_p, un_g = greedy_match(preds, gts)
        for pi, gi in matches:
            plabel, glabel = preds[pi][0], gts[gi][0]
            confusion[(glabel, plabel)] += 1
            if plabel == glabel:
                tp[glabel] += 1
            else:
                fp[plabel] += 1
                fn[glabel] += 1
        for pi in un_p:
            fp[preds[pi][0]] += 1
        for gi in un_g:
            fn[gts[gi][0]] += 1
    classes = sorted(set(tp) | set(fp) | set(fn))
    per_class = {}
    for c in classes:
        p = tp[c] / (tp[c] + fp[c]) if tp[c] + fp[c] else None
        r = tp[c] / (tp[c] + fn[c]) if tp[c] + fn[c] else None
        per_class[c] = {"tp": tp[c], "fp": fp[c], "fn": fn[c],
                        "precision": round(p, 3) if p is not None else None,
                        "recall": round(r, 3) if r is not None else None}
    return {"per_class": per_class,
            "confusion": {f"{g}->{p}": n for (g, p), n in sorted(confusion.items())}}


def gate(report, floor=None, gated=GATED_CLASSES):
    """Returns (ok, failures). A gated class with no ground truth in the set is a
    failure — the regression set must cover all five LED states."""
    floor = floor or DEFAULT_FLOOR
    failures = []
    for c in gated:
        pc = report["per_class"].get(c)
        if pc is None or (pc["tp"] + pc["fn"]) == 0:
            failures.append(f"{c}: no ground truth in regression set")
            continue
        for metric, m_floor in floor.items():
            v = pc[metric]
            if v is None or v < m_floor:
                failures.append(f"{c}: {metric}={v} < {m_floor}")
    return (not failures), failures


# ---------- runtime ----------

def load_yolo_labels(txt: Path, classes, w, h):
    gts = []
    for line in txt.read_text().split("\n"):
        if not line.strip():
            continue
        ci, cx, cy, bw, bh = line.split()[:5]
        cx, cy, bw, bh = (float(v) for v in (cx, cy, bw, bh))
        gts.append((classes[int(ci)],
                    ((cx - bw / 2) * w, (cy - bh / 2) * h,
                     (cx + bw / 2) * w, (cy + bh / 2) * h)))
    return gts


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--regression", required=True)
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--out", default="regression_report.json")
    args = ap.parse_args()

    import cv2
    from ultralytics import YOLO
    model = YOLO(args.weights)

    reg = Path(args.regression)
    classes = None
    for cfile in reg.rglob("classes.txt"):
        classes = cfile.read_text().split()
        break
    if classes is None:
        sys.exit("no classes.txt under the regression dir")

    frames = []
    for img_path in sorted(reg.rglob("*.jpg")):
        txt = img_path.with_suffix(".txt")
        if not txt.exists():
            continue
        img = cv2.imread(str(img_path))
        h, w = img.shape[:2]
        res = model(img, conf=args.conf, verbose=False)[0]
        preds = [(res.names[int(b.cls[0])], float(b.conf[0]),
                  tuple(float(v) for v in b.xyxy[0])) for b in res.boxes]
        frames.append((preds, load_yolo_labels(txt, classes, w, h)))

    report = score(frames)
    report["n_frames"] = len(frames)
    ok, failures = gate(report)
    report["gate"] = {"ok": ok, "failures": failures}
    Path(args.out).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    if not ok:
        print("\nGATE FAIL — do not export/deploy this model:", file=sys.stderr)
        for f in failures:
            print(f"  {f}", file=sys.stderr)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
