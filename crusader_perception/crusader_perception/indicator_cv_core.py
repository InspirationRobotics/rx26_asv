"""indicator_cv_core: a docking bay's colour indicator read by colour thresholds, no model.

camera_node's dock model uses it when its `dock_indicator` is "cv" (the task3_scan_fire_cv
tree asks for that). The detector still finds each bay's face and its windows; the indicator
is read from the pixels at the base of the face instead of from the detector's
dock_indicator box and colour_core.

The indicator (handbook: Docking Bay Structure, Color Indicator Build Guide) is a RoboBoat
colour indicator: a 3D-printed cylinder, red or green PCTG fins, NOT lit, mounted at the base
of the structure's front panel, centred, facing into the bay.

  1. where to look: a box in face units, `roi` = (x0, x1, y0, y1) from the face box's
     top-left, its width and height 1 (y > 1 is below the face); the windows' boxes, padded,
     cut out of it (a lit window is RED or GREEN too)
  2. which pixels count: saturated (s >= sat_min) and not dark (v >= val_min), then RED if the
     hue is within red_hue (degrees, through 0) and GREEN if within green_hue
  3. each colour's count as a share of the FACE's area. The larger must reach min_share, or
     nothing coloured is there (no indicator this frame); and be `ratio` times the other, or
     the read is "unknown" (both colours: not a vote)
  4. confidence = larger / (red + green): 0.5 is a tie, and the tree takes a read as a vote
     from 0.5 (dock::BookParams.min_ind_conf)
  5. the indicator's box: where the winning colour's pixels are (2nd to 98th percentile), for
     its bearing and for the face-plane fit to leave out

numpy only (and indicator_cv_settings: the settings, their defaults and their check),
so it runs and is tested anywhere.
"""
import math

import numpy as np

from crusader_perception.indicator_cv_settings import check

MIN_FACE_PX = 8                         # a face box smaller than this is not read
HOLE_PAD = 0.15                         # windows cut out this much bigger, each way


def hsv(rgb):
    """HSV of an RGB uint8 array: hue in degrees [0, 360), saturation and value 0..1."""
    x = np.asarray(rgb, dtype=np.float32) / 255.0
    r, g, b = x[..., 0], x[..., 1], x[..., 2]
    v = x.max(axis=-1)
    c = v - x.min(axis=-1)
    s = np.divide(c, v, out=np.zeros_like(v), where=v > 0.0)
    safe = np.where(c > 0.0, c, 1.0)
    h = np.where(v == r, ((g - b) / safe) % 6.0,
                 np.where(v == g, (b - r) / safe + 2.0, (r - g) / safe + 4.0))
    return np.where(c > 0.0, h * 60.0, 0.0), s, v


def hue_in(h, band):
    """Hues within band = (lo, hi) degrees; lo > hi wraps through 0."""
    lo, hi = band
    return (h >= lo) & (h <= hi) if lo <= hi else (h >= lo) | (h <= hi)


def search_box(face, shape, roi):
    """The search box in frame pixels (x0, y0, x1, y1), clipped to the frame; None when the
    face is too small or the box falls off the frame."""
    H, W = shape[:2]
    fx0, fy0, fx1, fy1 = (float(v) for v in face)
    fw, fh = fx1 - fx0, fy1 - fy0
    if fw < MIN_FACE_PX or fh < MIN_FACE_PX:
        return None
    x0 = max(0, int(math.floor(fx0 + roi[0] * fw)))
    x1 = min(W, int(math.ceil(fx0 + roi[1] * fw)))
    y0 = max(0, int(math.floor(fy0 + roi[2] * fh)))
    y1 = min(H, int(math.ceil(fy0 + roi[3] * fh)))
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None
    return x0, y0, x1, y1


def read_indicator(rgb, face, holes=(), cfg=None):
    """The indicator of the bay whose face box (full-resolution px x0, y0, x1, y1) is `face`,
    read from `rgb` (H x W x 3, RGB). `holes`: the bay's window boxes, left out.

    -> dict(present, colour "red" | "green" | "unknown", confidence, box [x0, y0, x1, y1] or
    None, red, green (shares of the face), roi (the search box) or None). present is False
    when nothing coloured enough is there."""
    k = check(cfg)
    out = dict(present=False, colour="unknown", confidence=0.0, box=None, red=0.0, green=0.0,
               roi=None)
    roi = search_box(face, rgb.shape, k["roi"])
    if roi is None:
        return out
    x0, y0, x1, y1 = roi
    out["roi"] = roi
    keep = np.ones((y1 - y0, x1 - x0), bool)
    for hb in holes:
        px, py = HOLE_PAD * (hb[2] - hb[0]), HOLE_PAD * (hb[3] - hb[1])
        a0 = max(0, int(math.floor(hb[0] - px)) - x0)
        a1 = min(x1 - x0, int(math.ceil(hb[2] + px)) - x0)
        b0 = max(0, int(math.floor(hb[1] - py)) - y0)
        b1 = min(y1 - y0, int(math.ceil(hb[3] + py)) - y0)
        if a1 > a0 and b1 > b0:
            keep[b0:b1, a0:a1] = False
    h, s, v = hsv(rgb[y0:y1, x0:x1])
    vivid = keep & (s >= k["sat_min"]) & (v >= k["val_min"])
    red = vivid & hue_in(h, k["red_hue"])
    green = vivid & hue_in(h, k["green_hue"])
    n_red, n_green = int(red.sum()), int(green.sum())
    area = (float(face[2]) - float(face[0])) * (float(face[3]) - float(face[1]))
    out["red"], out["green"] = n_red / area, n_green / area
    big, small, name, mask = ((n_red, n_green, "red", red) if n_red >= n_green
                              else (n_green, n_red, "green", green))
    if big / area < k["min_share"]:
        return out
    ys, xs = np.nonzero(mask)
    out.update(
        present=True,
        colour=name if big >= k["ratio"] * small else "unknown",
        confidence=big / float(big + small),
        box=[x0 + float(np.percentile(xs, 2)), y0 + float(np.percentile(ys, 2)),
             x0 + float(np.percentile(xs, 98)) + 1.0, y0 + float(np.percentile(ys, 98)) + 1.0])
    return out
