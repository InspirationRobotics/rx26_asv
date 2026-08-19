"""
Shared helpers for buoy LED-colour classification.

Core idea
---------
The buoy hull is white, so it doubles as a per-object, per-frame white
reference. Normalising the LED patch by the hull's measured white point
removes most illumination-colour variation (sunset warmth, overcast blue,
auto-WB drift, camera-to-camera differences) *before* we ever look at hue.

Pixel selection is done with thresholds, not geometry, so we never have to
segment the black diamond explicitly:
    hull   = low saturation AND mid-to-high value   (diamond fails the value
                                                     floor, LED fails the
                                                     saturation ceiling)
    LED    = high saturation AND not blown out      (the clipped white core of
                                                     a bright LED is excluded;
                                                     the colour lives in the
                                                     bloom around it)

Everything here is numpy/OpenCV and runs in tens of microseconds per buoy.
"""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

# --------------------------------------------------------------------------
# tunables (sane defaults; the calibration step does not need you to touch these)
# --------------------------------------------------------------------------
HULL_SAT_MAX = 70      # hull is desaturated
HULL_VAL_PCT = 60      # keep only the brighter 40% of body pixels -> drops
                       # the black diamond and deep shadow
VAL_CLIP = 250         # anything above this is sensor clipping
LED_SAT_MIN = 70
LED_SAT_MARGIN = 40   # an LED must out-saturate the hull beside it by this much
LED_VAL_MIN = 50
LED_VAL_MAX = 252
BAND_FRAC = 0.22       # fraction of the crop treated as the LED band
BAND_OFFSET = 0.00     # push the band DOWN by this fraction of the crop


# --------------------------------------------------------------------------
# label parsing (Roboflow YOLO export: detection boxes OR segmentation polygons)
# --------------------------------------------------------------------------
def read_yolo_label(path, img_w, img_h):
    """Parse a YOLO .txt. Returns a list of dicts with 'cls', 'poly', 'box'.

    Handles both `cls cx cy w h` (detection) and
    `cls x1 y1 x2 y2 ...` (segmentation polygon) lines.
    """
    path = Path(path)
    out = []
    if not path.exists():
        return out
    for line in path.read_text().strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cls = int(float(parts[0]))
        vals = np.asarray([float(v) for v in parts[1:]], dtype=np.float32)
        if len(vals) == 4:
            cx, cy, bw, bh = vals
            x1, x2 = (cx - bw / 2) * img_w, (cx + bw / 2) * img_w
            y1, y2 = (cy - bh / 2) * img_h, (cy + bh / 2) * img_h
            poly = np.asarray([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], np.float32)
        else:
            if len(vals) % 2:
                vals = vals[:-1]
            poly = vals.reshape(-1, 2) * np.asarray([img_w, img_h], np.float32)
            x1, y1 = poly.min(0)
            x2, y2 = poly.max(0)
        out.append(
            {
                "cls": cls,
                "poly": poly,
                "box": (float(x1), float(y1), float(x2), float(y2)),
            }
        )
    return out


def estimate_tilt(sub, min_deg=8.0, max_deg=30.0):
    """Estimate buoy tilt from the picture, for labels that carry no orientation.

    A detection-format export gives an axis-aligned box, which says nothing
    about how the buoy is rolled. But the hull is a big bright blob, so its own
    minAreaRect recovers the tilt. Refuses implausible answers (a rocking boat
    rolls; it does not lie on its side) and returns 0 rather than guessing.
    """
    if sub is None or sub.size == 0 or min(sub.shape[:2]) < 12:
        return 0.0
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    S = hsv[..., 1].astype(np.float32)
    V = hsv[..., 2].astype(np.float32)
    m = ((V >= np.percentile(V, 65)) & (S <= np.percentile(S, 65))).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return 0.0
    c = max(cnts, key=cv2.contourArea)
    # demand a big, convincing blob before believing any rotation at all
    if cv2.contourArea(c) < 0.25 * m.size or len(c) < 5:
        return 0.0
    ang = cv2.minAreaRect(c)[2] % 90.0
    if ang > 45.0:
        ang -= 90.0
    return float(ang) if min_deg <= abs(ang) <= max_deg else 0.0


# --------------------------------------------------------------------------
# geometry: get the buoy into its own upright frame
# --------------------------------------------------------------------------
def upright_crop(img, poly, pad=0.10, use_rotation=True, with_mask=False):
    """Deskew a buoy so that 'top' means the top *of the buoy*, not of the image.

    Matters as soon as the boat rolls. With a segmentation polygon this uses
    minAreaRect; with a plain box it degenerates to an axis-aligned crop.

    with_mask=True also returns the polygon rasterised into the same frame, so
    downstream steps can restrict themselves to actual buoy pixels rather than
    the water in the corners of the crop.
    """
    poly = np.asarray(poly, np.float32)
    if use_rotation and len(poly) > 4:
        (cx, cy), (w, h), ang = cv2.minAreaRect(poly)
        ang = ang % 180.0
        if ang >= 90.0:
            ang -= 180.0
        if ang > 45.0:
            ang -= 90.0
            w, h = h, w
        elif ang < -45.0:
            ang += 90.0
            w, h = h, w
    else:
        x1, y1 = poly.min(0)
        x2, y2 = poly.max(0)
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        w, h, ang = float(x2 - x1), float(y2 - y1), 0.0

    W = max(8, int(round(w * (1 + 2 * pad))))
    H = max(8, int(round(h * (1 + 2 * pad))))
    M = cv2.getRotationMatrix2D((float(cx), float(cy)), float(ang), 1.0)
    M[0, 2] += W / 2.0 - cx
    M[1, 2] += H / 2.0 - cy
    crop = cv2.warpAffine(
        img, M, (W, H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE
    )
    if not with_mask:
        return crop
    full = np.zeros(img.shape[:2], np.uint8)
    cv2.fillPoly(full, [poly.astype(np.int32)], 255)
    mask = cv2.warpAffine(full, M, (W, H), flags=cv2.INTER_NEAREST)
    return crop, mask


def orient_crop(crop, band_frac=BAND_FRAC, mask=None):
    """minAreaRect has a 180-degree ambiguity. The LED end is the saturated end."""
    n = max(4, int(round(crop.shape[0] * band_frac)))
    top = cv2.cvtColor(crop[:n], cv2.COLOR_BGR2HSV)[..., 1]
    bot = cv2.cvtColor(crop[-n:], cv2.COLOR_BGR2HSV)[..., 1]
    if float(np.percentile(bot, 95)) > float(np.percentile(top, 95)) * 1.25:
        crop = cv2.rotate(crop, cv2.ROTATE_180)
        if mask is not None:
            mask = cv2.rotate(mask, cv2.ROTATE_180)
    return crop if mask is None else (crop, mask)


def led_band(crop, band_frac=BAND_FRAC):
    n = max(4, int(round(crop.shape[0] * band_frac)))
    return crop[:n]


# --------------------------------------------------------------------------
# white balance from the hull itself
# --------------------------------------------------------------------------
def find_led_strip(crop, mask, sat_floor, search_frac=0.45, min_px=40):
    """UNUSED -- kept for reference. Do not wire this back in without testing
    against a saturated background (terracotta deck, brick, red hull), which is
    exactly what defeated it: those blobs are far larger than an LED strip and
    win any size-weighted score.

    Locate the LED strip directly, as the saturated blob near the buoy top.

    A fixed "top 22% of the crop" band is fragile in two ways the user sees
    immediately: padding and annotation slack push it above the strip, and any
    residual tilt makes a horizontal slice clip the strip diagonally and fill
    the rest with hull.

    The strip is the one strongly coloured thing up there, so find it instead
    of assuming where it is. Returns a boolean mask over the FULL crop, or None
    when nothing qualifies (an unlit LED, correctly, produces nothing).
    """
    H = crop.shape[0]
    h = max(6, int(round(H * search_frac)))
    sub = crop[:h]
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    S = hsv[..., 1].astype(np.float32)
    V = hsv[..., 2].astype(np.float32)

    cand = (S > sat_floor) & (V > LED_VAL_MIN)
    if mask is not None and mask.shape[0] >= h:
        mm = mask[:h] > 127
        if mm.shape == cand.shape and int(mm.sum()) >= 30:
            cand &= mm
    if int(cand.sum()) < min_px:
        return None

    u8 = cv2.morphologyEx(cand.astype(np.uint8), cv2.MORPH_CLOSE,
                          np.ones((3, 3), np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(u8, 8)
    if n <= 1:
        return None

    # Prefer big AND high. A saturated float ring or fender low in the search
    # window should not beat the strip sitting at the very top.
    best, best_score = 0, -1.0
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_px:
            continue
        top_bias = 1.0 - (cent[i][1] / max(h, 1))
        score = area * (0.35 + top_bias)
        if score > best_score:
            best, best_score = i, score
    if best == 0:
        return None

    out = np.zeros(crop.shape[:2], bool)
    out[:h] = lab == best
    # a couple of rows of slack: the strip's own glow carries colour too
    out[:h] = cv2.dilate(out[:h].astype(np.uint8),
                         np.ones((3, 3), np.uint8)).astype(bool)
    if mask is not None and mask.shape[:2] == out.shape:
        keep = mask > 127
        if int((out & keep).sum()) >= min_px:
            out &= keep
    return out if int(out.sum()) >= min_px else None


def hull_white_point(crop, mask=None, exclude_top_frac=0.30, local=True,
                     return_sat=False):
    """Median BGR of white-hull pixels -> illuminant estimate for THIS buoy.

    `local=True` biases the sample toward the hull just below the LED box, so
    a partially shaded buoy is normalised by light that actually falls on the
    LED region rather than by an average over sun and shade.

    IMPORTANT: hull pixels are selected by *relative brightness*, never by an
    absolute saturation threshold. Under a strongly coloured illuminant (low
    sun, sodium dock lights) a white hull is genuinely saturated in raw pixel
    values -- an "S < k" test rejects exactly the pixels we need, and the
    correction silently turns itself off in the conditions it exists for.
    Brightness rank is illuminant-invariant; saturation is not.

    We take a band (55th-97th percentile of value), not "the brightest N%":
    the brightest hull pixels are the ones most likely clipped at 255, and a
    clipped pixel under-reports whichever channel saturated.
    """
    H = crop.shape[0]
    h0 = int(H * exclude_top_frac)
    h1 = int(H * 0.75) if local else H
    if h1 - h0 < 3:
        h0, h1 = int(H * exclude_top_frac), H
    body = crop[h0:h1]
    if body.size == 0:
        return None

    hsv = cv2.cvtColor(body, cv2.COLOR_BGR2HSV)
    S = hsv[..., 1].astype(np.float32)
    V = hsv[..., 2].astype(np.float32)

    # restrict to real buoy pixels when a segmentation mask is available,
    # so bright water or sun glitter in the crop corners cannot poison the
    # white reference
    valid = np.ones(S.shape, bool)
    if mask is not None:
        mm = mask[h0:h1] > 127
        if mm.shape == S.shape and int(mm.sum()) >= 40:
            valid = mm

    vv = V[valid]
    if vv.size < 40:
        return None
    lo = float(np.percentile(vv, 55))
    hi = float(np.percentile(vv, 97))

    m = valid & (V >= lo) & (V <= hi)
    unclipped = m & (V <= VAL_CLIP)
    if int(unclipped.sum()) >= 40:
        m = unclipped
    if int(m.sum()) >= 100:  # drop the most saturated fifth (LED bleed, reflections)
        m = m & (S <= float(np.percentile(S[m], 80)))
    if int(m.sum()) < 30:
        return None
    if return_sat:
        return (np.median(body[m].astype(np.float32), axis=0), float(np.median(S[m])))
    return np.median(body[m].astype(np.float32), axis=0)


def apply_white_balance(img, wp):
    if wp is None:
        return img
    gain = float(np.mean(wp)) / np.clip(np.asarray(wp, np.float32), 1.0, None)
    return np.clip(img.astype(np.float32) * gain.reshape(1, 1, 3), 0, 255).astype(
        np.uint8
    )


# --------------------------------------------------------------------------
# LED colour measurement
# --------------------------------------------------------------------------
def measure_led(crop, mask=None, band_frac=BAND_FRAC, orient=True,
                band_offset=BAND_OFFSET):
    """Measure the LED band's hue after per-buoy white balance.

    Returns dict:
        hue    circular, saturation*value weighted mean hue in degrees (or None)
        conc   0..1 concentration of the hue distribution -> a confidence proxy
        sat    mean saturation of the LED pixels
        n      number of LED pixels used
        white_px  bright-but-unsaturated pixel count (white LED / blown out)
    """
    if crop is None or crop.size == 0 or crop.shape[0] < 8:
        return None
    if orient:
        r = orient_crop(crop, band_frac, mask)
        crop, mask = r if mask is not None else (r, None)

    ref = hull_white_point(crop, mask, return_sat=True)
    wp, hull_sat = ref if ref is not None else (None, None)

    # Fixed measurement window: a band of the crop, with an adjustable offset.
    # An earlier version tried to FIND the strip as the largest saturated blob
    # near the top. It failed badly on real frames: a terracotta pool deck is
    # large and saturated, so it outscored the LED and the window wandered onto
    # the background. Predictable-and-tunable beats clever-and-wrong here.
    H_ = crop.shape[0]
    r0 = max(0, min(H_ - 4, int(round(H_ * band_offset))))
    r1 = min(H_, max(r0 + 4, r0 + int(round(H_ * band_frac))))
    raw = crop[r0:r1]
    band = apply_white_balance(raw, wp)

    # CRITICAL: select LED pixels on the RAW band, measure hue on the balanced
    # one. White balance MANUFACTURES saturation -- a perfectly grey hull pixel
    # (100,100,100) under a bluish white point comes out at S=119, hue 26 deg.
    # Selecting on balanced pixels therefore invents orange "LED" pixels out of
    # neutral hull, and since the band holds far more hull than LED, they win
    # the vote. Raw saturation is the honest question: was this pixel actually
    # coloured in the sensor?
    hsv_raw = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV)
    S = hsv_raw[..., 1].astype(np.float32)
    V = hsv_raw[..., 2].astype(np.float32)
    H = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)[..., 0].astype(np.float32) * 2.0

    # Tiered pixel selection. Tier 0 is the clean case: saturated, unclipped
    # bloom around the emitter. A very bright LED clips its dominant channel,
    # which pushes V to 255 and empties tier 0 -- so fall through rather than
    # reporting "no LED found", and mark the result lower quality.
    # Restrict the search to actual buoy pixels. Without this the band's outer
    # corners -- brick deck, pool water, sky -- get counted as "LED", and a
    # saturated background trivially outvotes a small strip.
    #
    # But the polygon mask is only usable if the polygon ENCLOSES the LED box.
    # Plenty of real annotations trace just the white hull, and then the mask
    # excludes the very thing we are measuring. So try regions in order of
    # precision and take the first that actually finds an LED.
    regions = []
    nb = band.shape[0]
    if mask is not None and mask.shape[0] >= r1:
        mm = (mask[r0:r1] > 127).astype(np.uint8)
        k = 3 if min(mm.shape) >= 24 else 2
        er = cv2.erode(mm, np.ones((k, k), np.uint8))   # drop anti-aliased rim
        if int(er.sum()) >= 30:
            mm = er
        mmb = mm.astype(bool)
        if mmb.shape == S.shape and int(mmb.sum()) >= 30:
            regions.append(mmb)
    centre = np.zeros(S.shape, bool)
    w_ = S.shape[1]
    centre[:, int(w_ * 0.18):max(int(w_ * 0.82), int(w_ * 0.18) + 1)] = True
    regions.append(centre)

    sat_floor = LED_SAT_MIN
    if hull_sat is not None:
        sat_floor = max(LED_SAT_MIN * 0.55, float(hull_sat) + LED_SAT_MARGIN)

    # Best region of all: the strip located by appearance rather than assumed
    # by geometry. Immune to residual tilt and to the band sitting too high.

    def tiers(region):
        def sel(smin, vmax):
            q = (S > smin) & (V > LED_VAL_MIN)
            if vmax is not None:
                q &= V < vmax
            return q & region
        a = sel(sat_floor, LED_VAL_MAX)
        # A bright LED clips its dominant channel, so the body fails the "not
        # blown out" test and tier 0 holds only the anti-aliased rim -- enough
        # pixels to pass a bare count check, every one a blend with background.
        # If the clipped-but-saturated population is larger, the body IS signal.
        clipped = sel(sat_floor, None) & ~a
        if int(a.sum()) < 15 or int(clipped.sum()) > int(a.sum()):
            return sel(sat_floor, None), 1
        return a, 0

    m, tier = tiers(regions[0])
    for r in regions[1:]:
        if int(m.sum()) >= 15:
            break
        m, tier = tiers(r)
    if int(m.sum()) < 15:
        m = (S > sat_floor * 0.7) & (V > LED_VAL_MIN) & regions[-1]
        tier = 2

    # LED-box brightness and saturation, relative to the hull. These are what
    # the OFF class is calibrated on -- measured over the best available region.
    v_ratio = None
    sat_med = None
    if wp is not None:
        hull_v = float(np.max(wp))
        reg = regions[0]
        if int(reg.sum()) < 10:
            reg = regions[-1]
        if hull_v > 5 and int(reg.sum()) >= 10:
            v_ratio = float(np.median(V[reg]) / hull_v)
            sat_med = float(np.median(S[reg]))

    n = int(m.sum())
    if n < 15:
        bright = int(((V > 200) & (S <= sat_floor)).sum())
        return {
            "hue": None, "conc": 0.0, "sat": float(S.mean()),
            "n": n, "white_px": bright, "quality": 0.0, "tier": 3,
            "v_ratio": v_ratio, "sat_med": sat_med, "rows": (int(r0), int(r1)), "rows": (int(r0), int(r1)),
        }

    # weight by saturation * value, but cap value so clipped pixels do not
    # dominate the circular mean
    wgt = S[m] * np.minimum(V[m], 250.0)
    ang = np.deg2rad(H[m])
    x = float((wgt * np.cos(ang)).sum())
    y = float((wgt * np.sin(ang)).sum())
    hue = float(np.degrees(np.arctan2(y, x)) % 360.0)
    conc = float(np.hypot(x, y) / max(float(wgt.sum()), 1e-6))
    return {
        "hue": hue, "conc": conc, "sat": float(S[m].mean()),
        "n": n, "white_px": 0,
        "quality": (1.0, 0.8, 0.6)[tier], "tier": tier,
        "v_ratio": v_ratio, "sat_med": sat_med, "rows": (int(r0), int(r1)),
    }


# --------------------------------------------------------------------------
# circular statistics
# --------------------------------------------------------------------------
def circ_dist(a, b):
    d = abs(float(a) - float(b)) % 360.0
    return min(d, 360.0 - d)


def robust_circ_fit(hues, weights=None, max_dev=45.0, iters=4):
    """Circular mean/std that ignores outliers.

    A plain mean+std is hopeless here: ONE bad sample in thirty takes the
    fitted tolerance from 12 degrees to 51. And bad samples are guaranteed --
    you label what your eye sees in the left panel, but a buoy that is far
    away, dim, or half-occluded measures garbage, and that garbage goes into
    the fit at full weight.

    Seeds from the densest cluster (the mode) rather than the mean, because
    the mean is itself already dragged by the outliers we are trying to find.

    Returns (mean, std, keep_mask).
    """
    h = np.asarray(hues, np.float32)
    w = np.ones_like(h) if weights is None else np.asarray(weights, np.float32)
    n = len(h)
    if n == 0:
        return 0.0, 0.0, np.zeros(0, bool)
    if n < 4:
        m, s = circ_mean_std(h, w)
        return m, s, np.ones(n, bool)

    # seed: the sample with the most neighbours within max_dev
    d = np.abs(h[:, None] - h[None, :]) % 360.0
    d = np.minimum(d, 360.0 - d)
    seed = int((d <= max_dev).sum(axis=1).argmax())
    mean = float(h[seed])

    keep = np.ones(n, bool)
    for _ in range(iters):
        dev = np.abs(h - mean) % 360.0
        dev = np.minimum(dev, 360.0 - dev)
        nk = dev <= max_dev
        if int(nk.sum()) < 3:
            break
        if np.array_equal(nk, keep) and _ > 0:
            break
        keep = nk
        mean, std = circ_mean_std(h[keep], w[keep])
    mean, std = circ_mean_std(h[keep], w[keep])
    return mean, std, keep


def pick_off_feature(off_vals, lit_vals):
    """Choose whichever measurement separates OFF from lit best.

    Brightness is the obvious signal ("an off LED is dark") but it is wrong for
    a translucent white diffuser housing, which stays bright when unlit. For
    that hardware SATURATION is the discriminator: bright-and-grey vs
    bright-and-coloured. Rather than guess which your buoys are, measure both
    and keep the one with the wider gap.

    Returns (name, threshold, gap) with gap in [0,1]; gap <= 0 means overlap.
    """
    best = (None, None, -9e9)
    for name in ("sat_med", "v_ratio"):
        o = [d[name] for d in off_vals if d.get(name) is not None]
        l = [d[name] for d in lit_vals if d.get(name) is not None]
        if len(o) < 3 or len(l) < 3:
            continue
        hi_off = float(np.percentile(o, 90))
        lo_lit = float(np.percentile(l, 10))
        span = max(float(np.percentile(l, 90)) - min(o + l), 1e-6)
        gap = (lo_lit - hi_off) / span
        thr = (hi_off + lo_lit) / 2 if lo_lit > hi_off else hi_off + 0.02 * span
        if gap > best[2]:
            best = (name, float(thr), float(gap))
    return best


def circ_mean_std(hues, weights=None):
    hues = np.asarray(hues, np.float32)
    w = np.ones_like(hues) if weights is None else np.asarray(weights, np.float32)
    ang = np.deg2rad(hues)
    x = float((w * np.cos(ang)).sum())
    y = float((w * np.sin(ang)).sum())
    mean = float(np.degrees(np.arctan2(y, x)) % 360.0)
    R = np.hypot(x, y) / max(float(w.sum()), 1e-6)
    std = float(np.degrees(np.sqrt(max(-2.0 * np.log(max(R, 1e-6)), 0.0))))
    return mean, std


# --------------------------------------------------------------------------
# classifier
# --------------------------------------------------------------------------
class HsvClassifier:
    """Nearest hue centre in circular distance, gated by a per-class tolerance."""

    def __init__(self, cfg):
        self.cfg = cfg

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text()))

    def save(self, path):
        Path(path).write_text(json.dumps(self.cfg, indent=2))

    def classify(self, meas):
        """-> (class_name or None, confidence 0..1)"""
        if not meas:
            return None, 0.0

        # An LED that is OFF is dark relative to the hull. Check this BEFORE the
        # hue path: an off LED sometimes scrapes together enough noisy saturated
        # pixels to report a bogus hue, and dark-vs-hull is the stronger signal.
        oc = self.cfg.get("off_class")
        if oc:
            feat = self.cfg.get("off_feature", "v_ratio")
            thr = self.cfg.get("off_thresh", self.cfg.get("off_max_ratio", 0.55))
            val = meas.get(feat)
            if val is not None and val < thr:
                scale = 60.0 if feat == "sat_med" else 0.25
                return oc, float(round(min(1.0, 0.5 + (thr - val) / scale), 4))

        if meas["hue"] is None:
            wc = self.cfg.get("white_class")
            if wc and meas.get("white_px", 0) >= self.cfg.get("white_min_px", 40):
                return wc, 0.5
            return None, 0.0

        best, bd = None, 1e9
        for name, c in self.cfg["classes"].items():
            d = circ_dist(meas["hue"], c["hue"])
            if d < bd:
                best, bd = name, d
        if best is None:
            return None, 0.0

        tol = float(self.cfg["classes"][best]["tol"])
        if bd > tol:
            return None, 0.0
        conf = (
            max(0.0, 1.0 - bd / max(tol, 1e-6))
            * min(1.0, meas["conc"] * 1.5)
            * meas.get("quality", 1.0)
        )
        return best, float(round(conf, 4))


# --------------------------------------------------------------------------
# convenience: image + polygon -> (class, conf, measurement, crop)
# --------------------------------------------------------------------------
def buoy_crop(img, poly, band_frac=BAND_FRAC, use_rotation=True, top_pad=0.0,
              with_mask=None, allow_flip=False, content_tilt=False):
    """Deskewed, correctly-oriented buoy crop -- the one entry point to use.

    Resolves the 180-degree flip BEFORE applying `top_pad`, which a naive
    crop-then-flip would get wrong (the pad would end up at the bottom).

    top_pad extends the crop ABOVE the polygon by that fraction of buoy height.
    Set it if your Roboflow polygons trace only the white hull and leave the LED
    box outside; leave it at 0 if the polygon already encloses the LED, which is
    what you want to be annotating.
    """
    poly = np.asarray(poly, np.float32)
    if with_mask is None:
        with_mask = len(poly) > 4  # a 4-point box mask is just the rect: useless

    ang = 0.0
    if use_rotation and len(poly) > 4:
        (cx, cy), _, a = cv2.minAreaRect(poly)
        # A rectangle's orientation is only defined modulo 90 degrees, so map
        # into (-45, 45] -- the smallest rotation that axis-aligns it. Do NOT
        # pick the long axis as "up": these buoys are squat, wider than tall,
        # and that heuristic tips them onto their side. Image-up is the right
        # prior for a camera that rolls but does not tumble.
        ang = a % 90.0
        if ang > 45.0:
            ang -= 90.0
    elif use_rotation and content_tilt:
        # OPT-IN ONLY (content_tilt=False by default), and for good reason.
        #
        # An axis-aligned box carries no orientation, so this guesses one from
        # the picture. On real frames it guesses wrong: the box also contains
        # pool deck, water and a pink float ring, and the "bright hull" blob it
        # measures is often not the hull. The result was upright buoys being
        # rotated 20-30 degrees for no reason -- which then clips the LED strip
        # diagonally, exactly what the deskew was supposed to prevent.
        #
        # An axis-aligned crop of an upright buoy is already upright. A boat
        # rolls a little; it does not tumble. Doing nothing is the better
        # default, and it is predictable.
        x1, y1 = poly.min(0)
        x2, y2 = poly.max(0)
        sub = img[max(0, int(y1)):int(y2), max(0, int(x1)):int(x2)]
        ang = estimate_tilt(sub)

    if len(poly) > 4:
        (cx, cy), _, _ = cv2.minAreaRect(poly)
        cx, cy = float(cx), float(cy)
        # Derive w/h from the polygon AFTER derotating it. Reading them off
        # minAreaRect and swapping alongside the angle depends on OpenCV's
        # (version-dependent) w/h convention, and getting it wrong transposes
        # the crop -- which put the "top band" in the sky above the buoy.
        M0 = cv2.getRotationMatrix2D((cx, cy), ang, 1.0)
        pr = cv2.transform(poly.reshape(-1, 1, 2), M0).reshape(-1, 2)
        w = float(pr[:, 0].max() - pr[:, 0].min())
        h = float(pr[:, 1].max() - pr[:, 1].min())
    else:
        x1, y1 = poly.min(0)
        x2, y2 = poly.max(0)
        cx, cy = float((x1 + x2) / 2), float((y1 + y2) / 2)
        if ang:
            # Derotating a box inflates its extents a little; that slack is
            # better than leaving the buoy tilted inside the band.
            M0 = cv2.getRotationMatrix2D((cx, cy), ang, 1.0)
            pr = cv2.transform(poly.reshape(-1, 1, 2), M0).reshape(-1, 2)
            w = float(pr[:, 0].max() - pr[:, 0].min())
            h = float(pr[:, 1].max() - pr[:, 1].min())
        else:
            w, h = float(x2 - x1), float(y2 - y1)

    def warp(angle, extra_top):
        W = max(8, int(round(w * 1.2)))
        H0 = max(8, int(round(h * 1.2)))
        e = max(0, int(round(h * extra_top)))
        M = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
        M[0, 2] += W / 2.0 - cx
        M[1, 2] += H0 / 2.0 - cy + e
        c = cv2.warpAffine(img, M, (W, H0 + e), flags=cv2.INTER_LINEAR,
                           borderMode=cv2.BORDER_REPLICATE)
        if not with_mask:
            return c, None
        full = np.zeros(img.shape[:2], np.uint8)
        cv2.fillPoly(full, [poly.astype(np.int32)], 255)
        return c, cv2.warpAffine(full, M, (W, H0 + e), flags=cv2.INTER_NEAREST)

    # The 180-degree flip check is OFF by default. With the angle normalised to
    # +/-45 the crop is already close to image-up, and a saturated background
    # (brick deck, pink float ring, sunlit water) below the buoy will happily
    # out-saturate a dim LED and flip a perfectly good crop upside down.
    if allow_flip:
        probe, _ = warp(ang, 0.0)
        n = max(4, int(round(probe.shape[0] * band_frac)))
        s_top = float(np.percentile(cv2.cvtColor(probe[:n], cv2.COLOR_BGR2HSV)[..., 1], 95))
        s_bot = float(np.percentile(cv2.cvtColor(probe[-n:], cv2.COLOR_BGR2HSV)[..., 1], 95))
        if s_bot > s_top * 1.5:
            ang += 180.0

    return warp(ang, top_pad)


def classify_instance(img, poly, clf, band_frac=BAND_FRAC, use_rotation=True,
                      top_pad=0.0, band_offset=BAND_OFFSET):
    crop, mask = buoy_crop(img, poly, band_frac, use_rotation, top_pad)
    meas = measure_led(crop, mask=mask, band_frac=band_frac, orient=False,
                       band_offset=band_offset)
    name, conf = clf.classify(meas)
    return name, conf, meas, crop


IMG_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def list_images(d):
    d = Path(d)
    return sorted(p for p in d.rglob("*") if p.suffix.lower() in IMG_EXTS)
