"""LED colour measurement and classification, with no network in it.

This is the on-boat twin of tools/buoy_color/buoylib.py's runtime half. The two
MUST measure the same pixels: hsv_config.json is fitted off-boat against
buoylib, and a config that describes a region this file never looks at is a
config that describes a picture nobody classifies.

  measured on the 210-instance val split, 2026-08-25
  ------------------------------------------------------------------
  shipped config (green 188.8 deg, off = sat_med < 45)        54.6%
  green refit to 164 deg, off still saturation-based          54.6%
  green 164 deg, off = "hue is not near any LED colour"       84.8%

The third row is what this file implements. The change that mattered was not
the hue centre -- it was throwing the saturation-based off test away.

WHY SATURATION CANNOT DECIDE "OFF"
----------------------------------
Median saturation of the LED region, minus the same buoy's own hull:

    red    +28      blue   +15      green   +6      OFF   +6

A lit green LED and an unlit buoy are the same number. No threshold, absolute
or hull-relative, separates them -- and an absolute one throws away half the
blues as well (30 of 69 on the val split).

WHAT DOES WORK
--------------
An unlit buoy is not colourless; it is faintly WARM. All 18 unlit instances in
the val split measured a hue of 4-27 deg or 352-359 deg -- white plastic under
warm ambient. That sits 23-38 deg away from the red LED at 348.7 deg, while a
lit LED lands a median of 6 deg from its own centre. So the question that
separates them is not "how colourful is this" but "is this hue near any colour
an LED can actually make". That is the gate below.

It costs nothing that saturation was buying: unlit buoys are still caught (83%
recall here), and red and blue come back at 100% and 98.6%.

WHAT IS STILL BROKEN, AND WHY IT IS NOT A SOFTWARE PROBLEM
----------------------------------------------------------
Green recall is 57.6%. Fitted mean chromaticity of the LED pixels, as a
fraction of total intensity (0.333 each = pure grey):

    red    b 0.236  g 0.188  r 0.577      <- 0.24 off neutral
    blue   b 0.390  g 0.340  r 0.270      <- 0.06 off neutral
    green  b 0.341  g 0.349  r 0.310      <- 0.016 off neutral
    OFF    b 0.303  g 0.324  r 0.372

The green LED is recorded as grey with a rumour of green in it. Hue is an ANGLE
around the grey axis, so the closer a pixel sits to that axis the more sensor
noise swings the angle -- which is exactly why green's readings scatter (circular
concentration R = 0.61, against 1.00 for red and 0.99 for blue) while red and
blue are essentially solved.

That colour is not in the recorded pixels, so no measurement here can recover
it. It has to be captured: shoot the scene darker so the sunlit white hull comes
down and the self-luminous LED stands out. The acceptance test is one number --
green's median (sat_med - hull_sat), currently +6. Get it to +15, where blue
already is, and green stops being the weak class. Until then the flash tracker
carries green on votes: it never appears as a steady light in RobotX Task 1, so
a track that flashes and reads green-ish over several seconds is green.
"""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

# -- hull sampling -----------------------------------------------------------
VAL_CLIP = 250          # at/above this a channel may be clipped and under-reports

# -- LED pixel selection -----------------------------------------------------
LED_SAT_MIN = 70
LED_SAT_MARGIN = 40     # an LED must out-saturate the hull beside it by this much
LED_VAL_MIN = 50
LED_VAL_MAX = 252

# -- LED box geometry. MUST match buoylib's LED_FRAC / LED_APEX / LED_WIDTH --
LED_FRAC = 0.25         # rows: top fraction OF THE BUOY handed to the classifier
LED_APEX = 0.15         # cols: width is taken from the top of the buoy only
LED_WIDTH = 0.62        # ...then narrowed to this centred fraction of it


def circ_dist(a, b):
    d = abs(float(a) - float(b)) % 360.0
    return min(d, 360.0 - d)


# ---------------------------------------------------------------------------
# the LED box
# ---------------------------------------------------------------------------
def led_box(crop, mask=None, frac=LED_FRAC, apex=LED_APEX,
            width=LED_WIDTH, pad=0.10):
    """The LED box, cropped tight -- what the colour stage should be fed.

    Not "the top 40% of the bounding box, full width". A buoy is a truncated
    pyramid: wide at the base, narrow at the top, LED on the top face. A
    full-width top band frames mostly sky and deck -- measured on 29 real
    instances the old export was 27% buoy and 73% background, so the hue
    centres got fitted to a patio.

      1. Rows start at the MASK's top, not the crop's. Padding plus annotation
         slack pushes a crop-relative band up into the sky.
      2. Columns come from the APEX only (top `apex` of the buoy). Taking the
         extent over the whole band lets the widest row set the width, and on a
         trapezoid that is the bottom row.
      3. `frac` decides how far down to go. 0.25 keeps a little hull under the
         LED, which is the white reference and worth having in view.

    Same 29 instances end to end: 27% -> 69% buoy.

    With no mask the column extent comes from the bright hull instead -- the
    same question, asked of the pixels rather than the polygon.
    """
    H, W = crop.shape[:2]
    r0 = 0
    cols = None
    rb = None

    if mask is not None and mask.shape[:2] == crop.shape[:2]:
        mm = mask > 127
        rr = np.flatnonzero(mm.sum(1) > 0)
        if rr.size >= 6:
            r0 = int(rr[0])
            bh = int(rr[-1]) - r0 + 1
            ar = min(H, r0 + max(4, int(round(bh * apex))))
            cc = np.flatnonzero(mm[r0:ar].sum(0) > 0)
            if cc.size >= 4:
                cols = (int(cc[0]), int(cc[-1]))
                rb = min(H, r0 + max(6, int(round(bh * frac))))

    if cols is None:
        r0 = 0
        n = max(6, int(round(H * apex)))
        g = cv2.cvtColor(crop[:n], cv2.COLOR_BGR2GRAY)
        cc = np.flatnonzero((g > np.percentile(g, 65)).sum(0) > 0.35 * n)
        cols = (int(cc[0]), int(cc[-1])) if cc.size >= 4 else (0, W - 1)
        rb = max(8, int(round(H * frac)))

    # Shrink toward the centre. The polygon cannot localise the LED any further:
    # these buoys have a wide FLAT top and the LED sits in the middle of it, so
    # there is no narrow apex to find. The last step is a fixed ratio tied to
    # the hardware -- the LED box is roughly 60% of the top face, and centred.
    x0, x1 = cols
    cx = (x0 + x1) / 2.0
    half = (x1 - x0) * max(0.15, min(1.0, width)) / 2.0
    x0, x1 = int(round(cx - half)), int(round(cx + half))

    mh = int((x1 - x0) * pad)
    mv = int((rb - r0) * pad)
    return (max(0, x0 - mh), max(0, r0 - mv),
            min(W, x1 + mh + 1), min(H, rb + mv))


def led_crop(crop, mask=None, **kw):
    x0, y0, x1, y1 = led_box(crop, mask, **kw)
    return crop[y0:y1, x0:x1]


# ---------------------------------------------------------------------------
# white balance from the buoy's own hull
# ---------------------------------------------------------------------------
def hull_white_point(crop, mask=None, exclude_top_frac=0.30, local=True,
                     return_sat=False):
    """Median BGR of white-hull pixels -> this buoy's illuminant, this frame.

    Hull pixels are picked by relative BRIGHTNESS, never by an absolute
    saturation threshold: under a strongly coloured illuminant a white hull is
    genuinely saturated in raw values, so an "S < k" test rejects exactly the
    pixels the correction needs and switches itself off in the conditions it
    exists for.

    The sample is the 55th-97th percentile band rather than "the brightest N%":
    the brightest hull pixels are the ones most likely clipped, and a clipped
    pixel under-reports whichever channel saturated.
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

    valid = np.ones(S.shape, bool)
    if mask is not None:
        mm = mask[h0:h1] > 127
        if mm.shape == S.shape and int(mm.sum()) >= 40:
            valid = mm
    vv = V[valid]
    if vv.size < 40:
        return None

    m = valid & (V >= np.percentile(vv, 55)) & (V <= np.percentile(vv, 97))
    unclipped = m & (V <= VAL_CLIP)
    if int(unclipped.sum()) >= 40:
        m = unclipped
    if int(m.sum()) >= 100:            # drop the most saturated fifth: LED bleed
        m = m & (S <= float(np.percentile(S[m], 80)))
    if int(m.sum()) < 30:
        return None
    wp = np.median(body[m].astype(np.float32), axis=0)
    return (wp, float(np.median(S[m]))) if return_sat else wp


def apply_white_balance(img, wp):
    if wp is None:
        return img
    gain = float(np.mean(wp)) / np.clip(np.asarray(wp, np.float32), 1.0, None)
    return np.clip(img.astype(np.float32) * gain.reshape(1, 1, 3),
                   0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# measurement
# ---------------------------------------------------------------------------
def measure_led(crop, mask=None, frac=LED_FRAC, apex=LED_APEX, width=LED_WIDTH):
    """-> {hue, conc, sat, n, quality, tier, v_ratio, sat_med, hull_sat, box}.

    Measures EXACTLY the region led_box() defines. Calibration, auto-labelling
    and the runtime have to look at the same pixels or the config describes a
    picture that is never actually classified.

    Returns None only when there is no crop to work with. A crop with nothing
    LED-like in it still comes back, with hue=None -- "I looked and found
    nothing" is a different answer from "I could not look", and the flash
    tracker needs to tell them apart.
    """
    if crop is None or crop.size == 0 or crop.shape[0] < 8:
        return None

    ref = hull_white_point(crop, mask, return_sat=True)
    wp, hull_sat = ref if ref is not None else (None, None)

    bx0, by0, bx1, by1 = led_box(crop, mask, frac, apex, width)
    raw = crop[by0:by1, bx0:bx1]
    if raw.size == 0:
        return None
    band = apply_white_balance(raw, wp)

    # Select on RAW saturation, measure hue on the BALANCED band. White balance
    # manufactures saturation -- a grey hull pixel under a bluish white point
    # comes out S=119 at hue 26 deg -- so selecting on balanced pixels invents
    # orange "LED" pixels out of neutral hull, and the box holds more hull than
    # LED, so the invented ones win the vote.
    hsv_raw = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV)
    S = hsv_raw[..., 1].astype(np.float32)
    V = hsv_raw[..., 2].astype(np.float32)
    H = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)[..., 0].astype(np.float32) * 2.0

    # Region order: polygon mask first, centre strip as fallback. The mask is
    # only usable when it encloses the LED box; plenty of labels trace just the
    # hull, and then it masks out the thing being measured.
    regions = []
    if mask is not None and mask.shape[:2] == crop.shape[:2]:
        mm = (mask[by0:by1, bx0:bx1] > 127).astype(np.uint8)
        k = np.ones((3, 3) if min(mm.shape) >= 24 else (2, 2), np.uint8)
        er = cv2.erode(mm, k)
        mmb = (er if int(er.sum()) >= 30 else mm).astype(bool)
        if mmb.shape == S.shape and int(mmb.sum()) >= 30:
            regions.append(mmb)
    centre = np.zeros(S.shape, bool)
    w_ = S.shape[1]
    centre[:, int(w_ * 0.18):max(int(w_ * 0.82), int(w_ * 0.18) + 1)] = True
    regions.append(centre)

    sat_floor = LED_SAT_MIN
    if hull_sat is not None:
        sat_floor = max(LED_SAT_MIN * 0.55, float(hull_sat) + LED_SAT_MARGIN)

    def tiers(region):
        def sel(smin, vmax):
            q = (S > smin) & (V > LED_VAL_MIN)
            return (q & (V < vmax) if vmax else q) & region
        a = sel(sat_floor, LED_VAL_MAX)
        # A bright LED clips its dominant channel, so the body fails the "not
        # blown out" test and tier 0 holds only the anti-aliased rim -- enough
        # pixels to pass a count check, every one a blend with background. If
        # the clipped-but-saturated population is larger, the body IS signal.
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

    v_ratio = sat_med = None
    if wp is not None:
        hull_v = float(np.max(wp))
        reg = regions[0] if int(regions[0].sum()) >= 10 else regions[-1]
        if hull_v > 5 and int(reg.sum()) >= 10:
            v_ratio = float(np.median(V[reg]) / hull_v)
            sat_med = float(np.median(S[reg]))

    base = {"sat": float(S.mean()), "n": int(m.sum()), "v_ratio": v_ratio,
            "sat_med": sat_med, "hull_sat": hull_sat,
            "box": (int(bx0), int(by0), int(bx1), int(by1))}
    if base["n"] < 15:
        return {**base, "hue": None, "conc": 0.0, "quality": 0.0, "tier": 3}

    wgt = S[m] * np.minimum(V[m], 250.0)   # cap value so clipped px don't dominate
    ang = np.deg2rad(H[m])
    x = float((wgt * np.cos(ang)).sum())
    y = float((wgt * np.sin(ang)).sum())
    return {**base, "sat": float(S[m].mean()),
            "hue": float(np.degrees(np.arctan2(y, x)) % 360.0),
            "conc": float(np.hypot(x, y) / max(float(wgt.sum()), 1e-6)),
            "quality": (1.0, 0.8, 0.6)[tier], "tier": tier}


# ---------------------------------------------------------------------------
# classification
# ---------------------------------------------------------------------------
class HsvClassifier:
    """Nearest hue centre, with "near nothing" as its own answer.

    The tolerances are NOT symmetric, and that is measured, not stylistic:

        red    15 deg    circular concentration 1.00, 57/57 correct
        blue   20 deg    circular concentration 0.99, 68/69 correct
        green  50 deg    circular concentration 0.61

    Green is wide because green's readings scatter, not because green is
    ambiguous against the others. Nearest-centre already arbitrates green
    against blue (38 deg apart) and red (175 deg apart) before tolerance is
    consulted, so widening green cannot steal from them -- it only decides how
    far from 164 deg a reading may drift before the answer becomes "off". Unlit
    buoys sit 137 deg from green, so they are never at risk either. Measured:
    green recall 13% -> 58% with no cost anywhere else.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.classes = cfg["classes"]
        self.off_class = cfg.get("off_class", "off")
        self.ramp = float(cfg.get("lit_ramp_deg", 15.0))
        self.conc_ref = float(cfg.get("conc_ref", 0.35))
        # Legacy saturation gate. Off by default and it should stay off -- see
        # the module docstring. Kept only so an old config still loads.
        self.off_feature = cfg.get("off_feature")
        self.off_thresh = cfg.get("off_thresh")

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text()))

    def save(self, path):
        Path(path).write_text(json.dumps(self.cfg, indent=2))

    def _nearest(self, meas):
        """-> (name, distance_deg) or (None, None) if nothing was measurable."""
        if not meas or meas.get("hue") is None:
            return None, None
        best, bd = None, 1e9
        for name, c in self.classes.items():
            d = circ_dist(meas["hue"], c["hue"])
            if d < bd:
                best, bd = name, d
        return best, bd

    def classify(self, meas):
        """-> (label, confidence). Label may be the off class, or None.

        None means "I could not measure", which is not the same as off and must
        not be voted on. Off is a positive finding: a hue too far from every
        LED colour to have come from an LED.
        """
        if not meas:
            return None, 0.0

        if self.off_feature and self.off_thresh is not None:
            val = meas.get(self.off_feature)
            if val is not None and val < float(self.off_thresh):
                return self.off_class, 0.5

        best, bd = self._nearest(meas)
        if best is None:
            return None, 0.0
        tol = float(self.classes[best]["tol"])
        if bd > tol:
            # Not near any LED colour. On the val split this is what an unlit
            # buoy looks like: warm white plastic at 4-27 deg, a good 23 deg
            # clear of the red LED.
            return self.off_class, float(round(min(1.0, (bd - tol) / max(self.ramp, 1e-6)), 4))

        conf = (max(0.0, 1.0 - bd / max(tol, 1e-6))
                * min(1.0, float(meas.get("conc", 0.0)) * 1.5)
                * float(meas.get("quality", 1.0)))
        return best, float(round(conf, 4))

    def lit_score(self, meas):
        """-> 0.0-1.0, the flash tracker's per-frame input.

        This is deliberately NOT a brightness or saturation measure. It is "how
        much does this look like light from an LED", which is the same question
        classify() answers, expressed as a ramp so the tracker gets a gradient
        instead of a cliff:

            distance <= tol             -> 1.0   confidently an LED colour
            tol .. tol + lit_ramp_deg   -> 1..0  linear
            beyond                      -> 0.0   not an LED colour

        scaled by hue concentration, so a reading assembled from pixels that
        disagreed with each other counts for less than a unanimous one.

        Units matter here and have bitten this code before: everything on this
        path is degrees and a 0-1 concentration. Nothing on a 0-255 scale ever
        reaches it, so a threshold cannot silently read as "always lit".
        """
        if not meas:
            return 0.0
        best, bd = self._nearest(meas)
        if best is None:
            return 0.0
        tol = float(self.classes[best]["tol"])
        if bd <= tol:
            base = 1.0
        elif bd >= tol + self.ramp:
            base = 0.0
        else:
            base = 1.0 - (bd - tol) / max(self.ramp, 1e-6)
        conc = min(1.0, float(meas.get("conc", 0.0)) / max(self.conc_ref, 1e-6))
        return float(max(0.0, min(1.0, base * conc)))

    def probabilities(self, meas, labels):
        """-> np.ndarray over `labels`, for oak_detector's vote and health topic.

        A distribution, not a one-hot: the voter and the flash tracker both want
        to know how close the runner-up was. Mass goes to the off class when the
        hue is near nothing, and spreads evenly when nothing was measurable at
        all -- an even vector adds no evidence, which is the honest answer.
        """
        out = np.zeros(len(labels), np.float32)
        idx = {n: i for i, n in enumerate(labels)}
        if not meas or meas.get("hue") is None:
            if self.off_class in idx and meas is not None:
                out[idx[self.off_class]] = 1.0     # measured, found no LED
            else:
                out[:] = 1.0 / max(len(labels), 1)  # could not measure
            return out

        lit = self.lit_score(meas)
        if self.off_class in idx:
            out[idx[self.off_class]] = 1.0 - lit
        if lit <= 0.0:
            if self.off_class not in idx:
                out[:] = 1.0 / max(len(labels), 1)
            return out

        # Softmax-free: inverse circular distance, so the numbers stay readable
        # in the health topic and a human can check them against the config.
        w = {}
        for name, c in self.classes.items():
            if name not in idx:
                continue
            d = circ_dist(meas["hue"], c["hue"])
            w[name] = 1.0 / (1.0 + (d / max(float(c["tol"]), 1e-6)) ** 2)
        tot = sum(w.values()) or 1.0
        for name, v in w.items():
            out[idx[name]] = lit * v / tot
        return out
