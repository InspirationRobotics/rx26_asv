"""oak_detector_core — the ROS-free half of buoy_model (camera_node's buoy model).

No rclpy, no depthai, no ultralytics, and no cv2 at module scope: crop geometry,
association, voting and flash detection are exercisable against invented inputs on
a laptop, the only way to trust logic none of which is verifiable from a live
stream. The module keeps its name because crusader_sim imports it.

One section per pipeline stage, each headed by why it works the way it does:

  crops         what each stage is allowed to look at — and they differ
  marking_shape diamond vs circle, from the outline. No model.
  TrackTable    one identity per buoy, across frames
  LabelVoter    colour, smoothed. DESTROYS a flash; see the last section
  FlashTracker  flashing / solid / off, from how the light moves over time

`import cv2` is function-local so this module imports with numpy alone
(tools/bench/bench_flash.py runs on a laptop with no OpenCV).
"""
import numpy as np

# buoy_model's parameters and defaults: here, with no ROS, so the sim reads what the boat runs.
PARAMS = {
    "det_engine": "/root/robotx_ws/models/buoy_detector.engine",
    "det_labels": ["circle", "diamond"],    # IN ENGINE ORDER: a mismatch mislabels every box
    "det_conf_min": 0.6,
    "det_imgsz_height": 640,
    "det_imgsz_width": 640,
    "shape_source": "geometry",             # geometry | detector; geometry falls back to
                                            # the detector's class when it abstains
    "shape_min_radius": 18.0,               # px: the outline test is unreliable under ~6
    "shape_min_share": 0.6,
    "cls_engine": "/root/robotx_ws/models/led_classifier.engine",
    "cls_labels": ["blue", "green", "off", "red"],   # IN ENGINE ORDER (training folders)
    "cls_imgsz": 64,                        # these three MUST match the training exporter
    "crop_pad": 0.1,
    "crop_top_frac": 0.25,
    "iou_match": 0.25,
    "max_missed": 10,
    "vote_decay": 0.85,
    "vote_min": 0.55,
    "flash_enable": True,
    "flash_window_s": 6.0,                  # whole 1 s ON / 1 s OFF periods
    "flash_min_span_s": 4.0,
    "flash_min_samples": 8,
    "solid_min": 0.7,                       # the gaps between these four are dead bands
    "off_max": 0.3,
    "colour_min_share": 0.6,
    "flash_period_s": 2.0,                  # RobotX Task 1: 1 s ON / 1 s OFF
    "flash_corr_min": 0.45,
    "flash_debug_path": "",                 # jsonl of every observation, for replay
    "range_min_m": 0.3,
    "range_max_m": 25.0,
    "min_depth_samples": 5,
}


def camera_params(doc):
    """The buoy parameters camera_node runs with, from a parsed crusader_params.yaml."""
    return dict(PARAMS, **doc.get("camera_node", {}).get("ros__parameters", {}))

# LED box geometry. MUST match buoylib's LED_APEX / LED_WIDTH, which is what
# tools/buoy_color/4_export.py cut the training crops with.
LED_APEX = 0.15     # columns are taken from the top 15% of the buoy only
LED_WIDTH = 0.62    # ...then narrowed to this centred fraction of that width.
                    # MUST EQUAL buoylib.LED_WIDTH, i.e. whatever 4_export.py
                    # cut the training crops with. Change both or neither.

# Flash-tracker shaping. NOT parameters: neither decides anything, they only
# shape a confidence number and pick which word explains an abstention. Every
# knob in crusader_params.yaml is one somebody has to understand before a run,
# so a number nobody would tune belongs here instead.
FLASH_BALANCE_SPAN = 0.12   # how far duty may sit from 0.5 before a `flashing`
                            # verdict's confidence starts falling off
MOD_STEADY_MAX = 0.20       # p90-p10 swing below which an abstention is
                            # reported as `steady-but-unsure` rather than
                            # `aperiodic` — diagnostic wording, nothing else


# ---------------------------------------------------------------- what each stage may look at
# Two crops, two jobs, NOT interchangeable:
#   led_patch   the colour classifier's input: a tight box on the LED, resized to the engine's
#               input size. Must match the training exporter exactly: a full-width top band
#               once drifted from 4_export.py's led_box, and a classifier measuring 98.6% on
#               held-out crops silently underperformed on the boat.
#   buoy_patch  the shape stage's input: the whole buoy at native resolution, because a
#               resize destroys the outline it measures.


def iou(a, b):
    """Intersection over union of two (x1, y1, x2, y2) boxes."""
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter <= 0.0:
        return 0.0
    union = ((a[2] - a[0]) * (a[3] - a[1])
             + (b[2] - b[0]) * (b[3] - b[1]) - inter)
    return inter / max(union, 1e-6)


def led_patch(image, box, pad=0.10, top_frac=0.25, size=64, min_height_px=10,
              return_box=False):
    """The classifier's input for one detection, or None if the box is too small.

    return_box=True gives (patch, (x0, y0, x1, y1)) with the box in FRAME pixels, what the
    annotated stream should draw, so the overlay and the engine's picture come from one line.

    THIS MUST CUT THE SAME PICTURE THE TRAINING EXPORTER CUT (buoylib.led_box); this reproduces
    it from a detector box, with no polygon available. A buoy is a truncated pyramid (wide base,
    narrow flat top, LED on the top face), so a full-width band frames mostly sky and deck (27%
    buoy vs 69% for the box below, measured on 29 real instances).

    Steps:
      1. pad the box by `pad` of its size on each side.
      2. ROWS START AT THE TOP OF THE BRIGHT REGION, not row 0: the pad puts ~10% background
         above the buoy, which would pull sky into the LED band.
      3. COLUMNS COME FROM THE APEX ONLY (top 15% of the buoy), then narrow to the centred
         62%; the widest row would otherwise set the width, and on a trapezoid that is the
         bottom.
      4. resize to `size` x `size`.

    `top_frac` means what the exporter's --crop-frac means: fraction of the BUOY kept below the
    LED (0.25), not of the crop taken off the top. Change the param with the code. The exporter
    takes rows/columns from a polygon mask and a detector box gives only pixels, so brightness
    stands in for the outline, leaving a residual gap; closing it needs a maskless mode in
    4_export.py and a retrain on the boat's own crop.

    Sampling is bilinear and OUT-OF-IMAGE PIXELS REPLICATE THE EDGE rather than going black: a
    black bar in the LED rows reads as "off".
    """
    import cv2                                   # not needed to import this file

    x1, y1, x2, y2 = (float(v) for v in box)
    w, h = x2 - x1, y2 - y1
    if w <= 0.0 or h <= 0.0:
        return (None, None) if return_box else None

    scale = 1.0 + 2.0 * pad
    crop_w = max(8, int(round(w * scale)))
    crop_h = max(8, int(round(h * scale)))
    if crop_h < min_height_px:
        return (None, None) if return_box else None

    centre = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
    crop = cv2.getRectSubPix(image, (crop_w, crop_h), centre)

    H, W = crop.shape[:2]

    # -- rows: first row that actually contains buoy --
    look = crop[:max(8, int(round(H * 0.5)))]
    grey = cv2.cvtColor(look, cv2.COLOR_BGR2GRAY)
    bright = grey > np.percentile(grey, 65)
    rows = np.flatnonzero(bright.sum(1) > 0.15 * W)
    r0 = int(rows[0]) if rows.size else 0

    # -- columns: from the APEX only, then narrowed to the LED's own width --
    apex_h = max(6, int(round(H * LED_APEX)))
    apex = cv2.cvtColor(crop[r0:min(H, r0 + apex_h)], cv2.COLOR_BGR2GRAY)
    seen = max(apex.shape[0], 1)
    cols = np.flatnonzero((apex > np.percentile(apex, 65)).sum(0) > 0.35 * seen)
    cx0, cx1 = (int(cols[0]), int(cols[-1])) if cols.size >= 4 else (0, W - 1)

    mid = (cx0 + cx1) / 2.0
    half = (cx1 - cx0) * LED_WIDTH / 2.0
    bx0, bx1 = int(round(mid - half)), int(round(mid + half))
    by0, by1 = r0, min(H, r0 + max(8, int(round(H * top_frac))))

    # 10% margin, matching the exporter's led_box(pad=0.10)
    mh = int((bx1 - bx0) * 0.10)
    mv = int((by1 - by0) * 0.10)
    bx0, by0 = max(0, bx0 - mh), max(0, by0 - mv)
    bx1, by1 = min(W, bx1 + mh + 1), min(H, by1 + mv)

    band = crop[by0:by1, bx0:bx1]
    if band.size == 0:
        return (None, None) if return_box else None
    patch = cv2.resize(band, (size, size), interpolation=cv2.INTER_LINEAR)
    if not return_box:
        return patch
    # Crop coords -> frame coords. getRectSubPix centres the crop on the box,
    # so the crop's origin is the box centre minus half the crop.
    ox = (x1 + x2) / 2.0 - crop_w / 2.0
    oy = (y1 + y2) / 2.0 - crop_h / 2.0
    return patch, (int(round(ox + bx0)), int(round(oy + by0)),
                   int(round(ox + bx1)), int(round(oy + by1)))


def buoy_patch(image, box, pad=0.10, min_height_px=24):
    """The WHOLE padded box at native resolution — what the shape stage needs.

    Deliberately not led_patch: the shape stage measures the outline of a marking a few dozen
    pixels across, and a resize is what destroys it.
    """
    import cv2                                   # not needed to import this file

    x1, y1, x2, y2 = (float(v) for v in box)
    w, h = x2 - x1, y2 - y1
    if w <= 0.0 or h <= 0.0:
        return None
    scale = 1.0 + 2.0 * pad
    crop_w = max(8, int(round(w * scale)))
    crop_h = max(8, int(round(h * scale)))
    if crop_h < min_height_px:
        return None
    return cv2.getRectSubPix(image, (crop_w, crop_h),
                             ((x1 + x2) / 2.0, (y1 + y2) / 2.0))


# ---------------------------------------------------------------- STAGE 2: shape, measured not predicted
# No model: the 4th harmonic of r(theta) after whitening the outline by its own second moments,
# plus the bounding-box fill ratio; the two must agree or it abstains.
# The whitening is what survives a yawed buoy: a foreshortened circle is an ellipse with a real
# 4th harmonic (raw h4 = 0.068 at 66 deg yaw, squarely diamond; whitened 0.003).
# The marking is found as a HOLE in the hull, not by thresholding darkness, so a shadow on the
# deck or the water is never a candidate.

# Decision constants. MUST match buoylib's, or the bench stops predicting the
# boat. Ranges in the comments are measured, not assumed.
SHAPE_H4_MID = 0.015     # circle 0.000-0.009, diamond 0.020-0.069 (r >= 6px)
SHAPE_H4_SPAN = 0.015
SHAPE_FILL_MID = 0.668   # circle 0.678-0.770, diamond 0.514-0.657
SHAPE_FILL_SPAN = 0.060
SHAPE_MIN_RADIUS = 6.0
SHAPE_MAX_ASPECT = 4.0   # past this the marking is nearly edge-on: too few
                         # pixels across the short axis for whitening to recover
SHAPE_NA = 64            # angular samples of r(theta) before the FFT


def _otsu(g, mask=None):
    import cv2
    if mask is None:
        return cv2.threshold(g, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[0]
    v = g[mask]
    if v.size < 30:
        return None
    return cv2.threshold(v.reshape(-1, 1), 0, 255,
                         cv2.THRESH_BINARY | cv2.THRESH_OTSU)[0]


def _largest_hole(bw):
    """Biggest hole inside the biggest bright blob — i.e. the marking."""
    import cv2
    cnts, hier = cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if not cnts or hier is None:
        return None
    hier = hier[0]
    outer = [i for i in range(len(cnts)) if hier[i][3] < 0]
    if not outer:
        return None
    p = max(outer, key=lambda i: cv2.contourArea(cnts[i]))
    hull_area = cv2.contourArea(cnts[p])
    best, ba, ch = None, 0.0, hier[p][2]
    while ch >= 0:
        a = cv2.contourArea(cnts[ch])
        if a > ba:
            best, ba = ch, a
        ch = hier[ch][0]
    if best is None or ba < 30 or ba > 0.75 * max(hull_area, 1):
        return None
    return cnts[best].reshape(-1, 2).astype(np.float32)


def _redraw_hull(c):
    """Refill the convex hull and re-trace, so a glint or a small bite does not
    put a notch in the radial signature. Both markings are convex."""
    import cv2
    h = cv2.convexHull(c.astype(np.int32))
    x, y, w, hh = cv2.boundingRect(h)
    m = np.zeros((hh + 4, w + 4), np.uint8)
    cv2.fillPoly(m, [h - [x - 2, y - 2]], 255)
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return cs[0].reshape(-1, 2).astype(np.float32) if cs else c


def marking_shape(crop, names=("circle", "diamond"),
                  min_radius=SHAPE_MIN_RADIUS):
    """-> (name or None, confidence, info). None means 'cannot tell'.

    `info` carries r, h4, fill, sol, ar, pass and a `why` string. Publish or log
    it: every abstention has a reason, and "the marking is 4 px across" and "h4
    and fill disagree" are different problems.
    """
    import cv2
    K3 = np.ones((3, 3), np.uint8)
    circle_name, diamond_name = names
    info = {"r": 0.0, "h4": 0.0, "fill": 0.0, "sol": 0.0, "ar": 1.0,
            "pass": 0, "why": ""}
    if crop is None or crop.size == 0 or min(crop.shape[:2]) < 16:
        info["why"] = "crop too small"
        return None, 0.0, info

    g = cv2.GaussianBlur(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), (3, 3), 0)
    t1 = _otsu(g)
    c = _largest_hole(cv2.morphologyEx((g > t1).astype(np.uint8) * 255,
                                       cv2.MORPH_CLOSE, K3))
    info["pass"] = 1
    if c is None:
        # A hard shadow drags half the hull under the global threshold, so the
        # marking stops being enclosed. Re-threshold only the sub-threshold
        # pixels; that splits shadowed hull from marking.
        t2 = _otsu(g, g <= t1)
        if t2 is not None and t2 < t1:
            c = _largest_hole(cv2.morphologyEx((g > t2).astype(np.uint8) * 255,
                                               cv2.MORPH_CLOSE, K3))
            info["pass"] = 2
    if c is None:
        info["why"] = "no dark region enclosed by hull"
        return None, 0.0, info

    area = cv2.contourArea(c)
    sol = area / max(cv2.contourArea(cv2.convexHull(c)), 1e-6)
    info["sol"] = float(sol)
    if sol < 0.80:
        info["why"] = f"outline not convex (sol {sol:.2f})"
        return None, 0.0, info
    if sol < 0.97:
        c = _redraw_hull(c)
        area = cv2.contourArea(c)

    # Centre on the AREA centroid, not the mean of contour points: uneven point
    # spacing offsets the point-mean and leaks energy into every harmonic.
    M = cv2.moments(c.astype(np.float32))
    if M["m00"] <= 0:
        info["why"] = "degenerate outline"
        return None, 0.0, info
    ctr = np.array([M["m10"] / M["m00"], M["m01"] / M["m00"]], np.float32)
    d = (c - ctr).astype(np.float64)

    # Whiten by the region's own second moments: rotate onto the principal axes
    # and rescale each to unit variance. Undoes foreshortening, and does nothing
    # to a shape that is already isotropic, which a diamond is.
    cov = np.array([[M["mu20"], M["mu11"]], [M["mu11"], M["mu02"]]]) / M["m00"]
    evals, evecs = np.linalg.eigh(cov)
    evals = np.clip(evals, 1e-9, None)
    info["ar"] = float(np.sqrt(evals.max() / evals.min()))
    if info["ar"] > SHAPE_MAX_ASPECT:
        info["why"] = f"marking is edge-on (aspect {info['ar']:.1f})"
        return None, 0.0, info
    dw = (d @ evecs) / np.sqrt(evals)

    r = np.hypot(dw[:, 0], dw[:, 1])
    th = np.arctan2(dw[:, 1], dw[:, 0]) % (2 * np.pi)
    o = np.argsort(th)
    th, r = th[o], r[o]
    grid = np.linspace(0, 2 * np.pi, SHAPE_NA, endpoint=False)
    rr = np.interp(grid, np.concatenate([th, th[:1] + 2 * np.pi]),
                   np.concatenate([r, r[:1]]))
    F = np.abs(np.fft.rfft(rr)) / SHAPE_NA
    # r0 is in whitened units; report the real pixel radius from the area.
    info["r"] = r0 = float(np.sqrt(max(area, 0.0) / np.pi))
    if r0 < min_radius:
        info["why"] = f"marking only {r0:.1f}px radius"
        return None, 0.0, info

    h = F / max(float(F[0]), 1e-6)
    h4 = float(h[4])
    _, _, bw_, bh_ = cv2.boundingRect(c.astype(np.int32))
    fill = float(area / max(bw_ * bh_, 1))     # raw outline, on purpose
    info["h4"], info["fill"] = h4, fill

    # h2 is deliberately not policed: a buoy seen off-axis foreshortens into an
    # ellipse, which is exactly h2. Odd and high harmonics mean a broken blob.
    junk = float(max(h[3], h[5], h[6], h[7]))
    if h4 > SHAPE_H4_MID and junk > 0.8 * h4:
        info["why"] = f"outline is noise (junk {junk:.3f} vs h4 {h4:.3f})"
        return None, 0.0, info

    v_h4 = np.clip((h4 - SHAPE_H4_MID) / SHAPE_H4_SPAN, -1.0, 1.0)
    v_fill = np.clip((SHAPE_FILL_MID - fill) / SHAPE_FILL_SPAN, -1.0, 1.0)
    if (v_h4 > 0) != (v_fill > 0):
        info["why"] = f"h4 and fill disagree ({h4:.3f} / {fill:.3f})"
        return None, 0.0, info

    s = float((abs(v_h4) + abs(v_fill)) / 2)
    return (diamond_name if v_h4 > 0 else circle_name), round(min(1.0, s), 3), info


class ShapeVoter:
    """Per-track majority vote over `marking_shape` calls.

    The geometric test abstains often (range, edge-on, shadow) and can flip at the boundary, so
    a track votes over its many sightings. Abstentions are NOT votes: counting them as evidence
    is how "cannot see it yet" becomes a confident wrong answer at range.
    """

    def __init__(self, min_share=0.6, min_votes=3):
        self.min_share = min_share
        self.min_votes = min_votes
        self.counts = {}          # track id -> {name: weight}

    def update(self, track_id, name, confidence):
        c = self.counts.setdefault(track_id, {})
        if name is not None:
            # Weighted by confidence: a marginal call should not outvote a clear
            # one just by arriving more often.
            c[name] = c.get(name, 0.0) + max(0.05, float(confidence))
        total = sum(c.values())
        if not c or total < self.min_votes * 0.05:
            return None, 0.0
        best = max(c, key=c.get)
        share = c[best] / max(total, 1e-6)
        if len(c) > 1 and share < self.min_share:
            return None, share
        return best, share

    def prune(self, live_ids):
        for k in list(self.counts):
            if k not in live_ids:
                del self.counts[k]


# ---------------------------------------------------------------- STAGE 3: identity, one buoy across frames
# Greedy IoU association, not a real tracker: buoys are slow and rarely occlude, and the
# association that matters happens downstream in crusader_world_model, in the world frame. This
# only has to hold an identity long enough to vote on it.


class TrackTable:
    """Greedy IoU association across frames. `update(boxes) -> one id per box`.

    Ids are never reused, matching TrackedTarget's rule: a stored reference to
    "track 7" can never be silently re-pointed at a different buoy.
    """

    def __init__(self, iou_match=0.25, max_missed=10):
        self.iou_match = iou_match
        self.max_missed = max_missed
        self.tracks = {}          # id -> [box, frames_missed]
        self.next_id = 1

    def update(self, boxes):
        ids = [None] * len(boxes)
        pairs = sorted(
            ((iou(track[0], box), tid, i)
             for tid, track in self.tracks.items()
             for i, box in enumerate(boxes)
             if iou(track[0], box) >= self.iou_match),
            reverse=True)

        claimed_tracks, claimed_boxes = set(), set()
        for _, tid, i in pairs:
            if tid in claimed_tracks or i in claimed_boxes:
                continue
            claimed_tracks.add(tid)
            claimed_boxes.add(i)
            ids[i] = tid
            self.tracks[tid] = [boxes[i], 0]

        for i, box in enumerate(boxes):
            if ids[i] is None:
                ids[i] = self.next_id
                self.tracks[self.next_id] = [box, 0]
                self.next_id += 1

        for tid in list(self.tracks):
            if tid in claimed_tracks or tid in ids:
                continue
            self.tracks[tid][1] += 1
            if self.tracks[tid][1] > self.max_missed:
                del self.tracks[tid]
        return ids

    @property
    def live_ids(self):
        return set(self.tracks)


# ---------------------------------------------------------------- STAGE 4: colour over time
# A single frame's colour flickers (an LED strip is a handful of pixels at range). Decayed
# accumulation lets a confident observation outweigh several ambiguous ones while still being
# able to change its mind, which a plain majority vote cannot. Memory is 1/(1-decay) frames;
# decay=0.0 and min_vote=0.0 turn it off (raw per-frame argmax).
# THIS DESTROYS A FLASH, which is why FlashTracker bypasses it entirely.


class LabelVoter:
    """Per-track exponentially decayed accumulation of class probabilities.

    `update` returns (label, confidence). The label is None until the winning class clears
    `min_vote`: an unresolved colour is reported as unresolved, not guessed, because downstream
    a wrong colour is worse than none (a gate is defined by which colour is on which side).

    SET decay=0.0, min_vote=0.0 TO TURN VOTING OFF: the accumulator becomes this frame's
    probabilities, the raw per-frame argmax. FlashTracker needs that, since smoothing averages
    a flashing light into whichever of {colour, off} dominates.
    """

    def __init__(self, classes, decay=0.85, min_vote=0.55):
        self.classes = list(classes)
        self.decay = decay
        self.min_vote = min_vote
        self.accumulated = {}

    def update(self, track_id, probabilities):
        previous = self.accumulated.get(
            track_id, np.zeros(len(self.classes), np.float32))
        total = previous * self.decay + np.asarray(probabilities, np.float32)
        self.accumulated[track_id] = total

        weight = float(total.sum())
        if weight <= 0.0:
            return None, 0.0
        normalised = total / weight
        best = int(normalised.argmax())
        confidence = float(normalised[best])
        if confidence < self.min_vote:
            return None, confidence
        return self.classes[best], confidence

    def prune(self, live_ids):
        """Drop accumulators for tracks that no longer exist. Without this the
        dict grows for the whole run and a recycled id would inherit votes."""
        for track_id in list(self.accumulated):
            if track_id not in live_ids:
                del self.accumulated[track_id]


# ---------------------------------------------------------------- STAGE 5: light state over time
# A flashing buoy's off-phase is pixel-identical to an unlit buoy, so no per-frame model can
# tell them apart; it is a property of a track.
# NOT decided by duty cycle: a 1 s ON / 1 s OFF light averages 0.5 lit, and so does a classifier
# hedging at 0.5 on every frame (how it actually fails: 27% recall on `off`, measured).
# What separates them is whether the signal MOVES LIKE A LIGHT: anti-correlation at half the
# flash period (square wave ~1.0, noise ~0). That test is SCALE-FREE, so a barely-visible green
# flash (0.52 to 0.47; green's whole margin over an unlit hull is +6 saturation points) passes
# as easily as a bright red one.
# Level is asked second and only decides solid vs off. The gap between off_max and solid_min is
# a deliberate dead band: mistaking Task 1's solid-blue EXIT for its flashing-blue ENTRY sends
# the boat to the wrong end of the course, so refusing to answer is much cheaper than answering
# wrongly. SAMPLING FLOOR: 2 fps, measured; below that it reports `unknown`.


class FlashState:
    """One track's light state. `state` is the word mission logic branches on."""

    __slots__ = ("state", "duty", "modulation", "basis", "confidence",
                 "samples", "span_s", "colour", "colour_share")

    def __init__(self, state, duty, confidence, samples, span_s,
                 colour, colour_share, modulation=None, basis="none"):
        self.state = state              # flashing | solid | off | unknown
        self.duty = duty                # mean lit score over the window
        self.modulation = modulation    # how much it SWINGS; see FlashTracker
        self.basis = basis              # which test decided: modulation|level|
                                        # dead-band|insufficient
        self.confidence = confidence    # 0..1
        self.samples = samples
        self.span_s = span_s
        self.colour = colour            # dominant colour of the LIT frames
        self.colour_share = colour_share

    def label(self, shape=None):
        """The published name. Mission logic branches on these words.

            flashing   flash_red_diamond
            solid      red_diamond
            off        off_diamond
            no colour  diamond

        SOLID IS THE UNMARKED CASE, on purpose: `flash_` carries the meaning, so Task 1's ENTRY
        buoy stays distinguishable from its EXIT buoy (`flash_blue_circle` against `blue_circle`).

        AN UNDECIDED STATE PUBLISHES AS SOLID, a deliberate trade: every track spends its first
        flash_min_span_s as `unknown` (window not full), so a flashing buoy reads as steady for
        that time. Downstream that washes out: target_tracker labels by MAJORITY VOTE over every
        sighting, and four seconds of wrong loses against minutes of right.

        So do not branch on a track younger than flash_min_span_s. For certainty now,
        crsd/oak_detector_health carries the state, basis and sample count per track, where
        `unknown` is unambiguous.
        """
        if self.state == "off":
            parts = ["off"]
        elif not self.colour:
            parts = []                      # nothing to say but the shape
        elif self.state == "flashing":
            parts = ["flash", self.colour]
        else:
            parts = [self.colour]           # solid, or not yet decided
        if shape:
            parts.append(shape)
        return "_".join(parts) if parts else "unknown"

    def __repr__(self):
        d = "None" if self.duty is None else f"{self.duty:.2f}"
        m = "None" if self.modulation is None else f"{self.modulation:.2f}"
        return (f"<{self.state} colour={self.colour} duty={d} mod={m} "
                f"by={self.basis} conf={self.confidence:.2f} n={self.samples}>")


class FlashTracker:
    """Per-track light state. `update` returns a FlashState every frame.

    Takes a SOFT lit score (1 - P(off)), not an argmax: when the classifier
    hedges, argmax turns a shrug into a full vote.
    """

    def __init__(self, window_s=6.0, min_span_s=4.0, min_samples=8,
                 solid_min=0.70, off_max=0.30, colour_min_share=0.6,
                 flash_period_s=2.0, flash_corr_min=0.45):
        self.window_s = window_s
        self.min_span_s = min_span_s
        self.min_samples = min_samples
        self.solid_min, self.off_max = solid_min, off_max
        self.colour_min_share = colour_min_share
        self.flash_period_s = flash_period_s
        self.flash_corr_min = flash_corr_min
        self.hist = {}                  # track id -> [(t, lit_score, colour)]

    @staticmethod
    def modulation(scores):
        """How much the light SWINGS over the window, 0..1.

        p90 - p10 of the lit score, not max - min: one misclassified frame in a 180-frame window
        would take max-min to 1.0 and call a dead buoy flashing. The percentile pair ignores the
        tails and still sees a real 1 s ON / 1 s OFF cycle (~45% of the window at each end).
        """
        if len(scores) < 4:
            return 0.0
        a = np.asarray(scores, np.float32)
        return float(np.percentile(a, 90) - np.percentile(a, 10))

    def periodicity(self, history):
        """-> anti-correlation at half the flash period, 0..1. Higher = more like a square wave
        at the expected rate.

        THIS IS THE TEST THAT AMPLITUDE CANNOT DO: a weak flash (lit score 0.60 to 0.35, a
        barely-separated green LED) and a classifier hedging with noise both swing about 0.25,
        and sweeping the modulation threshold either accepted both or rejected both. What
        differs is SHAPE: a 1 s ON / 1 s OFF light is always the opposite of what it was one
        second earlier (correlation at lag = period/2 near -1), noise has no memory (near 0),
        and correlation is scale-free.

        Samples arrive at irregular times, so the history is resampled onto a uniform grid
        first. Returns 0.0 rather than guessing when the window is too short or sparse for the
        lag asked.
        """
        if len(history) < 8:
            return 0.0
        t = np.asarray([h[0] for h in history], np.float64)
        v = np.asarray([h[1] for h in history], np.float64)
        span = t[-1] - t[0]
        lag = self.flash_period_s / 2.0
        if span < self.flash_period_s * 1.5:      # under 1.5 periods: no verdict
            return 0.0

        # Grid at the observed mean rate, capped so a long window stays cheap.
        rate = min(len(history) / max(span, 1e-6), 30.0)
        n = int(span * rate)
        k = int(round(lag * rate))
        if k < 1 or n < 2 * k + 4:
            return 0.0
        grid = np.linspace(t[0], t[-1], n)
        x = np.interp(grid, t, v)

        a, b = x[:-k], x[k:]
        a = a - a.mean()
        b = b - b.mean()
        denom = float(np.sqrt((a * a).sum() * (b * b).sum()))
        if denom < 1e-9:                          # perfectly flat: no swing
            return 0.0
        corr = float((a * b).sum() / denom)
        return max(0.0, -corr)                    # only ANTI-correlation counts

    def update(self, track_id, now, lit_score, colour=None):
        """Add one observation and return the track's FlashState.

        lit_score: 0..1, how lit this frame was. Pass `1 - P(off)`.
        colour:    the argmax colour name this frame, or None. Only counted
                   while the light is actually lit — the colour reported during
                   an off-phase is meaningless and would dilute the vote.
        """
        history = self.hist.setdefault(track_id, [])
        history.append((float(now), float(np.clip(lit_score, 0.0, 1.0)), colour))

        cutoff = now - self.window_s
        while history and history[0][0] < cutoff:
            history.pop(0)

        span = history[-1][0] - history[0][0]
        if len(history) < self.min_samples or span < self.min_span_s:
            return FlashState("unknown", None, 0.0, len(history), span,
                              None, 0.0, None, "insufficient")

        scores = [s for _, s, _ in history]
        duty = float(np.mean(scores))
        mod = self.modulation(scores)

        lit_colours = [c for _, s, c in history if c and s > 0.5]
        colour, share = None, 0.0
        if lit_colours:
            names, counts = np.unique(lit_colours, return_counts=True)
            best = int(counts.argmax())
            share = float(counts[best] / len(lit_colours))
            if share >= self.colour_min_share:
                colour = str(names[best])

        period = self.periodicity(history)
        state, confidence, basis = self._classify(duty, mod, period)
        # Coverage: a window only half-filled has seen less than one full
        # period, so its duty is a phase artefact rather than a measurement.
        coverage = min(1.0, span / self.window_s)
        return FlashState(state, duty, confidence * coverage,
                          len(history), span, colour, share, mod, basis)

    def _classify(self, duty, mod, period):
        """(state, confidence-before-coverage, which test decided).

        THREE QUESTIONS, IN THIS ORDER, AND THE ORDER IS THE DESIGN.

        1. Does it move like a LIGHT?  ->  anti-correlation at half the flash period. Scale-free,
           so a barely-visible green flash scores like a bright red one, while noise of any
           amplitude scores near zero.
        2. Not periodic. Is the LEVEL decisive?  ->  the mean decides solid or off.
        3. Mid-range and not periodic  ->  `unknown`: a classifier with no idea, which duty
           alone would have called FLASHING.

        THE DEAD BANDS ARE DELIBERATE: mistaking Task 1's solid-blue EXIT for its flashing-blue
        ENTRY sends the boat to the wrong end of the course, so refusing to answer is much
        cheaper than answering wrongly.
        """
        # 1. Does it move like a LIGHT? Asked FIRST: the test is SCALE-FREE and a level test is
        #    not. A barely-separated green LED (0.55 to 0.42) swings less than some hedging
        #    noise, so an amplitude gate placed first would file it "not moving" and never run
        #    the periodicity test on the case that needs it most.
        if period >= self.flash_corr_min:
            near = 1.0 - min(1.0, abs(duty - 0.5) / FLASH_BALANCE_SPAN)
            return "flashing", max(0.15, 0.5 * period + 0.5 * near), "periodic"

        # 2. Not periodic. Is the LEVEL decisive on its own? Modulation deliberately does NOT
        #    gate this: a solid light seen through a noisy classifier swings enough to fail a
        #    quietness test, and a buoy lit 90% of the time would come back `unknown`. Noise
        #    moves the mean very little; only in the MIDDLE are noise and signal confusable,
        #    and the dead band covers the middle.
        if duty >= self.solid_min:
            return "solid", (duty - self.solid_min) / max(
                1.0 - self.solid_min, 1e-6), "level"
        if duty <= self.off_max:
            return "off", 1.0 - duty / max(self.off_max, 1e-6), "level"

        # 3. Middle of the range and not periodic: a classifier with no idea (duty ALONE would
        #    have called it flashing). `mod` is reported so the debug log can tell a hedging
        #    model (flat) from a light the tracker failed to lock onto (not flat).
        return "unknown", 0.0, ("steady-but-unsure" if mod <= MOD_STEADY_MAX
                                else "aperiodic")

    def prune(self, live_ids):
        for track_id in list(self.hist):
            if track_id not in live_ids:
                del self.hist[track_id]