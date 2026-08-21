"""oak_detector_core — the ROS-free half of oak_detector.

No rclpy, no depthai, no ultralytics, and no cv2 at module scope: crop geometry,
association, voting and flash detection are all exercisable against invented
inputs on a laptop, which is the only reason the logic below is trustworthy —
none of it is verifiable by watching a live stream. `oak_detector.py` is the
thin ROS wrapper (Format rule 3, same split as lidar_cluster_core/node).

WHAT LIVES HERE

  led_patch     the crop the colour classifier sees. THE ONE THING THAT MUST NOT
                DRIFT: the training exporter cuts crops the same way, and a crop
                that differs by a few percent hands the engine a picture it was
                never trained on. That does not fail loudly — it just classifies
                worse, everywhere, for reasons nothing reports.
  TrackTable    greedy IoU association, so one buoy keeps one identity across
                frames and its history accumulates against one accumulator.
  LabelVoter    exponentially decayed accumulation of classifier probabilities.
                Stabilises a colour. DESTROYS a flash — see FlashTracker.
  FlashTracker  duty cycle over a track -> flashing / solid / off.

WHY FLASH DETECTION IS NOT A CLASSIFIER OUTPUT

RobotX 2026 Task 1 lights cycle 1 s ON / 1 s OFF. The off-phase of a flashing
buoy is PIXEL-IDENTICAL to a buoy whose light is off, so no single frame can
tell them apart and no per-frame model can learn the difference — half its
training images for "flashing" would be indistinguishable from "off". It is a
property of a track over time, so it is computed from one, here.

Task 1 needs flashing RED / GREEN / BLUE and solid BLUE. Task 2 puts SOLID red
and green indicators on the survey buoys. So flash state matters for every
colour, not just blue, and this reports it for every colour rather than encoding
one task's assumptions.

`import cv2` is function-local so this module imports with numpy alone — which
is what lets tools/bench/bench_flash.py run on a laptop with no OpenCV, and what
would let CI's import check cover it.
"""
import numpy as np


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


def led_patch(image, box, pad=0.10, top_frac=0.40, size=96, min_height_px=10):
    """The classifier's input for one detection, or None if the box is too small.

    Cut in three steps, and every constant has to match the training exporter:

      1. pad the box by `pad` of its size on each side. Annotation slack and the
         detector's own box tightness both vary; the pad is what stops the LED
         strip being clipped off the top by a box that ends a few pixels low.
      2. take the top `top_frac` of the padded crop — the LED band.
      3. resize to `size` x `size`.

    Sampling is bilinear and OUT-OF-IMAGE PIXELS REPLICATE THE EDGE rather than
    going black. A buoy at the top of the frame otherwise gets a black bar in
    exactly the rows the LED lives in, and the classifier reads that as "off".

    min_height_px applies to the padded crop, not the band: below it the band is
    a couple of rows and its colour is whatever the resize invented.
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

    centre = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
    crop = cv2.getRectSubPix(image, (crop_w, crop_h), centre)

    band_h = max(8, int(round(crop_h * top_frac)))
    band = crop[:band_h]
    if band.size == 0:
        return None
    return cv2.resize(band, (size, size), interpolation=cv2.INTER_LINEAR)


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


class LabelVoter:
    """Per-track exponentially decayed accumulation of class probabilities.

    `update` returns (label, confidence). The label is None until the winning
    class clears `min_vote` — an unresolved colour is reported as unresolved
    rather than as a guess, because downstream a wrong colour is worse than no
    colour (a gate is defined by which colour is on which side).

    SET decay=0.0, min_vote=0.0 TO TURN VOTING OFF: the accumulator becomes this
    frame's probabilities and you get the raw per-frame argmax. That is the
    honest way to look at what the engine is actually saying, and it is what
    FlashTracker needs — smoothing averages a flashing light into whichever of
    {colour, off} happens to dominate, which is exactly the signal we want.
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


class FlashState:
    """One track's light state. `state` is the word mission logic branches on."""

    __slots__ = ("state", "duty", "confidence", "samples", "span_s",
                 "colour", "colour_share")

    def __init__(self, state, duty, confidence, samples, span_s,
                 colour, colour_share):
        self.state = state              # flashing | solid | off | unknown
        self.duty = duty                # mean lit fraction over the window
        self.confidence = confidence    # 0..1, see _confidence
        self.samples = samples
        self.span_s = span_s
        self.colour = colour            # dominant colour of the LIT frames
        self.colour_share = colour_share

    def label(self, shape=None):
        """The published name. Mission logic branches on these words.

        `flash_red`, `flash_green`, `flash_blue`, `solid_blue` are Task 1's five
        states (with `off`); `solid_red` / `solid_green` are Task 2's survey-buoy
        indicators. An UNKNOWN flash state degrades to the bare colour rather
        than guessing: calling a solid blue EXIT buoy a flashing blue ENTRY buoy
        sends the boat to the wrong end of the course.
        """
        if self.state == "off":
            parts = ["off"]
        elif self.state in ("flashing", "solid") and self.colour:
            parts = [("flash" if self.state == "flashing" else "solid"),
                     self.colour]
        elif self.colour:
            parts = [self.colour]
        else:
            parts = ["unknown"]
        if shape:
            parts.append(shape)
        return "_".join(parts)

    def __repr__(self):
        d = "None" if self.duty is None else f"{self.duty:.2f}"
        return (f"<{self.state} colour={self.colour} duty={d} "
                f"conf={self.confidence:.2f} n={self.samples}>")


class FlashTracker:
    """Per-track duty cycle -> flashing / solid / off.

    Task 1's lights are 1 s ON / 1 s OFF, so over a window covering whole
    periods a flashing buoy sits near 50% duty, a solid one near 100% and an
    unlit one near 0%. Phase does not matter, which is what makes duty the right
    statistic rather than trying to find edges.

    IT TAKES A SOFT SIGNAL, NOT A HARD LABEL. `lit_score` is 1 - P(off) from the
    classifier, not `argmax != "off"`. Measured, not assumed — see
    tools/bench/bench_flash.py, which separates the two ways a classifier fails:

      HEDGING (it reports ~0.5 instead of committing): soft scores stay at 100%
      correct where hard labels start producing WRONG verdicts, because argmax
      turns a shrug into a full vote. This is the real case for soft input.

      CONFIDENTLY WRONG (P(off) swaps end to end): soft and hard carry the same
      information, and soft simply abstains more often. That is the failure we
      prefer, but it is not an accuracy win — do not claim one.

    Neither rescues a classifier wrong one frame in four. At that error rate
    both mostly report `unknown` for solid-vs-off, which is the correct answer.

    THE DEAD BAND IS DELIBERATE. Between `off_max` and `flash_lo`, and between
    `flash_hi` and `solid_min`, the answer is `unknown`. Mistaking Task 1's
    solid-blue EXIT for its flashing-blue ENTRY sends the boat to the wrong end
    of the course, so refusing to answer is cheaper than answering wrongly.

    WHERE THE BANDS COME FROM. Not taste — tools/bench/bench_flash.py. A
    confident classifier reports ~0.9 for a lit frame, not 1.0, so a SOLID light
    measures ~0.90 duty and an OFF one ~0.10 before any noise at all. Set
    solid_min at 0.85 and a solid buoy becomes unclassifiable the moment the
    classifier is wrong one frame in ten. These bands survive 10% frame error
    and abstain at 25%, which is the right place to give up: a model wrong every
    fourth frame should not be deciding ENTRY versus EXIT.

    SAMPLING. A 0.5 Hz square wave needs more than 1 Hz of sampling. Measured
    against this class, 2 fps is the floor and 3+ fps is comfortable; below that
    it reports `unknown` rather than guessing. Check the node's health line for
    the rate you actually sustain before trusting a verdict.
    """

    def __init__(self, window_s=6.0, min_span_s=4.0, min_samples=8,
                 flash_lo=0.38, flash_hi=0.62, solid_min=0.70, off_max=0.30,
                 colour_min_share=0.6):
        self.window_s = window_s
        self.min_span_s = min_span_s
        self.min_samples = min_samples
        self.flash_lo, self.flash_hi = flash_lo, flash_hi
        self.solid_min, self.off_max = solid_min, off_max
        self.colour_min_share = colour_min_share
        self.hist = {}                  # track id -> [(t, lit_score, colour)]

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
                              None, 0.0)

        duty = float(np.mean([s for _, s, _ in history]))
        lit_colours = [c for _, s, c in history if c and s > 0.5]
        colour, share = None, 0.0
        if lit_colours:
            names, counts = np.unique(lit_colours, return_counts=True)
            best = int(counts.argmax())
            share = float(counts[best] / len(lit_colours))
            if share >= self.colour_min_share:
                colour = str(names[best])

        state, confidence = self._classify(duty)
        # Coverage: a window only half-filled has seen less than one full
        # period, so its duty is a phase artefact rather than a measurement.
        coverage = min(1.0, span / self.window_s)
        return FlashState(state, duty, confidence * coverage,
                          len(history), span, colour, share)

    def _classify(self, duty):
        """(state, confidence-before-coverage). Confidence is distance from the
        nearest decision boundary, normalised so it reaches 1 at the ideal duty
        (0.0 off, 0.5 flashing, 1.0 solid) and 0 at the edge of the band."""
        if duty <= self.off_max:
            return "off", 1.0 - duty / max(self.off_max, 1e-6)
        if duty >= self.solid_min:
            return "solid", (duty - self.solid_min) / max(1.0 - self.solid_min, 1e-6)
        if self.flash_lo <= duty <= self.flash_hi:
            half = max(0.5 - self.flash_lo, self.flash_hi - 0.5, 1e-6)
            return "flashing", 1.0 - abs(duty - 0.5) / half
        return "unknown", 0.0

    def prune(self, live_ids):
        for track_id in list(self.hist):
            if track_id not in live_ids:
                del self.hist[track_id]