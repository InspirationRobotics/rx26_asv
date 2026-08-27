"""shape_led_core — the ROS-free half of the two-stage buoy pipeline.

No rclpy, no depthai, no ultralytics. Association, voting and crop geometry are
all exercisable against invented boxes on a laptop, which is the only reason the
geometry below is trustworthy — none of it is verifiable by watching a live
stream. `oak_detector.py` is the thin ROS wrapper.

Three pieces, and each is here for a different reason:

  led_patch    the crop the colour classifier sees. THE ONE THING THAT MUST NOT
               DRIFT: the training-side exporter cuts crops the same way, and a
               crop that differs by a few percent hands the engine a picture it
               was never trained on. That does not fail loudly — it just
               classifies worse, everywhere, for reasons nothing reports.
  TrackTable   greedy IoU association, so one buoy keeps one identity across
               frames and its colour votes accumulate against one accumulator
               instead of a fresh one every frame.
  LabelVoter   exponentially decayed accumulation of classifier probabilities.

WHY NOT A REAL TRACKER. ultralytics ships ByteTrack and it works, but wiring one
to a TensorRT engine adds a moving part on the boat for no benefit here: buoys
are slow, few, and rarely occlude each other. The association that actually
matters happens downstream in crusader_world_model, in the world frame, against
pose — this one only has to hold an identity long enough to vote on it.

WHY VOTE AT ALL. A single frame's colour call flickers. An LED strip is a
handful of pixels at range, and one frame of glare reads as a different colour.
Decayed accumulation lets a confident observation outweigh several ambiguous
ones while still being able to change its mind, which a plain majority vote
cannot.
"""
import cv2
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