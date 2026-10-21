"""buoy_model: camera_node's `buoy` model. Each frame -> a Detection3DArray of buoys, their
marking shape, their LED colour and light state, positioned from the stereo depth.

  stage 1  det_engine     where the buoys are
  stage 2  marking_shape  diamond or circle, from the outline (CPU, no model)
  stage 3  TrackTable     one identity per buoy, across frames
  stage 4  cls_engine     what colour each LED is
  stage 5  FlashTracker   flashing / solid / off, from the track over time

Stage 2 has no model, so stage 1 can be a single-class buoy detector: at 15 m a marking
spans a couple of feature-map cells, while the full-resolution crop is 60-200 px. Stage 3
runs before stage 4 so a crop that fails to cut does not cost a track its history.

THE LABEL IS THE CONTRACT (mission logic branches on these words):
    flashing   flash_red_diamond
    solid      red_diamond
    off        off_diamond
    no colour  diamond
Flashing is a property of a track over time, not a classifier class: the off-phase of a
flashing buoy is pixel-identical to an unlit one (FlashTracker, oak_detector_core).

To see what the classifier says frame by frame: flash_enable false, vote_decay 0.0 and
vote_min 0.0 (flash_enable true bypasses the voter entirely).

Positions are REP-103 body axes in camera_link (x forward, y left, z up). The parameters
and their defaults are oak_detector_core.PARAMS.
"""
import json
import time

import numpy as np

from crusader_msgs.msg import Detection3D, Detection3DArray
from crusader_perception.oak_detector_core import (FlashTracker, LabelVoter, ShapeVoter,
                                                   TrackTable, buoy_patch, led_patch,
                                                   marking_shape)
# The cls_labels entry that means "unlit". Fixed by the dataset's folder names, not chosen
# at run time.
OFF_CLASS = "off"
SHAPE_NAMES = ("circle", "diamond")
# Padded-crop height below which the LED region is a couple of rows and its colour is
# whatever the resize invented.
MIN_CROP_PX = 10
# Where in the box to sample depth: centred, slightly below the middle, because a buoy's
# upper half is often against sky where stereo has nothing to match.
SAMPLE_V_RATIO = 0.55
HALF_W_LIMITS = (4, 12)
HALF_H_LIMITS = (5, 20)
FRAME_ID = "camera_link"

# Box colour by LED colour (BGR). Drawing only.
LED_COLORS = {"red": (0, 0, 255), "green": (0, 255, 0), "blue": (255, 128, 0),
              "yellow": (0, 255, 255), "white": (255, 255, 255), "off": (160, 160, 160)}
UNRESOLVED = (200, 200, 200)


class BuoyModel:
    name = "buoy"
    topic = "/crsd/oak/detections"
    msg_type = Detection3DArray

    def __init__(self, p, log):
        """Loads both engines: slow, and loud on failure."""
        from ultralytics import YOLO            # CUDA container only
        self.p = p
        self.log = log
        self.shapes = list(p["det_labels"])
        self.shape_source = str(p["shape_source"]).strip().lower()
        if self.shape_source not in ("detector", "geometry"):
            raise ValueError(f"shape_source must be 'detector' or 'geometry', "
                             f"not {p['shape_source']!r}")
        self.colours = list(p["cls_labels"])
        self.det_imgsz = (int(p["det_imgsz_height"]), int(p["det_imgsz_width"]))
        self.cls_imgsz = int(p["cls_imgsz"])
        # The flash tracker reads 1 - P(off): without an off class every light would look
        # solid, so flash detection is refused out loud instead.
        self.off_index = self.colours.index(OFF_CLASS) if OFF_CLASS in self.colours else None
        self.flash_enable = bool(p["flash_enable"]) and self.off_index is not None
        if p["flash_enable"] and self.off_index is None:
            log.error(f"{OFF_CLASS!r} is not in cls_labels ({', '.join(self.colours)}): flash "
                      "detection is OFF, so ENTRY and EXIT buoys look the same")

        self.shape_voter = ShapeVoter(min_share=p["shape_min_share"])
        self.tracks = TrackTable(iou_match=p["iou_match"], max_missed=int(p["max_missed"]))
        self.voter = LabelVoter(self.colours, decay=p["vote_decay"], min_vote=p["vote_min"])
        self.flash = FlashTracker(
            window_s=p["flash_window_s"], min_span_s=p["flash_min_span_s"],
            min_samples=int(p["flash_min_samples"]), solid_min=p["solid_min"],
            off_max=p["off_max"], colour_min_share=p["colour_min_share"],
            flash_period_s=p["flash_period_s"], flash_corr_min=p["flash_corr_min"])
        self.batched = True                 # until the engine says otherwise
        self.det_ms = self.cls_ms = self.shape_ms = 0.0
        self.counts = dict(detections=0, no_depth=0, no_crop=0, unresolved=0, no_shape=0)
        self.states = {}                    # track id -> latest FlashState
        self.drawn = []

        self._debug = None
        if p["flash_debug_path"]:
            try:
                self._debug = open(p["flash_debug_path"], "a", buffering=1)
                log.warn(f"flash debug log ON -> {p['flash_debug_path']}")
            except OSError as e:
                log.warn(f"flash debug log disabled: {e}")

        log.info(f"loading detector {p['det_engine']}")
        self.detector = YOLO(p["det_engine"], task="detect")
        log.info(f"loading classifier {p['cls_engine']}")
        self.classifier = YOLO(p["cls_engine"], task="classify")
        log.info(f"buoy model ready: shapes {', '.join(self.shapes)}, colours "
                 f"{', '.join(self.colours)}, flash detection "
                 f"{'ON' if self.flash_enable else 'OFF'}")

    def close(self):
        if self._debug is not None:
            try:
                self._debug.close()
            except OSError:
                pass
            self._debug = None

    # ------------------------------------------------------------------ one frame

    def run(self, frame):
        """frame (camera_node.Frame) -> Detection3DArray, published even when empty: "alive
        and saw nothing" and "dead" must not look the same to a consumer."""
        rgb = frame.bgr
        now = frame.t

        start = time.monotonic()
        result = self.detector.predict(rgb, verbose=False, imgsz=self.det_imgsz,
                                       conf=self.p["det_conf_min"])[0]
        self.det_ms = (time.monotonic() - start) * 1000.0

        boxes, shapes, confidences = [], [], []
        for box in result.boxes:
            index = int(box.cls[0])
            if not 0 <= index < len(self.shapes):
                self.log.error(f"detector returned class {index} but det_labels has "
                               f"{len(self.shapes)} names: check it against the engine",
                               throttle_duration_sec=10.0)
                continue
            boxes.append(tuple(int(v) for v in box.xyxy[0]))
            shapes.append(self.shapes[index])
            confidences.append(float(box.conf[0]))

        # stage 2: shape from the outline, full resolution; an abstention keeps the
        # detector's class
        shape_calls = [None] * len(boxes)
        if self.shape_source == "geometry":
            start = time.monotonic()
            for i, box in enumerate(boxes):
                whole = buoy_patch(rgb, box, pad=self.p["crop_pad"])
                if whole is None:
                    continue
                try:
                    shape_calls[i] = marking_shape(whole, names=SHAPE_NAMES,
                                                   min_radius=float(self.p["shape_min_radius"]))
                except Exception as e:  # noqa: BLE001 - a bad crop must not stop the camera
                    self.log.warn(f"marking_shape failed ({type(e).__name__}: {e})",
                                  throttle_duration_sec=10.0)
            self.shape_ms = (time.monotonic() - start) * 1000.0

        # stage 3: association before classification (see the module docstring)
        track_ids = self.tracks.update(boxes)
        live = self.tracks.live_ids
        self.voter.prune(live)
        self.flash.prune(live)
        self.shape_voter.prune(live)
        if self.shape_source == "geometry":
            # voted per track: the outline test abstains often and can flip at the boundary
            for i in range(len(boxes)):
                name, conf, _ = shape_calls[i] or (None, 0.0, None)
                voted, _share = self.shape_voter.update(track_ids[i], name, conf)
                if voted is not None:
                    shapes[i] = voted
                else:
                    self.counts["no_shape"] += 1
        self.states = {k: v for k, v in self.states.items() if k in live}

        # stage 4: the LED crop and its colour
        crops, cropped_index, led_boxes = [], [], {}
        for i, box in enumerate(boxes):
            patch, cut = led_patch(rgb, box, pad=self.p["crop_pad"],
                                   top_frac=self.p["crop_top_frac"], size=self.cls_imgsz,
                                   min_height_px=MIN_CROP_PX, return_box=True)
            if patch is None:
                self.counts["no_crop"] += 1
                continue
            crops.append(patch)
            cropped_index.append(i)
            led_boxes[i] = cut
        start = time.monotonic()
        by_index = dict(zip(cropped_index, self._classify(crops)))
        self.cls_ms = (time.monotonic() - start) * 1000.0

        # stage 5: light state, range, message
        array = Detection3DArray()
        array.header.stamp = frame.stamp
        array.header.frame_id = FRAME_ID
        self.drawn = []
        for i, box in enumerate(boxes):
            track_id = track_ids[i]
            probs = by_index.get(i)
            state, colour, colour_conf = None, None, 0.0
            if probs is not None:
                # The per-frame argmax, not the voted colour: smoothing across frames is
                # exactly what erases a flash.
                frame_index = int(np.argmax(probs))
                frame_colour = self.colours[frame_index]
                if frame_colour == OFF_CLASS:
                    frame_colour = None
                if self.flash_enable:
                    state = self.flash.update(track_id, now,
                                              1.0 - float(probs[self.off_index]), frame_colour)
                    self.states[track_id] = state
                    colour, colour_conf = state.colour, state.confidence
                else:
                    colour, colour_conf = self.voter.update(track_id, probs)
                if self._debug is not None:
                    self._write_debug(now, track_id, shapes[i], confidences[i], box, probs,
                                      frame_index, colour, state)

            if self.flash_enable and state is not None:
                label = state.label(shapes[i])
            elif colour:
                label = f"{colour}_{shapes[i]}"
            else:
                label = shapes[i]
            if colour is None:
                self.counts["unresolved"] += 1

            position, sample = self._position(box, frame)
            self.drawn.append({"box": box, "label": label, "id": track_id,
                               "conf": confidences[i], "colour": colour,
                               "colour_conf": colour_conf, "state": state, "pos": position,
                               "sample": sample, "led_box": led_boxes.get(i)})
            if position is None:
                # Sky behind the box, featureless water, or beyond stereo range: fusion
                # cannot use a detection without a position, so it is counted, not sent.
                self.counts["no_depth"] += 1
                continue
            d = Detection3D()
            d.label = label
            # The detector's "is this a buoy". The light state's confidence lives in the
            # label (an uncertain state degrades it) and on /crsd/camera/health.
            d.confidence = confidences[i]
            d.x, d.y, d.z = position
            x1, y1, x2, y2 = box
            d.bbox = [max(0, x1), max(0, y1), max(0, x2), max(0, y2)]
            array.detections.append(d)
        self.counts["detections"] += len(array.detections)
        return array

    def _write_debug(self, now, track_id, shape, det_conf, box, probs, frame_index, colour,
                     state):
        def r(v, n):
            return None if v is None else round(v, n)
        self._debug.write(json.dumps({
            "t": round(now, 4), "id": int(track_id), "shape": shape,
            "det_conf": round(det_conf, 3), "box": [int(v) for v in box],
            "probs": {n: round(float(v), 4) for n, v in zip(self.colours, probs)},
            "argmax": self.colours[frame_index], "colour": colour,
            "state": None if state is None else state.state,
            "basis": None if state is None else state.basis,
            "duty": None if state is None else r(state.duty, 4),
            "mod": None if state is None else r(state.modulation, 4),
        }) + "\n")

    def _classify(self, crops):
        """One probability vector per crop. A TensorRT engine has a fixed batch size, so the
        batched call is tried once and abandoned for good, loudly, if the engine refuses."""
        if not crops:
            return []
        if self.batched:
            try:
                results = self.classifier.predict(crops, imgsz=self.cls_imgsz, verbose=False)
                return [r.probs.data.cpu().numpy() for r in results]
            except Exception as e:  # noqa: BLE001
                self.batched = False
                self.log.warn(f"batched classification failed ({type(e).__name__}): one crop "
                              "at a time from now on. Re-export with a batch dimension: yolo "
                              f"export model=classifier.pt format=engine batch=8 "
                              f"imgsz={self.cls_imgsz} half=True")
        return [self.classifier.predict(c, imgsz=self.cls_imgsz, verbose=False)[0]
                .probs.data.cpu().numpy() for c in crops]

    def _position(self, box, frame):
        """((x, y, z) in camera_link or None, where depth was read).

        The median over a small patch, not the centre pixel: one pixel of stereo noise on an
        edge is metres of range error. The box and the bearing are in RGB pixels; only the
        depth lookup moves into depth pixels, by the plain ratio of the two sizes (depth is
        aligned to the colour camera's field of view).
        """
        if frame.depth is None:
            return None, None
        height, width = frame.bgr.shape[:2]
        x1, y1, x2, y2 = box
        x1, x2 = max(0, min(width - 1, x1)), max(0, min(width - 1, x2))
        y1, y2 = max(0, min(height - 1, y1)), max(0, min(height - 1, y2))
        if x2 <= x1 or y2 <= y1:
            return None, None
        u = int((x1 + x2) / 2)
        v = int(y1 + SAMPLE_V_RATIO * (y2 - y1))
        half_w = max(HALF_W_LIMITS[0], min(HALF_W_LIMITS[1], (x2 - x1) // 8))
        half_h = max(HALF_H_LIMITS[0], min(HALF_H_LIMITS[1], (y2 - y1) // 6))

        depth = frame.depth
        dh, dw = depth.shape[:2]
        sx, sy = dw / float(width), dh / float(height)
        du, dv = int(u * sx), int(v * sy)
        dhw, dhh = max(1, int(half_w * sx)), max(1, int(half_h * sy))
        patch = depth[max(0, dv - dhh):min(dh, dv + dhh + 1),
                      max(0, du - dhw):min(dw, du + dhw + 1)]
        valid = patch[(patch >= self.p["range_min_m"] * 1000.0)
                      & (patch <= self.p["range_max_m"] * 1000.0)]
        sample = (u, v, half_w, half_h, int(valid.size))
        if valid.size < self.p["min_depth_samples"]:
            return None, sample
        fx, fy, cx, cy = frame.intrinsics
        forward = float(np.median(valid)) / 1000.0
        right = (u - cx) * forward / fx
        down = (v - cy) * forward / fy
        return (forward, -right, -down), sample      # optical -> REP-103 body axes

    # ------------------------------------------------------------------ the picture

    def annotate(self, image):
        """Boxes and labels drawn onto `image` (a copy the caller owns). The box colour is
        read from the label, so the picture can never contradict the words."""
        import cv2
        fs = 0.34 * max(1.0, image.shape[1] / 640.0)    # legible once scaled to the view
        th = max(1, round(fs))
        for item in self.drawn:
            x1, y1, x2, y2 = item["box"]
            head = item["label"].split("_")
            if head[0] == "off":
                key = "off"
            elif head[0] in ("flash", "unknown") and len(head) > 1:
                key = head[1]
            else:
                key = head[0] if head[0] in LED_COLORS else None
            color = LED_COLORS.get(key, UNRESOLVED)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2 * th)
            bits = [f"#{item['id']}", item["label"], f"conf:{item['conf'] * 100:.0f}%"]
            state = item["state"]
            if state is not None and state.duty is not None:
                bits.append(f"lit:{state.duty:.2f}")
            elif item["colour"]:
                bits.append(f"vote:{item['colour_conf'] * 100:.0f}%")
            bits.append("no-depth" if item["pos"] is None
                        else f"{np.linalg.norm(item['pos']):.1f}m")
            cv2.putText(image, " ".join(bits), (x1, max(y1 - 5 * th, round(30 * fs))),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, color, th, cv2.LINE_AA)
        ranged = sum(1 for item in self.drawn if item["pos"] is not None)
        return (f"det {self.det_ms:.0f}ms shp[{self.shape_source[:3]}] {self.shape_ms:.0f}ms "
                f"cls {self.cls_ms:.0f}ms {ranged}/{len(self.drawn)} ranged "
                f"flash={'on' if self.flash_enable else 'OFF'}")

    def health(self):
        tracks = [{"id": int(k), "state": s.state, "colour": s.colour, "basis": s.basis,
                   "mod": None if s.modulation is None else round(s.modulation, 3),
                   "duty": None if s.duty is None else round(s.duty, 3),
                   "conf": round(s.confidence, 3), "n": s.samples, "span_s": round(s.span_s, 2)}
                  for k, s in sorted(self.states.items())]
        return dict(self.counts, det_ms=round(self.det_ms, 1), cls_ms=round(self.cls_ms, 1),
                    shape_ms=round(self.shape_ms, 1), shape_source=self.shape_source,
                    batched=self.batched, flash_enable=self.flash_enable, tracks=tracks)

    def warnings(self, fps):
        """What camera_node should say on its health tick."""
        # The frame rate is a correctness number here: a 0.5 Hz square wave needs more than
        # 1 Hz of sampling, and FlashTracker wants 3 fps to be comfortable.
        if self.flash_enable and 0.0 < fps < 3.0:
            return [f"{fps:.1f} fps is below the 3 fps the flash tracker needs: light "
                    "states will mostly read unknown"]
        return []
