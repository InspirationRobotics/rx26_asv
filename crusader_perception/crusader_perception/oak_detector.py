"""oak_detector — buoys, their marking shape, their LED colour and light state.

NOTE: unverified on the boat. Bench-run only, and deliberately NOT in
core.launch.py — it contends for the one OAK-D with oakd_publisher and
buoy_detector, so which one runs is an operator choice.

  stage 1  det_engine     where the buoys are
  stage 2  marking_shape  diamond or circle, from the outline (CPU, no model)
  stage 3  TrackTable     one identity per buoy, across frames
  stage 4  cls_engine     what colour each LED is
  stage 5  FlashTracker   flashing / solid / off, from the track over time

Publishes `oak/detections` (Detection3DArray, every frame) and
`crsd/oak_detector_health` (JSON, per-track detail).

Stage 2 has no model in it, which is what lets stage 1 be a SINGLE-CLASS buoy
detector: at 15 m a marking spans a couple of feature-map cells and the detector
head is guessing, while the full-resolution crop is 60-200 px and the outline is
right there. Stage 3 runs before stage 4 on purpose — a crop that fails to cut
must not cost a track its history.

THE LABEL IS THE CONTRACT
-------------------------
    flashing   flash_red_diamond
    solid      red_diamond
    off        off_diamond
    no colour  diamond

Mission logic branches on these words, so they are the one thing here that other
packages depend on.

FLASHING IS THE MARKED CASE and steady is the plain object, because steady is
what a light does unless something is being signalled by it. Task 1's ENTRY buoy
is still distinguishable from its EXIT buoy — `flash_blue_circle` against
`blue_circle` — which is the one place the distinction decides where the boat
goes.

An undecided state publishes as solid; see FlashState.label for why that is a
trade rather than an oversight, and what not to do with a young track.

WHY FLASHING IS NOT A CLASSIFIER CLASS
--------------------------------------
The off-phase of a flashing buoy is pixel-identical to an unlit one, so no
single frame can tell them apart. It is a property of a track over time, and
FlashTracker computes it from one — see oak_detector_core.

TURNING THE TEMPORAL LAYERS OFF
-------------------------------
To see what the classifier actually says, frame by frame, with nothing smoothing
it:

    flash_enable: false     # stop FlashTracker owning the answer
    vote_decay:   0.0       # LabelVoter keeps only this frame
    vote_min:     0.0       # ...and publishes it however weak

All three matter. flash_enable true bypasses the voter entirely, so leaving it
on keeps the smoothing whatever the vote params say.

WHERE IT RUNS
-------------
`asv`, the only image with both depthai and ultralytics/TensorRT. Both imports
are function-local, so colcon build and CI's import check still pass on a
machine with no SDK and no camera.

Positions are REP-103 body axes in `camera_link` (x forward, y left, z up),
converted from the camera's optical frame here so no consumer ever holds an
optical convention.
"""
import json
import threading
import time

import numpy as np
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from crusader_common import config as crsd_config
from crusader_common.mjpeg_view import FrameBuffer, serve_mjpeg, stop_mjpeg
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config
from crusader_msgs.msg import Detection3D, Detection3DArray
from crusader_perception import oak_pipeline
from crusader_perception.oak_detector_core import (FlashTracker, LabelVoter,
                                                   ShapeVoter, TrackTable,
                                                   buoy_patch, led_patch,
                                                   marking_shape)

MAX_GROUPS_PER_TICK = 2

# Which cls_labels entry means "unlit". A constant, not a parameter: the
# training folders are named by this and ultralytics orders classes by folder
# name, so it is fixed by the dataset, not chosen at run time. The node still
# refuses to guess if it is absent from cls_labels — see __init__.
OFF_CLASS = "off"

# How long teardown waits for depthai to let go of the camera before giving up
# on it. Five seconds is the ground station's own TERM_GRACE_S, so this expires
# BEFORE it escalates to SIGKILL — the node gets to say what happened.
DEVICE_CLOSE_TIMEOUT_S = 3.0

# Padded-crop height below which the LED region is a couple of rows and its
# colour is whatever the resize invented. A constant: it follows from the crop
# geometry, not from anything an operator decides.
MIN_CROP_PX = 10

# Where in the box to sample depth: centred horizontally, slightly BELOW centre
# vertically. A buoy's upper half is often against sky, where stereo has nothing
# to match. From gate_navigator, which earned these numbers on the water — and
# note it is the opposite end of the box from the LED crop, on purpose.
SAMPLE_V_RATIO = 0.55
HALF_W_LIMITS = (4, 12)
HALF_H_LIMITS = (5, 20)

PARAM_SPEC = {
    # -- stage 1: buoy detector --
    "det_engine": dict(read_only=True,
                       description="TensorRT .engine for the shape detector"),
    "det_labels": dict(read_only=True,
                       description="detector class names IN ENGINE ORDER; an index "
                                   "mismatch silently mislabels every box"),
    "det_conf_min": dict(read_only=True, lo=0.0, hi=1.0,
                         description="minimum detector confidence"),
    "det_imgsz_height": dict(read_only=True, lo=64, hi=1280,
                             description="detector engine input height"),
    "det_imgsz_width": dict(read_only=True, lo=64, hi=1280,
                            description="detector engine input width"),
    # -- stage 2: shape, measured not predicted --
    "shape_source": dict(read_only=True,
                         description="'detector' = trust the detector's class. "
                                     "'geometry' = measure the marking's "
                                     "outline with shape_core (CPU, no model), "
                                     "which is what lets det_engine be a "
                                     "SINGLE-CLASS buoy model. 'geometry' "
                                     "falls back to the detector class whenever "
                                     "the measurement abstains"),
    "shape_min_radius": dict(read_only=True, lo=0.0, hi=100.0,
                             description="marking radius [px] below which the "
                                         "geometric test refuses. Measured on "
                                         "synthetic renders the call is "
                                         "unreliable under ~6 px and solid by "
                                         "~10; raise it to abstain more"),
    "shape_min_share": dict(read_only=True, lo=0.0, hi=1.0,
                            description="share of a track's weighted shape "
                                        "votes one answer needs. Abstentions "
                                        "are never votes"),
    # -- stage 4: LED colour classifier --
    "cls_engine": dict(read_only=True,
                       description="TensorRT .engine for the LED classifier"),
    "cls_labels": dict(read_only=True,
                       description="colour class names IN ENGINE ORDER — for an "
                                   "ultralytics classifier that is the training "
                                   "folder order, i.e. alphabetical"),
    "cls_imgsz": dict(read_only=True, lo=32, hi=640,
                      description="classifier input size; MUST equal the crop "
                                  "size the training exporter wrote"),
    "crop_pad": dict(read_only=True, lo=0.0, hi=0.5,
                     description="box padding per side, as a fraction; MUST "
                                 "match the training exporter"),
    "crop_top_frac": dict(read_only=True, lo=0.05, hi=1.0,
                          description="fraction of the padded crop taken as the "
                                      "LED band; MUST match the exporter"),
    # -- stage 3: association --
    "iou_match": dict(read_only=True, lo=0.05, hi=0.9,
                      description="IoU above which a box continues a track"),
    "max_missed": dict(read_only=True, lo=1, hi=120,
                       description="frames a track survives without a match"),
    # -- stage 4: colour vote (bypassed when flash_enable) --
    "vote_decay": dict(read_only=True, lo=0.0, hi=0.999,
                       description="per-frame forgetting factor. 0.0 with "
                                   "vote_min 0.0 gives the RAW per-frame call"),
    "vote_min": dict(read_only=True, lo=0.0, hi=1.0,
                     description="accumulated share a colour needs to publish"),
    # -- stage 5: flash detection --
    "flash_enable": dict(read_only=True,
                         description="report flashing/solid/off from the duty "
                                     "cycle. False falls back to the voter"),
    "flash_window_s": dict(read_only=True, lo=2.0, hi=30.0,
                           description="duty-cycle window [s]. Must span whole "
                                       "1 s ON / 1 s OFF periods"),
    "flash_min_span_s": dict(read_only=True, lo=1.0, hi=30.0,
                             description="observed time before any verdict [s]"),
    "flash_min_samples": dict(read_only=True, lo=2, hi=500,
                              description="frames in the window before any "
                                          "verdict; a 0.5 Hz square wave needs "
                                          ">1 Hz sampling"),
    "solid_min": dict(read_only=True, lo=0.0, hi=1.0,
                      description="duty at or above this is 'solid'"),
    "off_max": dict(read_only=True, lo=0.0, hi=1.0,
                    description="duty at or below this is 'off'. The gaps "
                                "between these four are DELIBERATE dead bands"),
    "colour_min_share": dict(read_only=True, lo=0.0, hi=1.0,
                             description="share of lit frames one colour needs "
                                         "before it is named"),
    "flash_period_s": dict(read_only=True, lo=0.2, hi=20.0,
                           description="the light's FULL cycle [s]. RobotX "
                                       "Task 1 is 1 s ON / 1 s OFF, so 2.0. "
                                       "The tracker correlates the lit score "
                                       "against itself half a period back"),
    "flash_corr_min": dict(read_only=True, lo=0.0, hi=1.0,
                           description="anti-correlation at half a period "
                                       "needed to call it flashing. This is "
                                       "SCALE-FREE, so a barely-visible green "
                                       "flash passes as easily as a bright red "
                                       "one, while noise of any amplitude does "
                                       "not"),
    "flash_debug_path": dict(read_only=True,
                             description="jsonl of every per-frame observation, "
                                         "for replaying real footage through the "
                                         "tracker off-boat. Empty = off"),
    # -- ranging --
    "range_min_m": dict(read_only=True, lo=0.05, hi=5.0,
                        description="depth below this is discarded as invalid"),
    "range_max_m": dict(read_only=True, lo=1.0, hi=100.0,
                        description="depth above this is discarded as invalid"),
    "min_depth_samples": dict(read_only=True, lo=1, hi=500,
                              description="valid depth pixels required to range "
                                          "a box; fewer = no position, no publish"),
    # -- output --
    "frame_id": dict(read_only=True,
                     description="frame the positions are expressed in (REP-103 "
                                 "body axes: x forward, y left, z up)"),
    "detections_topic": dict(read_only=True,
                             description="Detection3DArray topic (relative name)"),
    "health_topic": dict(read_only=True,
                         description="std_msgs/String JSON; per-track light "
                                     "state and where the frames went"),
    # -- camera; MUST match the other OAK-D nodes, see check_config.py --
    "fps": dict(read_only=True, lo=1.0, hi=60.0, description="camera frame rate [Hz]"),
    "isp_denominator": dict(read_only=True, lo=1, hi=8,
                            description="ISP downscale 1/N of 1920x1200"),
    "ae_compensation": dict(read_only=True, lo=-9, hi=9,
                            description="auto-exposure bias, EV steps; negative "
                                        "is darker. Must match the exposure the "
                                        "classifier's footage was shot at"),
    "poll_period_s": dict(read_only=True, lo=0.001, hi=1.0,
                          description="output-queue poll period [s]"),
    "queue_size": dict(read_only=True, lo=1, hi=30,
                       description="device output queue depth (non-blocking)"),
    "health_period_s": dict(read_only=True, lo=1.0, hi=60.0,
                            description="rate/health log period [s]"),
    # -- annotated view --
    "stream_enable": dict(read_only=True,
                          description="serve the annotated MJPEG view over HTTP"),
    "stream_port": dict(read_only=True, lo=1024, hi=65535,
                        description="HTTP port for the annotated MJPEG view"),
    "stream_quality": dict(read_only=True, lo=10, hi=100,
                           description="JPEG quality for the MJPEG view"),
}

# Box colour by LED colour, in OpenCV's BGR order. Drawing only — nothing here
# classifies. Getting red and green the right way round matters more than it
# looks: a gate is defined by which side each colour is on, so a viewer that
# draws them wrong has you "confirming" a correct detection as broken.
LED_COLORS = {"red": (0, 0, 255), "green": (0, 255, 0), "blue": (255, 128, 0),
              "yellow": (0, 255, 255), "white": (255, 255, 255),
              "off": (160, 160, 160)}
UNRESOLVED = (200, 200, 200)


class OakDetector(Node):

    def __init__(self):
        super().__init__("oak_detector")
        p = declare_from_config(self, crsd_config.node_params("oak_detector"),
                                PARAM_SPEC)
        self.p = p
        self.shapes = list(p["det_labels"])
        self.shape_source = str(p["shape_source"]).strip().lower()
        if self.shape_source not in ("detector", "geometry"):
            raise ValueError(f"shape_source must be 'detector' or 'geometry', "
                             f"got {p['shape_source']!r}")
        self.shape_names = ("circle", "diamond")
        self.shape_voter = ShapeVoter(min_share=p["shape_min_share"])
        self.no_shape = 0
        self.colours = list(p["cls_labels"])
        self.frame_id = p["frame_id"]
        self.det_imgsz = (int(p["det_imgsz_height"]), int(p["det_imgsz_width"]))
        self.cls_imgsz = int(p["cls_imgsz"])

        # Which class means "unlit". The flash tracker reads 1 - P(off), so a
        # missing or misspelled name would silently make every light look solid.
        # Refuse instead: no off class, no flash detection, said out loud once.
        self.off_index = (self.colours.index(OFF_CLASS)
                          if OFF_CLASS in self.colours else None)
        self.flash_enable = bool(p["flash_enable"]) and self.off_index is not None
        if bool(p["flash_enable"]) and self.off_index is None:
            self.get_logger().error(
                f"{OFF_CLASS!r} is not in cls_labels "
                f"({', '.join(self.colours)}) — FLASH DETECTION IS OFF. Every "
                "light will be reported by colour alone, with no flashing/solid "
                "distinction, which is the ENTRY vs EXIT call in Task 1.")

        self.pub_detections = self.create_publisher(
            Detection3DArray, p["detections_topic"], qos_profile_sensor_data)
        self.pub_health = self.create_publisher(String, p["health_topic"], 10)

        self.tracks = TrackTable(iou_match=p["iou_match"],
                                 max_missed=int(p["max_missed"]))
        self.voter = LabelVoter(self.colours, decay=p["vote_decay"],
                                min_vote=p["vote_min"])
        self.flash = FlashTracker(
            window_s=p["flash_window_s"], min_span_s=p["flash_min_span_s"],
            min_samples=int(p["flash_min_samples"]),
            solid_min=p["solid_min"], off_max=p["off_max"],
            colour_min_share=p["colour_min_share"],
            flash_period_s=p["flash_period_s"],
            flash_corr_min=p["flash_corr_min"])

        # Per-frame observation log, for replaying REAL classifier noise through
        # the tracker on a laptop. The bench proves the duty-cycle logic against
        # synthetic error; only real footage proves the error rate itself.
        self._debug = None
        if p["flash_debug_path"]:
            try:
                self._debug = open(p["flash_debug_path"], "a", buffering=1)
                self.get_logger().warn(
                    f"flash debug log ON -> {p['flash_debug_path']} "
                    "(one line per detection per frame; turn it off for a run)")
            except OSError as e:
                self.get_logger().warn(f"flash debug log disabled: {e}")

        self.device = None
        self.server = None
        self.buffer = FrameBuffer()
        self.batched = True          # until the engine says otherwise
        self.fps = 0.0
        self.det_ms = self.cls_ms = self.shape_ms = 0.0
        self.frames = 0
        self.detections = 0
        self.no_depth = 0
        self.no_crop = 0
        self.unresolved = 0
        self.states = {}             # track id -> latest FlashState
        self.last_health_frames = 0
        self.last_health_time = time.monotonic()

        self._load_models()
        self._open_device()
        if p["stream_enable"]:
            self.server = serve_mjpeg(int(p["stream_port"]),
                                      int(p["stream_quality"]), self.buffer,
                                      self.get_logger(), title="oak_detector")

        self.create_timer(p["poll_period_s"], self._drain)
        self.create_timer(p["health_period_s"], self._health)

    # ---------------- startup ----------------
    def _load_models(self):
        """Load both engines. Slow (tens of seconds each) and loud on failure: a
        detector that starts without its engine is worse than one that does not
        start, because the boat looks ready."""
        from ultralytics import YOLO            # CUDA container only

        self.get_logger().info(f"loading detector {self.p['det_engine']} …")
        self.detector = YOLO(self.p["det_engine"], task="detect")
        self.get_logger().info(f"loading classifier {self.p['cls_engine']} …")
        self.classifier = YOLO(self.p["cls_engine"], task="classify")
        self.get_logger().info(
            f"engines ready — shapes: {', '.join(self.shapes)} | "
            f"colours: {', '.join(self.colours)} | "
            f"flash detection {'ON' if self.flash_enable else 'OFF'}")

    def _open_device(self):
        import depthai as dai                   # sensor SDK, function-local

        # depthai v3 removed XLinkOut in favour of output queues. Naming the
        # version here turns "AttributeError deep in a builder" into a sentence
        # about a dependency that moved — see the Dockerfile's pin.
        if not hasattr(dai.node, "XLinkOut"):
            raise RuntimeError(
                f"depthai {dai.__version__} is the v3 API (no XLinkOut); "
                "oak_pipeline targets v2. Pin depthai==2.x in the Dockerfile "
                "and rebuild the image.")

        pipeline, self.width, self.height = oak_pipeline.build_rgbd(
            isp_denominator=self.p["isp_denominator"], fps=self.p["fps"],
            ae_compensation=int(self.p["ae_compensation"]))

        self.dai = dai
        self.device = dai.Device(pipeline)
        self.queue = self.device.getOutputQueue("rgbd", self.p["queue_size"],
                                                blocking=False)
        self.fx, self.fy, self.cx, self.cy = oak_pipeline.rgb_intrinsics(
            self.device, self.width, self.height)

        self.get_logger().info(
            f"OAK-D open: mxid={self.device.getMxId()}, "
            f"usb={self.device.getUsbSpeed().name}, {self.width}x{self.height}")
        warning = oak_pipeline.usb_warning(self.device)
        if warning:
            self.get_logger().warn(warning)
        self.get_logger().info(
            f"publishing {self.p['detections_topic']} in frame "
            f"'{self.frame_id}' (x forward, y left, z up)")

    # ---------------- frame pump ----------------
    def _drain(self):
        """Run the pipeline on up to MAX_GROUPS_PER_TICK frames.

        Bounded on purpose: two engines are far slower than the camera, so an
        unbounded drain would never return and would starve the executor — no
        health log, no parameter services, no clean shutdown, while the node
        still looked alive. Bounded, a backlog costs latency instead of control.
        """
        for _ in range(MAX_GROUPS_PER_TICK):
            group = self.queue.tryGet()
            if group is None:
                return

            messages = {name: message for name, message in group}
            rgb_message = messages.get("rgb")
            depth_message = messages.get("depth")
            if rgb_message is None or depth_message is None:
                continue

            rgb_frame = rgb_message.getCvFrame()
            depth_frame = depth_message.getFrame()
            if rgb_frame.shape[:2] != depth_frame.shape[:2]:
                self.get_logger().error(
                    "RGB/depth size mismatch — depth lookups would read the "
                    "wrong pixel; dropping frame", throttle_duration_sec=5.0)
                continue

            self._run_pipeline(rgb_frame, depth_frame,
                               self._ros_stamp(rgb_message))
            self.frames += 1

    def _run_pipeline(self, rgb_frame, depth_frame, stamp):
        now = time.monotonic()

        start = now
        result = self.detector.predict(rgb_frame, verbose=False,
                                       imgsz=self.det_imgsz,
                                       conf=self.p["det_conf_min"])[0]
        self.det_ms = (time.monotonic() - start) * 1000.0

        boxes, shapes, confidences = [], [], []
        for box in result.boxes:
            index = int(box.cls[0])
            if not 0 <= index < len(self.shapes):
                # An engine emitting a class the label list does not cover means
                # the two disagree; publishing under a wrong name is worse than
                # dropping, because the name is what consumers act on.
                self.get_logger().error(
                    f"detector returned class {index} but only "
                    f"{len(self.shapes)} shape labels are configured — check "
                    "`det_labels` against the engine", throttle_duration_sec=10.0)
                continue
            boxes.append(tuple(int(v) for v in box.xyxy[0]))
            shapes.append(self.shapes[index])
            confidences.append(float(box.conf[0]))

        # -------- stage 2: SHAPE, from the marking's outline --------
        # Runs on the full-resolution crop, on the CPU, with no model. This is
        # what allows det_engine to be a single-class "buoy" detector: at 15 m a
        # marking is a couple of feature-map cells wide and the detector head is
        # guessing, while the ISP crop of the same buoy is 60-200 px across and
        # the outline is right there. Abstentions fall back to the detector's
        # own class, so switching to 'detector' is always safe.
        shape_calls = [None] * len(boxes)
        if self.shape_source == "geometry":
            start = time.monotonic()
            for i, box in enumerate(boxes):
                whole = buoy_patch(rgb_frame, box, pad=self.p["crop_pad"])
                if whole is None:
                    continue
                try:
                    shape_calls[i] = marking_shape(
                        whole, names=self.shape_names,
                        min_radius=float(self.p["shape_min_radius"]))
                except Exception as e:      # a bad crop must not kill the node
                    self.get_logger().warn(
                        f"marking_shape failed ({type(e).__name__}: {e})",
                        throttle_duration_sec=10.0)
            self.shape_ms = (time.monotonic() - start) * 1000.0

        # -------- stage 3: association, BEFORE classification --------
        # A crop that fails to cut still keeps its track alive and its history
        # intact. Classifying first and associating only what classified would
        # drop the track of any buoy that briefly got too small to crop, and it
        # would come back as a new one with an empty flash window — which for a
        # 6 s window means six seconds of `unknown` for a light we had already
        # called.
        track_ids = self.tracks.update(boxes)
        live = self.tracks.live_ids
        self.voter.prune(live)
        self.flash.prune(live)
        self.shape_voter.prune(live)

        # Vote the shape per TRACK, not per frame. The geometric test abstains
        # often (range, edge-on, marking in shadow) and can flip at the
        # boundary; a track sees the same buoy dozens of times.
        if self.shape_source == "geometry":
            for i in range(len(boxes)):
                call = shape_calls[i]
                name, conf, _ = call if call else (None, 0.0, None)
                voted, share = self.shape_voter.update(track_ids[i], name, conf)
                if voted is not None:
                    shapes[i] = voted
                else:
                    self.no_shape += 1      # detector's own class stands
        self.states = {k: v for k, v in self.states.items() if k in live}

        # -------- stage 4: crop the LED and classify its COLOUR --------
        crops, cropped_index = [], []
        led_boxes = {}          # detection index -> the box led_patch CUT
        for i, box in enumerate(boxes):
            patch, cut = led_patch(rgb_frame, box, pad=self.p["crop_pad"],
                                   top_frac=self.p["crop_top_frac"],
                                   size=self.cls_imgsz,
                                   min_height_px=MIN_CROP_PX,
                                   return_box=True)
            if patch is None:
                self.no_crop += 1
                continue
            crops.append(patch)
            cropped_index.append(i)
            led_boxes[i] = cut

        start = time.monotonic()
        probabilities = self._classify(crops)
        self.cls_ms = (time.monotonic() - start) * 1000.0
        by_index = dict(zip(cropped_index, probabilities))

        # -------- stage 5: light state, range, publish --------
        array = Detection3DArray()
        array.header.stamp = stamp
        array.header.frame_id = self.frame_id
        drawn = []

        for i, box in enumerate(boxes):
            track_id = track_ids[i]
            probs = by_index.get(i)
            state = None
            colour = None
            colour_conf = 0.0

            if probs is not None:
                # The per-frame argmax, which is what the flash tracker needs —
                # NOT the voted colour. The voter's whole job is to smooth
                # across frames, and that is precisely what erases a flash.
                frame_index = int(np.argmax(probs))
                frame_colour = self.colours[frame_index]
                if frame_colour == OFF_CLASS:
                    frame_colour = None

                if self.flash_enable:
                    lit_score = 1.0 - float(probs[self.off_index])
                    state = self.flash.update(track_id, now, lit_score,
                                              frame_colour)
                    self.states[track_id] = state
                    colour, colour_conf = state.colour, state.confidence
                else:
                    colour, colour_conf = self.voter.update(track_id, probs)

                # WRITTEN WHETHER OR NOT FLASH IS ON. It used to live inside the
                # flash branch, so turning flash off to look at raw predictions
                # also turned off the log that shows you the raw predictions —
                # which is the one moment you actually want it.
                #
                # The full probability vector goes in, not just the argmax. A
                # confident wrong answer and a coin-flip that landed wrong are
                # different problems with different fixes, and the argmax alone
                # cannot tell them apart.
                if self._debug is not None:
                    self._debug.write(json.dumps({
                        "t": round(now, 4), "id": int(track_id),
                        "shape": shapes[i], "det_conf": round(confidences[i], 3),
                        "box": [int(v) for v in box],
                        "probs": {n: round(float(v), 4)
                                  for n, v in zip(self.colours, probs)},
                        "argmax": self.colours[frame_index],
                        "colour": colour,
                        "state": None if state is None else state.state,
                        "basis": None if state is None else state.basis,
                        "duty": None if state is None or state.duty is None
                                else round(state.duty, 4),
                        "mod": None if state is None or state.modulation is None
                               else round(state.modulation, 4),
                    }) + "\n")

            if self.flash_enable and state is not None:
                label = state.label(shapes[i])
            elif colour:
                label = f"{colour}_{shapes[i]}"
            else:
                label = shapes[i]
            if colour is None:
                self.unresolved += 1

            position, sample = self._position(box, depth_frame)

            # Recorded whether or not it ranged: a box the viewer shows with no
            # range is the single most useful thing on the stream when tuning.
            drawn.append({"box": box, "label": label, "id": track_id,
                          "conf": confidences[i], "colour": colour,
                          "colour_conf": colour_conf, "state": state,
                          "pos": position, "sample": sample,
                          "led_box": led_boxes.get(i)})

            if position is None:
                # No usable depth: sky behind the box, featureless water, or
                # beyond stereo range. Fusion cannot use a detection without a
                # position, so it is counted, not published.
                self.no_depth += 1
                continue

            detection = Detection3D()
            detection.label = label
            # The DETECTOR's confidence — "is this a buoy". The light state has
            # its own confidence and nowhere in Detection3D to live, so the
            # LABEL carries that gate (an uncertain state degrades the label)
            # and the number goes on the health topic for anyone who needs it.
            detection.confidence = confidences[i]
            detection.x, detection.y, detection.z = position
            x1, y1, x2, y2 = box
            detection.bbox = [max(0, x1), max(0, y1), max(0, x2), max(0, y2)]
            array.detections.append(detection)

        self.detections += len(array.detections)
        # Published even when empty — see Detection3DArray.msg. "Alive and saw
        # nothing" and "dead" must not look the same to a consumer.
        self.pub_detections.publish(array)

        if self.buffer.viewers > 0:
            self.buffer.put(self._annotate(rgb_frame, drawn))

    def _classify(self, crops):
        """Colour probabilities for a list of crops, one vector each.

        A TensorRT engine is built for a fixed batch size. Passing five crops to
        a batch-1 engine either raises or silently processes one, so the batched
        path is tried once and the node falls back permanently on failure —
        loudly, with the re-export command, because the per-crop loop is the slow
        path and nobody should be on it without knowing.
        """
        if not crops:
            return []
        if self.batched:
            try:
                results = self.classifier.predict(crops, imgsz=self.cls_imgsz,
                                                  verbose=False)
                return [r.probs.data.cpu().numpy() for r in results]
            except Exception as e:
                self.batched = False
                self.get_logger().warn(
                    f"batched classification failed ({type(e).__name__}) — "
                    "falling back to one crop at a time. Re-export with a batch "
                    "dimension: yolo export model=classifier.pt format=engine "
                    f"batch=8 imgsz={self.cls_imgsz} half=True")
        return [self.classifier.predict(c, imgsz=self.cls_imgsz,
                                        verbose=False)[0].probs.data.cpu().numpy()
                for c in crops]

    def _position(self, box, depth_frame):
        """((x, y, z) or None, sample) — position in camera_link metres, plus
        where depth was read so the viewer can show it.

        Median over a small patch rather than the centre pixel: one pixel of
        stereo noise on a buoy edge is metres of range error, and the median
        rejects the background pixels that inevitably fall inside a box.
        """
        x1, y1, x2, y2 = box
        height, width = depth_frame.shape[:2]
        x1 = max(0, min(width - 1, x1))
        x2 = max(0, min(width - 1, x2))
        y1 = max(0, min(height - 1, y1))
        y2 = max(0, min(height - 1, y2))
        if x2 <= x1 or y2 <= y1:
            return None, None

        u = int((x1 + x2) / 2)
        v = int(y1 + SAMPLE_V_RATIO * (y2 - y1))
        half_w = max(HALF_W_LIMITS[0], min(HALF_W_LIMITS[1], (x2 - x1) // 8))
        half_h = max(HALF_H_LIMITS[0], min(HALF_H_LIMITS[1], (y2 - y1) // 6))

        patch = depth_frame[max(0, v - half_h):min(height, v + half_h + 1),
                            max(0, u - half_w):min(width, u + half_w + 1)]
        valid = patch[(patch >= self.p["range_min_m"] * 1000.0)
                      & (patch <= self.p["range_max_m"] * 1000.0)]
        sample = (u, v, half_w, half_h, int(valid.size))

        if valid.size < self.p["min_depth_samples"]:
            return None, sample

        forward = float(np.median(valid)) / 1000.0          # mm -> m
        right = (u - self.cx) * forward / self.fx           # optical x
        down = (v - self.cy) * forward / self.fy            # optical y

        # Optical (z fwd, x right, y down) -> REP-103 body (x fwd, y left, z up).
        return (forward, -right, -down), sample

    # ---------------- annotated MJPEG view ----------------
    def _annotate(self, rgb_frame, drawn):
        """A copy of the frame with one box and one line of text per detection.

        Copies rather than drawing in place: a viewer must never be able to
        alter what the detector saw.

        THE BOX COLOUR IS DERIVED FROM THE LABEL, not from a second lookup.
        Those used to be two independent reads — the box took `state.colour`
        while the text took `state.label()` — and they disagreed exactly when it
        mattered: a track whose lit frames voted green but whose duty said OFF
        drew a GREEN box next to the text `off_diamond`. Both were "right"; they
        were answering different questions. One source now, so the picture
        cannot contradict the words.

        Everything else — modulation, which test decided, sample counts, the LED
        crop, the depth patch — is on crsd/oak_detector_health and in the debug
        jsonl. A stream is for "is it looking at the right thing"; numbers are
        for reading, and they read better as text than as rectangles.
        """
        import cv2                          # already loaded; a dict lookup

        image = rgb_frame.copy()

        for item in drawn:
            x1, y1, x2, y2 = item["box"]
            label = item["label"]
            # First token of the label decides the colour: off_diamond -> off,
            # flash_red_diamond -> red, red_diamond -> red, diamond -> nothing.
            head = label.split("_")
            if head[0] == "off":
                key = "off"
            elif head[0] in ("flash", "unknown") and len(head) > 1:
                key = head[1]
            elif head[0] in LED_COLORS:
                key = head[0]               # solid: the unmarked case
            else:
                key = None
            color = LED_COLORS.get(key, UNRESOLVED)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

            bits = [f"#{item['id']}", label, f"conf:{item['conf'] * 100:.0f}%"]
            state = item["state"]
            if state is not None and state.duty is not None:
                bits.append(f"lit:{state.duty:.2f}")
            elif item["colour"]:
                bits.append(f"vote:{item['colour_conf'] * 100:.0f}%")
            bits.append("no-depth" if item["pos"] is None
                        else f"{np.linalg.norm(item['pos']):.1f}m")
            cv2.putText(image, " ".join(bits), (x1, max(y1 - 5, 11)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.34, color, 1, cv2.LINE_AA)

        ranged = sum(1 for item in drawn if item["pos"] is not None)
        status = (f"{self.fps:.0f}fps  det {self.det_ms:.0f}ms  "
                  f"shp[{self.shape_source[:3]}] {self.shape_ms:.0f}ms  "
                  f"cls {self.cls_ms:.0f}ms  {ranged}/{len(drawn)} ranged  "
                  f"no_depth={self.no_depth} no_crop={self.no_crop}  "
                  f"flash={'on' if self.flash_enable else 'OFF'}  "
                  f"{self.width}x{self.height}")
        cv2.putText(image, status, (6, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                    (255, 255, 255), 1, cv2.LINE_AA)
        return image

    # ---------------- helpers ----------------
    def _ros_stamp(self, message):
        """Device timestamp -> ROS time. Stamping with "now" would fold USB,
        decode and two inference passes into the instant a consumer associates
        the detection with — which is precisely what fusion keys on."""
        latency = self.dai.Clock.now() - message.getTimestamp()
        nanoseconds = max(int(latency.total_seconds() * 1e9), 0)
        return (self.get_clock().now() - Duration(nanoseconds=nanoseconds)).to_msg()

    def _health(self):
        now = time.monotonic()
        delivered = self.frames - self.last_health_frames
        elapsed = max(now - self.last_health_time, 1e-6)
        self.last_health_frames = self.frames
        self.last_health_time = now
        self.fps = delivered / elapsed if delivered else 0.0

        # JSON first, so the topic keeps publishing even on a stalled camera —
        # a consumer judging health by arrival needs the silence to be real, and
        # "fps 0" is information where no message at all is ambiguous.
        tracks = [{"id": int(k), "state": s.state, "colour": s.colour,
                   "basis": s.basis,
                   "mod": None if s.modulation is None else round(s.modulation, 3),
                   "duty": None if s.duty is None else round(s.duty, 3),
                   "conf": round(s.confidence, 3), "n": s.samples,
                   "span_s": round(s.span_s, 2)}
                  for k, s in sorted(self.states.items())]
        self.pub_health.publish(String(data=json.dumps({
            "fps": round(self.fps, 2), "det_ms": round(self.det_ms, 1),
            "cls_ms": round(self.cls_ms, 1), "shape_ms": round(self.shape_ms, 1),
            "shape_source": self.shape_source, "no_shape": self.no_shape,
            "batched": self.batched,
            "frames": self.frames, "detections": self.detections,
            "no_depth": self.no_depth, "no_crop": self.no_crop,
            "unresolved": self.unresolved, "viewers": self.buffer.viewers,
            "flash_enable": self.flash_enable, "tracks": tracks,
        })))

        if delivered == 0:
            self.get_logger().warn(
                f"no frames since last check (total={self.frames}) — camera "
                "stalled, unplugged, or inference wedged")
            return

        # THE FRAME RATE IS A CORRECTNESS NUMBER HERE, not just performance. A
        # 0.5 Hz square wave needs more than 1 Hz of sampling; measured against
        # FlashTracker, 2 fps is the floor and 3+ is comfortable. Below that the
        # tracker reports `unknown` rather than guessing, but you want to know.
        if self.flash_enable and 0.0 < self.fps < 3.0:
            self.get_logger().warn(
                f"{self.fps:.1f} fps is below the 3 fps the flash tracker wants "
                "— light states will mostly read `unknown`. See "
                "tools/bench/bench_flash.py", throttle_duration_sec=30.0)

        # The counters worth watching, and they fail differently:
        #   no_depth    detector sees buoys, stereo cannot range them -> empty
        #               arrays, which downstream looks exactly like seeing nothing
        #   no_crop     boxes too small for an LED band -> shape published with
        #               no colour, which reads as a classifier problem and is not
        #   unresolved  no colour called; with flash on, usually a window that
        #               has not filled yet rather than a bad classifier
        states = {}
        for s in self.states.values():
            states[s.state] = states.get(s.state, 0) + 1
        summary = " ".join(f"{k}={v}" for k, v in sorted(states.items())) or "none"
        self.get_logger().info(
            f"{self.fps:.1f} fps  det={self.det_ms:.0f}ms cls={self.cls_ms:.0f}ms"
            f"{' (unbatched)' if not self.batched else ''}  "
            f"detections={self.detections}  no_depth={self.no_depth}  "
            f"no_crop={self.no_crop}  unresolved={self.unresolved}  "
            f"tracks={len(self.tracks.tracks)} [{summary}]  "
            f"viewers={self.buffer.viewers}")

    def _close_device(self):
        """Let go of the camera, or give up loudly after DEVICE_CLOSE_TIMEOUT_S.

        depthai tears down XLink over USB here, and a device that has been
        streaming for hours can sit in an uninterruptible kernel wait doing it.
        A node stuck there ignores SIGTERM, the ground station escalates to
        SIGKILL after its 5 s grace, and even that cannot reap a process in D
        state — which is why the whole ROS stack ends up being restarted.

        So: daemon thread, deadline, exit anyway. Leaking the handle costs the
        next start its camera; a node that will not die costs more, and the
        warning says which one you have.
        """
        device, self.device = self.device, None
        if device is None:
            return
        done = threading.Event()

        def _close():
            try:
                device.close()
            except Exception as e:
                self.get_logger().warn(f"device close failed: {e}")
            finally:
                done.set()

        threading.Thread(target=_close, daemon=True).start()
        if not done.wait(DEVICE_CLOSE_TIMEOUT_S):
            self.get_logger().error(
                f"OAK-D did not close within {DEVICE_CLOSE_TIMEOUT_S:.0f}s — "
                "abandoning the handle and exiting. The camera may be wedged on "
                "USB; if the next start cannot open it, replug the cable rather "
                "than restarting the ROS stack.")

    def destroy_node(self):
        # Viewer first: a stream handler blocks waiting for the next frame, so
        # closing the camera first leaves it waiting for one that never comes.
        stop_mjpeg(self.server, self.buffer, self.get_logger())
        self.server = None
        if self._debug is not None:
            try:
                self._debug.close()
            except OSError:
                pass
            self._debug = None
        self._close_device()
        super().destroy_node()


def main(args=None):
    run_node(OakDetector, args=args)


if __name__ == "__main__":
    main()