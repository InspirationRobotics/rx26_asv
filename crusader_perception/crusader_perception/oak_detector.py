"""oak_detector — two-stage buoy detection with light-state tracking.

NOTE: unverified on the boat. Bench-run only, and deliberately NOT in
core.launch.py.

Owns the OAK-D, runs two TensorRT engines on each colour frame, ranges every box
against the aligned depth image, and publishes positions in `camera_link`:

  * `oak/detections`           crusader_msgs/Detection3DArray — EVERY frame
  * `crsd/oak_detector_health` std_msgs/String (JSON)         — per-track detail

  stage 1  det_engine   ->  where the buoys are, and what SHAPE each one is
  stage 2  cls_engine   ->  what COLOUR each LED is, from a crop of the box top
  stage 3  track + time ->  one stable answer per buoy, and whether it FLASHES

THE LABEL IS THE CONTRACT
-------------------------
`{state}_{colour}_{shape}`, e.g. `flash_red_diamond`, `solid_blue_circle`, or
`off_diamond`. Those are RobotX 2026's five Task 1 light states plus Task 2's
solid red/green survey indicators — the words mission logic branches on.

A state the tracker cannot yet call degrades to the bare colour, and an unknown
colour to the bare shape. It never guesses: calling a solid-blue EXIT buoy a
flashing-blue ENTRY buoy sends the boat to the wrong end of the course, and
`unknown` is far cheaper than wrong. `tools/bench/bench_flash.py` measures how
often each happens at a given classifier error rate.

WHY FLASHING IS NOT A CLASSIFIER CLASS
--------------------------------------
The off-phase of a flashing buoy is pixel-identical to an unlit one, so no
single frame distinguishes them and no per-frame model can learn to. It is a
property of a track over time. `FlashTracker` measures the duty cycle of
`1 - P(off)` over a window; 1 s ON / 1 s OFF puts a flashing light near 50%, a
solid one near 100% and an unlit one near 0%, whatever the phase.

It is fed the classifier's SOFT score, not its argmax. When the classifier
hedges at ~0.5, argmax turns a shrug into a full vote; the bench shows soft
input holding ~100% correct where hard labels start producing wrong verdicts.

THE VOTER AND THE FLASH TRACKER ARE ALTERNATIVES
------------------------------------------------
`LabelVoter` stabilises a colour by decaying old observations into new ones —
which is exactly what destroys a flash signal. With `flash_enable` true the
tracker owns the temporal answer and the voter is bypassed. With it false you
get the voted colour, and `vote_decay: 0.0` + `vote_min: 0.0` gives the raw
per-frame classification with no smoothing at all, for looking at what the model
actually says frame by frame.

WHY THIS NODE PUBLISHES NO FRAMES
---------------------------------
1.28 MB of raw RGB+depth per frame is ~38 MB/s at 30fps, to produce a few
hundred bytes of Detection3DArray. Nothing large leaves this process. Unlike
`buoy_detector` there is no `publish_frames` escape hatch — if you want pixels,
run `oakd_publisher`, which exists for exactly that.

IT CONTENDS FOR THE ONE OAK-D
------------------------------
Third client for a device that admits exactly one. `oakd_publisher`,
`buoy_detector` and this node are ALTERNATIVES: whichever starts first gets the
camera and the others fail to open it. It shares `stream_port` 8080 with
`buoy_detector` on purpose — they can never run together, so they can never
contend for the socket, and the ground station's camera tab finds whichever is
up with no extra wiring.

DETECTION RUNS ON THE ISP FRAME, NOT AN NN PREVIEW
---------------------------------------------------
The pipeline comes from `oak_pipeline.build_rgbd`, unchanged, so boxes, LED
crops and depth lookups are all in ONE pixel coordinate system. Running the
detector on a `setPreviewSize` stream means the preview is a centre-crop of the
ISP at a different scale, and every box has to be mapped back before a depth
lookup or a crop — a mapping that produces a plausible range rather than an
error when it is wrong. Using one frame for all three deletes that class of bug.

FRAME CONVENTION
----------------
Positions are REP-103 body axes in `camera_link`: x forward, y left, z up,
converted from the camera's optical frame (z forward, x right, y down) here, so
no consumer ever holds an optical convention.

WHERE IT RUNS
-------------
`asv`, the only image with both depthai and ultralytics/TensorRT. Both imports
are function-local, so `colcon build` and CI's import check still pass on a
machine with no SDK and no camera.
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
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config
from crusader_msgs.msg import Detection3D, Detection3DArray
from crusader_perception import oak_pipeline
from crusader_perception.oak_detector_core import (FlashTracker, LabelVoter,
                                                   TrackTable, led_patch)

MAX_GROUPS_PER_TICK = 2

# Where in the box to sample depth: centred horizontally, slightly BELOW centre
# vertically. A buoy's upper half is often against sky, where stereo has nothing
# to match. From gate_navigator, which earned these numbers on the water — and
# note it is the opposite end of the box from the LED crop, on purpose.
SAMPLE_V_RATIO = 0.55
HALF_W_LIMITS = (4, 12)
HALF_H_LIMITS = (5, 20)

PARAM_SPEC = {
    # -- stage 1: shape detector --
    "det_engine": dict(read_only=True,
                       description="TensorRT .engine for the shape detector"),
    "det_labels": dict(read_only=True,
                       description="shape class names IN ENGINE ORDER; an index "
                                   "mismatch silently mislabels every box"),
    "det_conf_min": dict(read_only=True, lo=0.0, hi=1.0,
                         description="minimum detector confidence"),
    "det_imgsz_height": dict(read_only=True, lo=64, hi=1280,
                             description="detector engine input height"),
    "det_imgsz_width": dict(read_only=True, lo=64, hi=1280,
                            description="detector engine input width"),
    # -- stage 2: LED colour classifier --
    "cls_engine": dict(read_only=True,
                       description="TensorRT .engine for the LED classifier"),
    "cls_labels": dict(read_only=True,
                       description="colour class names IN ENGINE ORDER — for an "
                                   "ultralytics classifier that is the training "
                                   "folder order, i.e. alphabetical"),
    "off_class": dict(read_only=True,
                      description="which cls_labels entry means 'unlit'. Its "
                                  "probability is what the flash tracker reads; "
                                  "a name that is not in cls_labels disables "
                                  "flash detection rather than guessing"),
    "cls_imgsz": dict(read_only=True, lo=32, hi=640,
                      description="classifier input size; MUST equal the crop "
                                  "size the training exporter wrote"),
    "crop_pad": dict(read_only=True, lo=0.0, hi=0.5,
                     description="box padding per side, as a fraction; MUST "
                                 "match the training exporter"),
    "crop_top_frac": dict(read_only=True, lo=0.05, hi=1.0,
                          description="fraction of the padded crop taken as the "
                                      "LED band; MUST match the exporter"),
    "min_crop_px": dict(read_only=True, lo=4, hi=200,
                        description="padded-crop height below which the LED band "
                                    "is too small to classify"),
    # -- stage 3: association --
    "iou_match": dict(read_only=True, lo=0.05, hi=0.9,
                      description="IoU above which a box continues a track"),
    "max_missed": dict(read_only=True, lo=1, hi=120,
                       description="frames a track survives without a match"),
    # -- stage 3: colour vote (bypassed when flash_enable) --
    "vote_decay": dict(read_only=True, lo=0.0, hi=0.999,
                       description="per-frame forgetting factor. 0.0 with "
                                   "vote_min 0.0 gives the RAW per-frame call"),
    "vote_min": dict(read_only=True, lo=0.0, hi=1.0,
                     description="accumulated share a colour needs to publish"),
    # -- stage 3: flash detection --
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
    "flash_lo": dict(read_only=True, lo=0.0, hi=1.0,
                     description="duty at or above this may be 'flashing'"),
    "flash_hi": dict(read_only=True, lo=0.0, hi=1.0,
                     description="duty at or below this may be 'flashing'"),
    "solid_min": dict(read_only=True, lo=0.0, hi=1.0,
                      description="duty at or above this is 'solid'"),
    "off_max": dict(read_only=True, lo=0.0, hi=1.0,
                    description="duty at or below this is 'off'. The gaps "
                                "between these four are DELIBERATE dead bands"),
    "colour_min_share": dict(read_only=True, lo=0.0, hi=1.0,
                             description="share of lit frames one colour needs "
                                         "before it is named"),
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
    "sync_threshold_ms": dict(read_only=True, lo=1, hi=200,
                              description="max RGB/depth pairing gap [ms]"),
    "subpixel": dict(read_only=True, description="StereoDepth subpixel mode"),
    "lr_check": dict(read_only=True, description="StereoDepth left/right check"),
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


class FrameBuffer:
    """Latest annotated frame + a condition to wake waiting HTTP clients.

    Holds exactly ONE frame: a queue would let a slow browser build a backlog and
    start showing the past, and the only question a bring-up viewer answers is
    what the camera sees NOW.

    `viewers` is read by the detection loop to skip annotation entirely when
    nobody is watching — drawing every frame for an audience of zero is CPU
    taken from two engines on a Jetson that has none to spare.
    """

    def __init__(self):
        self._cond = threading.Condition()
        self._frame = None
        self._seq = 0
        self.viewers = 0

    def put(self, frame):
        with self._cond:
            self._frame = frame
            self._seq += 1
            self._cond.notify_all()

    def get_after(self, last_seq, timeout=5.0):
        with self._cond:
            if not self._cond.wait_for(lambda: self._seq > last_seq, timeout):
                return None, last_seq
            return self._frame, self._seq


class OakDetector(Node):

    def __init__(self):
        super().__init__("oak_detector")
        p = declare_from_config(self, crsd_config.node_params("oak_detector"),
                                PARAM_SPEC)
        self.p = p
        self.shapes = list(p["det_labels"])
        self.colours = list(p["cls_labels"])
        self.frame_id = p["frame_id"]
        self.det_imgsz = (int(p["det_imgsz_height"]), int(p["det_imgsz_width"]))
        self.cls_imgsz = int(p["cls_imgsz"])

        # Which class means "unlit". The flash tracker reads 1 - P(off), so a
        # missing or misspelled name would silently make every light look solid.
        # Refuse instead: no off class, no flash detection, said out loud once.
        self.off_class = p["off_class"]
        self.off_index = (self.colours.index(self.off_class)
                          if self.off_class in self.colours else None)
        self.flash_enable = bool(p["flash_enable"]) and self.off_index is not None
        if bool(p["flash_enable"]) and self.off_index is None:
            self.get_logger().error(
                f"off_class {self.off_class!r} is not in cls_labels "
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
            flash_lo=p["flash_lo"], flash_hi=p["flash_hi"],
            solid_min=p["solid_min"], off_max=p["off_max"],
            colour_min_share=p["colour_min_share"])

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
        self.cv2 = None
        self.batched = True          # until the engine says otherwise
        self.fps = 0.0
        self.det_ms = self.cls_ms = 0.0
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
            self._start_stream(int(p["stream_port"]), int(p["stream_quality"]))

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
            subpixel=self.p["subpixel"], lr_check=self.p["lr_check"],
            sync_threshold_ms=self.p["sync_threshold_ms"])

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

        # -------- stage 3a: association, BEFORE classification --------
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
        self.states = {k: v for k, v in self.states.items() if k in live}

        # -------- stage 2: crop the LED band and classify --------
        crops, cropped_index = [], []
        for i, box in enumerate(boxes):
            patch = led_patch(rgb_frame, box, pad=self.p["crop_pad"],
                              top_frac=self.p["crop_top_frac"],
                              size=self.cls_imgsz,
                              min_height_px=int(self.p["min_crop_px"]))
            if patch is None:
                self.no_crop += 1
                continue
            crops.append(patch)
            cropped_index.append(i)

        start = time.monotonic()
        probabilities = self._classify(crops)
        self.cls_ms = (time.monotonic() - start) * 1000.0
        by_index = dict(zip(cropped_index, probabilities))

        # -------- stage 3b: light state, range, publish --------
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
                if frame_colour == self.off_class:
                    frame_colour = None

                if self.flash_enable:
                    lit_score = 1.0 - float(probs[self.off_index])
                    state = self.flash.update(track_id, now, lit_score,
                                              frame_colour)
                    self.states[track_id] = state
                    colour, colour_conf = state.colour, state.confidence
                    if self._debug is not None:
                        self._debug.write(json.dumps({
                            "t": round(now, 4), "id": int(track_id),
                            "lit": round(lit_score, 4),
                            "colour": frame_colour, "shape": shapes[i],
                            "state": state.state,
                            "duty": None if state.duty is None
                                    else round(state.duty, 4),
                        }) + "\n")
                else:
                    colour, colour_conf = self.voter.update(track_id, probs)

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
                          "pos": position, "sample": sample})

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
        """A copy of the frame with boxes, labels, ranges and the two sample
        regions. Copies rather than drawing in place: a viewer must never be able
        to alter what the detector saw.

        Both regions are drawn because they answer different questions. The LED
        band at the top is what the classifier read — if it is sitting on sky or
        on the hull, the colour is noise no matter what the duty cycle says. The
        depth patch lower down is what stereo ranged.

        The duty cycle is drawn as a number, not a word, because the words are
        thresholds over it: seeing 0.34 next to `unknown` tells you the dead band
        is doing its job, where the word alone reads as a failure.
        """
        cv2 = self.cv2
        image = rgb_frame.copy()

        for item in drawn:
            x1, y1, x2, y2 = item["box"]
            color = LED_COLORS.get(item["colour"], UNRESOLVED)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

            # The band the classifier actually saw, in the frame's own pixels.
            pad = self.p["crop_pad"]
            crop_h = (y2 - y1) * (1.0 + 2.0 * pad)
            band_y0 = int(round((y1 + y2) / 2.0 - crop_h / 2.0))
            band_y1 = int(round(band_y0 + crop_h * self.p["crop_top_frac"]))
            band_x0 = int(round((x1 + x2) / 2.0 - (x2 - x1) * (1.0 + 2 * pad) / 2))
            band_x1 = int(round(band_x0 + (x2 - x1) * (1.0 + 2 * pad)))
            cv2.rectangle(image, (band_x0, band_y0), (band_x1, band_y1), color, 1)

            text = f"#{item['id']} {item['label']} d{item['conf'] * 100:.0f}%"
            state = item["state"]
            if state is not None:
                duty = "--" if state.duty is None else f"{state.duty:.2f}"
                text += f" duty={duty} c{state.confidence * 100:.0f}% n{state.samples}"
            elif item["colour"]:
                text += f" c{item['colour_conf'] * 100:.0f}%"
            if item["pos"] is None:
                text += " NO DEPTH"
            else:
                x, y, z = item["pos"]
                # Range first: it is what you check against a tape measure.
                text += (f" {(x * x + y * y + z * z) ** 0.5:.1f}m "
                         f"[{x:.1f},{y:+.1f},{z:+.1f}]")
            cv2.putText(image, text, (x1, max(band_y0 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)

            sample = item["sample"]
            if sample is not None:
                u, v, half_w, half_h, valid = sample
                patch_color = (0, 0, 255) if item["pos"] is None else (255, 255, 255)
                cv2.rectangle(image, (u - half_w, v - half_h),
                              (u + half_w, v + half_h), patch_color, 1)
                cv2.circle(image, (u, v), 2, patch_color, -1)
                cv2.putText(image, str(valid), (u + half_w + 3, v),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, patch_color, 1)

        ranged = sum(1 for item in drawn if item["pos"] is not None)
        status = (f"{self.fps:.0f}fps  det {self.det_ms:.0f}ms  "
                  f"cls {self.cls_ms:.0f}ms  {ranged}/{len(drawn)} ranged  "
                  f"no_depth={self.no_depth} no_crop={self.no_crop}  "
                  f"flash={'on' if self.flash_enable else 'OFF'}  "
                  f"{self.width}x{self.height}  frame={self.frame_id}")
        cv2.putText(image, status, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1)
        return image

    def _start_stream(self, port, quality):
        """Serve the annotated frames as MJPEG over HTTP.

        Failure to bind is a WARNING, not a fatal error. This is a bring-up
        viewer; a busy port must not stop the boat from seeing buoys. The
        detections topic is the product — this is a convenience.
        """
        import cv2
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

        self.cv2 = cv2
        buffer = self.buffer
        logger = self.get_logger()

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.0"

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    return self._page()
                return self._stream()

            def _page(self):
                # object-fit:contain, NOT width:100%. The ground station embeds
                # this page in an iframe far wider than the frame, and a
                # width-only rule scales the height past the pane so the iframe
                # scrolls. `contain` fills whichever axis binds and letterboxes
                # the other, at any window size.
                # no-store because this markup changes and a cached copy of the
                # old page is indistinguishable from a fix that did not work.
                body = ("<html><head><title>oak_detector</title>"
                        "<meta name='viewport' content='width=device-width,"
                        "initial-scale=1'></head>"
                        "<body style='margin:0;height:100vh;overflow:hidden;"
                        "background:#111'>"
                        "<img src='/stream' style='width:100%;height:100%;"
                        "object-fit:contain;display:block'>"
                        "</body></html>").encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _stream(self):
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                with buffer._cond:
                    buffer.viewers += 1
                seq = 0
                try:
                    while True:
                        frame, seq = buffer.get_after(seq)
                        if frame is None:
                            continue          # no frames yet; keep waiting
                        ok, jpeg = cv2.imencode(
                            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
                        if not ok:
                            continue
                        payload = jpeg.tobytes()
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n"
                            b"Content-Length: " + str(len(payload)).encode()
                            + b"\r\n\r\n")
                        self.wfile.write(payload)
                        self.wfile.write(b"\r\n")
                except ConnectionError:
                    pass                      # tab closed; normal
                finally:
                    # Must always run: a leaked viewer count keeps the node
                    # annotating every frame for nobody, forever.
                    with buffer._cond:
                        buffer.viewers -= 1

            def log_message(self, *args):
                pass                          # keep the console for ROS logs

        try:
            self.server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        except OSError as e:
            self.server = None
            logger.warn(f"MJPEG view disabled — cannot bind port {port}: {e}")
            return

        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        logger.info(f"annotated view on http://<JETSON_IP>:{port} "
                    "(frames are only drawn while a browser is connected)")

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
                   "duty": None if s.duty is None else round(s.duty, 3),
                   "conf": round(s.confidence, 3), "n": s.samples,
                   "span_s": round(s.span_s, 2)}
                  for k, s in sorted(self.states.items())]
        self.pub_health.publish(String(data=json.dumps({
            "fps": round(self.fps, 2), "det_ms": round(self.det_ms, 1),
            "cls_ms": round(self.cls_ms, 1), "batched": self.batched,
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

    def destroy_node(self):
        if self.server is not None:
            # shutdown() before the device close: serve_forever runs on a daemon
            # thread that touches the frame buffer, and tearing the camera out
            # from under a mid-write handler is how a clean exit becomes a hang.
            try:
                self.server.shutdown()
                self.server.server_close()
            except Exception as e:
                self.get_logger().warn(f"stream shutdown failed: {e}")
            self.server = None
        if self._debug is not None:
            try:
                self._debug.close()
            except OSError:
                pass
            self._debug = None
        if self.device is not None:
            try:
                self.device.close()      # one client only: a leaked handle
            except Exception as e:       # blocks the next start
                self.get_logger().warn(f"device close failed: {e}")
            self.device = None
        super().destroy_node()


def main(args=None):
    run_node(OakDetector, args=args)


if __name__ == "__main__":
    main()