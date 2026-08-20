"""oak_detector — two-stage buoy detection. Frames in, labelled detections out.

NOTE: unverified on the boat. Bench-run only, and deliberately NOT in
core.launch.py.

Owns the OAK-D, runs two TensorRT engines on each colour frame, ranges every box
against the aligned depth image, and publishes positions in `camera_link`:

  * `oak/detections`  crusader_msgs/Detection3DArray  — EVERY frame, empty or not

  stage 1  det_engine  ->  where the buoys are, and what SHAPE each one is
  stage 2  cls_engine  ->  what COLOUR each one's LED is, from a crop of its top
  stage 3  track+vote  ->  one stable answer per buoy, not a per-frame flicker

The published label is `"{colour}_{shape}"` once the colour vote has converged
and bare `"{shape}"` until it has. An unresolved colour is reported as
unresolved rather than guessed: a gate is defined by which colour is on which
side, so a confident wrong colour is worse than an honest "shape only".

WHY THIS NODE PUBLISHES NO FRAMES
---------------------------------
Same trade `buoy_detector` makes, and it is the whole reason detection runs
beside the device: 1.28 MB of raw RGB+depth per frame is ~38 MB/s at 30fps, to
produce a few hundred bytes of Detection3DArray. Nothing large leaves this
process. Unlike `buoy_detector` there is no `publish_frames` escape hatch here —
if you want pixels, run `oakd_publisher`, which exists for exactly that.

The annotated MJPEG view is the way to watch this work (`stream_port`, default
8080). It costs nothing while no browser is attached.

IT CONTENDS FOR THE ONE OAK-D
------------------------------
Third client for a device that admits exactly one. `oakd_publisher`,
`buoy_detector` and this node are ALTERNATIVES: whichever starts first gets the
camera and the others fail to open it. Which one runs is an operator choice per
session, which is why none of them is in a launch file.

It shares `stream_port` 8080 with `buoy_detector` ON PURPOSE. They can never run
at the same time, so they can never contend for the socket — and the ground
station's camera tab then shows whichever one is running with no extra wiring.

DETECTION RUNS ON THE ISP FRAME, NOT AN NN PREVIEW
---------------------------------------------------
The pipeline comes from `oak_pipeline.build_rgbd`, unchanged, so this node sees
exactly the frame `buoy_detector` sees: ISP-scaled colour with depth aligned to
it and paired on-device. Boxes, LED crops and depth lookups are therefore all in
ONE pixel coordinate system.

That is not a simplification for its own sake. Running the detector on a
`setPreviewSize` stream means the preview is a centre-CROP of the ISP at a
different scale, so every box has to be mapped back into ISP pixels before a
depth lookup or a crop, and getting that mapping wrong produces a plausible
range rather than an error. Using one frame for all three deletes the mapping
and the class of bug that comes with it. The engine's own letterboxing handles
the aspect difference, exactly as it already does for `buoy_detector`.

FRAME CONVENTION
----------------
Positions are REP-103 body axes in `camera_link`: x forward, y left, z up,
converted from the camera's optical frame (z forward, x right, y down) here, so
no consumer ever holds an optical convention.

WHERE IT RUNS
-------------
`asv`, the only image with both depthai (the camera) and ultralytics/TensorRT
(the engines). Both imports are function-local, so `colcon build` and CI's
import check still pass on a machine with no SDK and no camera.
"""
import threading
import time

import numpy as np
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from crusader_common import config as crsd_config
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config
from crusader_msgs.msg import Detection3D, Detection3DArray
from crusader_perception import oak_pipeline
from crusader_perception.shape_led_core import LabelVoter, TrackTable, led_patch

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
    # -- stage 3: track + vote --
    "iou_match": dict(read_only=True, lo=0.05, hi=0.9,
                      description="IoU above which a box continues a track"),
    "max_missed": dict(read_only=True, lo=1, hi=120,
                       description="frames a track survives without a match"),
    "vote_decay": dict(read_only=True, lo=0.0, hi=0.999,
                       description="per-frame forgetting factor for colour votes"),
    "vote_min": dict(read_only=True, lo=0.0, hi=1.0,
                     description="accumulated share a colour needs before it is "
                                 "published at all"),
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

# Box colour by LED colour. Getting red and green the right way round matters
# more than it looks: a gate is defined by which side each colour is on, so a
# viewer that draws them wrong has you "confirming" a correct detection as broken.
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

        self.pub_detections = self.create_publisher(
            Detection3DArray, p["detections_topic"], qos_profile_sensor_data)

        self.tracks = TrackTable(iou_match=p["iou_match"],
                                 max_missed=int(p["max_missed"]))
        self.voter = LabelVoter(self.colours, decay=p["vote_decay"],
                                min_vote=p["vote_min"])

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
            f"colours: {', '.join(self.colours)}")

    def _open_device(self):
        import depthai as dai                   # sensor SDK, function-local

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
        start = time.monotonic()
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

        # -------- stage 3a: association, before classification --------
        # Association first so a crop that fails to cut still keeps its track
        # alive and its accumulated votes intact. Classifying first and
        # associating only what classified would drop the track of any buoy that
        # briefly got too small to crop, and it would come back as a new one.
        track_ids = self.tracks.update(boxes)
        self.voter.prune(self.tracks.live_ids)

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

        # -------- stage 3b: vote, range, publish --------
        array = Detection3DArray()
        array.header.stamp = stamp
        array.header.frame_id = self.frame_id
        drawn = []

        for i, box in enumerate(boxes):
            colour, colour_conf = (None, 0.0)
            if i in by_index:
                colour, colour_conf = self.voter.update(track_ids[i], by_index[i])
            if colour is None:
                self.unresolved += 1

            label = f"{colour}_{shapes[i]}" if colour else shapes[i]
            position, sample = self._position(box, depth_frame)

            # Recorded whether or not it ranged: a box the viewer shows with no
            # range is the single most useful thing on the stream when tuning.
            drawn.append({"box": box, "label": label, "id": track_ids[i],
                          "conf": confidences[i], "colour": colour,
                          "colour_conf": colour_conf, "pos": position,
                          "sample": sample})

            if position is None:
                # No usable depth: sky behind the box, featureless water, or
                # beyond stereo range. Fusion cannot use a detection without a
                # position, so it is counted, not published.
                self.no_depth += 1
                continue

            detection = Detection3D()
            detection.label = label
            # The DETECTOR's confidence — "is this a buoy". The colour's own
            # confidence has nowhere to live in Detection3D, so the label itself
            # carries that gate: a colour only appears once it cleared vote_min.
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
        on the hull, the colour is noise no matter what the vote says. The depth
        patch lower down is what stereo ranged.
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

            vote = f"{item['colour_conf'] * 100:.0f}%" if item["colour"] else "??"
            text = (f"#{item['id']} {item['label']} "
                    f"d{item['conf'] * 100:.0f}% c{vote}")
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
                # max-height, not width:100%. Embedded in the ground station's
                # camera tab the pane is far wider than 640x400, and a
                # width-only rule scales the height past the pane and makes the
                # iframe scroll. Contain-to-fit matches what the GCS already
                # does for a bare <img> in .viewer.
                body = ("<html><head><title>oak_detector</title>"
                        "<meta name='viewport' content='width=device-width,"
                        "initial-scale=1'></head>"
                        "<body style='margin:0;height:100vh;background:#111;"
                        "display:flex;align-items:center;justify-content:center'>"
                        "<img src='/stream' style='max-width:100%;"
                        "max-height:100vh;object-fit:contain;display:block'>"
                        "</body></html>").encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
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

        if delivered == 0:
            self.fps = 0.0
            self.get_logger().warn(
                f"no frames since last check (total={self.frames}) — camera "
                "stalled, unplugged, or inference wedged")
            return

        self.fps = delivered / elapsed
        # The three counters worth watching, and they fail differently:
        #   no_depth  detector sees buoys, stereo cannot range them -> empty
        #             arrays, which looks downstream exactly like seeing nothing
        #   no_crop   boxes too small for an LED band -> shape published without
        #             a colour, which reads as a classifier problem and is not
        #   unresolved votes not converging -> the classifier is being fed
        #             something it cannot call; check the band on the viewer
        self.get_logger().info(
            f"{self.fps:.1f} fps  det={self.det_ms:.0f}ms cls={self.cls_ms:.0f}ms"
            f"{' (unbatched)' if not self.batched else ''}  "
            f"detections={self.detections}  no_depth={self.no_depth}  "
            f"no_crop={self.no_crop}  unresolved={self.unresolved}  "
            f"tracks={len(self.tracks.tracks)}  viewers={self.buffer.viewers}")

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