"""buoy_detector — raw frames in, 3D detections out. Nothing else on the wire.

Owns the OAK-D, runs the TensorRT buoy engine on each colour frame, ranges every
box against the aligned depth image, and publishes positions in `camera_link`:

  * `oak/detections`  crusader_msgs/Detection3DArray  — EVERY frame, empty or not

WHY THIS NODE OWNS THE CAMERA
-----------------------------
`oakd_publisher` puts 1.28 MB of raw RGB+depth on the wire per frame — ~38 MB/s
at 30fps — so that a consumer in another process can look at pixels. But the
consumers that matter (fusion, the world model) do not want pixels. They want
"red buoy, 12.4 m, 8 degrees to port, at this instant". That is a few hundred
bytes. Detection here, beside the device, is what makes the transport question
disappear rather than get tuned: nothing large ever leaves this process.

The two nodes are ALTERNATIVES, not a pipeline. The OAK-D admits exactly one
client, so whichever starts first gets the camera and the other fails to open
it. Run `oakd_publisher` when a human needs to see frames; run this when the
boat needs to see buoys. `publish_frames` exists for the case where you want
both during bring-up, and it costs exactly the bandwidth it says it does.

WHERE THIS RUNS
---------------
`asv`, which is the only image with both of the things this node needs: depthai
(the camera) and ultralytics/TensorRT (the engine). It used to be neither —
`asv` asserted depthai was absent, under an older plan where a separate container
owned the OAK-D and published raw frames for detection to consume. This node is
what retired that plan: 1.28 MB frames at ~38 MB/s across a container boundary,
to produce a few hundred bytes of Detection3DArray, is the wrong trade. depthai
is now installed by the Dockerfile and the absence guard is gone.

It CONTENDS with `oakd_publisher` for the one OAK-D — only one may run.

FRAME CONVENTION
----------------
Positions are REP-103 body axes in `camera_link`: x forward, y left, z up. The
camera's own optical frame is z forward, x right, y down; the conversion happens
here, once, so no consumer ever has to remember which convention it is holding.
If your URDF puts camera_link somewhere other than the RGB sensor's optical
centre, that offset belongs in TF, not in this node.
"""
import threading
import time

import numpy as np
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from crusader_common import config as crsd_config
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config
from crusader_msgs.msg import Detection3D, Detection3DArray
from crusader_perception import oak_pipeline

MAX_GROUPS_PER_TICK = 2

# Where in the box to sample depth. Centred horizontally, slightly BELOW centre
# vertically (0.55): a buoy's upper half is often against sky, where stereo has
# nothing to match and returns invalid pixels. From gate_navigator, which earned
# these numbers on the water.
SAMPLE_V_RATIO = 0.55
HALF_W_LIMITS = (4, 12)
HALF_H_LIMITS = (5, 20)

PARAM_SPEC = {
    "engine": dict(read_only=True,
                   description="TensorRT .engine for the buoy detector"),
    "labels": dict(read_only=True,
                   description="class names IN ENGINE ORDER; index mismatch "
                               "silently mislabels every detection"),
    "conf_min": dict(read_only=True, lo=0.0, hi=1.0,
                     description="minimum detector confidence to publish"),
    "imgsz_height": dict(read_only=True, lo=64, hi=1280,
                         description="engine input height"),
    "imgsz_width": dict(read_only=True, lo=64, hi=1280,
                        description="engine input width"),
    "range_min_m": dict(read_only=True, lo=0.05, hi=5.0,
                        description="depth below this is discarded as invalid"),
    "range_max_m": dict(read_only=True, lo=1.0, hi=100.0,
                        description="depth above this is discarded as invalid"),
    "min_depth_samples": dict(read_only=True, lo=1, hi=500,
                              description="valid depth pixels required to range "
                                          "a box; fewer = no position, no publish"),
    "frame_id": dict(read_only=True,
                     description="frame the positions are expressed in (REP-103 "
                                 "body axes: x forward, y left, z up)"),
    "detections_topic": dict(read_only=True,
                             description="Detection3DArray topic (relative name)"),
    "publish_frames": dict(read_only=True,
                           description="also publish raw rgb/depth — BRING-UP "
                                       "ONLY, costs ~38 MB/s at 30fps"),
    "rgb_topic": dict(read_only=True, description="raw colour, if publish_frames"),
    "depth_topic": dict(read_only=True, description="raw depth, if publish_frames"),
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
    "stream_enable": dict(read_only=True,
                          description="serve the annotated MJPEG view over HTTP"),
    "stream_port": dict(read_only=True, lo=1024, hi=65535,
                        description="HTTP port for the annotated MJPEG view"),
    "stream_quality": dict(read_only=True, lo=10, hi=100,
                           description="JPEG quality for the MJPEG view"),
}

# Box colour by class family. Getting red and green right matters more than it
# looks: a gate is defined by which side each colour is on, so a viewer that
# draws them wrong will have you "confirming" a correct detection as broken.
FAMILY_COLORS = (("red", (0, 0, 255)), ("green", (0, 255, 0)),
                 ("yellow", (0, 255, 255)), ("blue", (255, 128, 0)),
                 ("black", (60, 60, 60)))


class FrameBuffer:
    """Latest annotated frame + a condition to wake waiting HTTP clients.

    Holds exactly ONE frame, like tools/oak_view.py: a queue would let a slow
    browser build a backlog and start showing the past, and the only question a
    bring-up viewer answers is what the camera sees NOW.

    `viewers` is read by the detection loop to skip annotation entirely when
    nobody is watching — drawing and copying every frame for an audience of zero
    is CPU taken from inference on a Jetson that has none to spare.
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
        """Block until a frame newer than `last_seq`. Returns (frame, seq), or
        (None, last_seq) on timeout so a dead stream is visible as a stall
        rather than a hang."""
        with self._cond:
            if not self._cond.wait_for(lambda: self._seq > last_seq, timeout):
                return None, last_seq
            return self._frame, self._seq


class BuoyDetector(Node):

    def __init__(self):
        super().__init__("buoy_detector")
        p = declare_from_config(self,
                                crsd_config.node_params("buoy_detector"),
                                PARAM_SPEC)
        self.p = p
        self.labels = list(p["labels"])
        self.frame_id = p["frame_id"]
        self.imgsz = (int(p["imgsz_height"]), int(p["imgsz_width"]))

        self.pub_detections = self.create_publisher(
            Detection3DArray, p["detections_topic"], qos_profile_sensor_data)
        self.pub_rgb = self.pub_depth = None
        if p["publish_frames"]:
            self.pub_rgb = self.create_publisher(Image, p["rgb_topic"],
                                                 qos_profile_sensor_data)
            self.pub_depth = self.create_publisher(Image, p["depth_topic"],
                                                   qos_profile_sensor_data)
            self.get_logger().warn(
                "publish_frames is ON — raw frames on the wire, bring-up only")

        self.device = None
        self.server = None
        self.buffer = FrameBuffer()
        self.cv2 = None
        self.fps = 0.0
        self.frames = 0
        self.detections = 0
        self.no_depth = 0
        self.last_health_frames = 0
        self.last_health_time = time.monotonic()

        self._load_model()
        self._open_device()
        if p["stream_enable"]:
            self._start_stream(int(p["stream_port"]), int(p["stream_quality"]))

        self.create_timer(p["poll_period_s"], self._drain)
        self.create_timer(p["health_period_s"], self._health)

    # ---------------- startup ----------------
    def _load_model(self):
        """Load the TensorRT engine. Slow (tens of seconds) and loud on failure:
        a detector that starts without its engine is worse than one that does not
        start, because the boat looks ready."""
        from ultralytics import YOLO            # CUDA container only

        self.get_logger().info(f"loading engine {self.p['engine']} …")
        self.model = YOLO(self.p["engine"], task="detect")
        self.get_logger().info(
            f"engine ready, {len(self.labels)} classes: {', '.join(self.labels)}")

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
        self.get_logger().info(
            f"intrinsics fx={self.fx:.1f} fy={self.fy:.1f} "
            f"cx={self.cx:.1f} cy={self.cy:.1f}")
        warning = oak_pipeline.usb_warning(self.device)
        if warning:
            self.get_logger().warn(warning)
        self.get_logger().info(
            f"publishing {self.p['detections_topic']} in frame "
            f"'{self.frame_id}' (x forward, y left, z up)")

    # ---------------- frame pump ----------------
    def _drain(self):
        """Detect on up to MAX_GROUPS_PER_TICK frames.

        Bounded on purpose: inference is far slower than the camera, so an
        unbounded drain would never return and starve the executor — no health
        log, no parameter services, no clean shutdown, while the node still
        looked alive. Bounded, a backlog costs latency instead of control.
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

            stamp = self._ros_stamp(rgb_message)
            self._detect_and_publish(rgb_frame, depth_frame, stamp)
            self.frames += 1

            if self.pub_rgb is not None:
                self.pub_rgb.publish(self._image(rgb_frame, "bgr8", stamp))
                self.pub_depth.publish(self._image(depth_frame, "16UC1", stamp))

    def _detect_and_publish(self, rgb_frame, depth_frame, stamp):
        result = self.model.predict(rgb_frame, verbose=False, imgsz=self.imgsz,
                                    conf=self.p["conf_min"])[0]

        array = Detection3DArray()
        array.header.stamp = stamp
        array.header.frame_id = self.frame_id
        drawn = []

        for box in result.boxes:
            class_index = int(box.cls[0])
            if not 0 <= class_index < len(self.labels):
                # An engine emitting a class the label list does not cover means
                # the two disagree; publishing it under a wrong name is worse
                # than dropping it, because the name is what consumers act on.
                self.get_logger().error(
                    f"engine returned class {class_index} but only "
                    f"{len(self.labels)} labels are configured — check the "
                    "`labels` param against the engine", throttle_duration_sec=10.0)
                continue

            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
            label = self.labels[class_index]
            confidence = float(box.conf[0])
            position, sample = self._position((x1, y1, x2, y2), depth_frame)

            # Recorded whether or not it ranged: a box the viewer shows with no
            # range is the single most useful thing on the stream when tuning.
            drawn.append({"box": (x1, y1, x2, y2), "label": label,
                          "conf": confidence, "pos": position, "sample": sample})

            if position is None:
                # No usable depth: sky behind the box, featureless water, or the
                # object beyond stereo range. A detection without a position is
                # not something fusion can use, so it is counted, not published.
                self.no_depth += 1
                continue

            detection = Detection3D()
            detection.label = label
            detection.confidence = confidence
            detection.x, detection.y, detection.z = position
            detection.bbox = [max(0, x1), max(0, y1), max(0, x2), max(0, y2)]
            array.detections.append(detection)

        self.detections += len(array.detections)
        # Published even when empty — see Detection3DArray.msg. "Alive and saw
        # nothing" and "dead" must not look the same to a consumer.
        self.pub_detections.publish(array)

        if self.buffer.viewers > 0:
            self.buffer.put(self._annotate(rgb_frame, drawn))

    def _position(self, box, depth_frame):
        """((x, y, z) or None, sample) — position in camera_link metres, plus
        where the depth was read so the viewer can show it.

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
        """A copy of the frame with boxes, ranges and depth sample points.

        Copies rather than drawing in place: the caller may still be publishing
        that array's source frame, and a viewer must never be able to alter what
        the detector saw.
        """
        cv2 = self.cv2
        image = rgb_frame.copy()

        for item in drawn:
            x1, y1, x2, y2 = item["box"]
            color = next((c for prefix, c in FAMILY_COLORS
                          if item["label"].startswith(prefix)), (200, 200, 200))
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

            if item["pos"] is None:
                text = f"{item['label']} {item['conf'] * 100:.0f}% NO DEPTH"
            else:
                x, y, z = item["pos"]
                # Range first: it is what you check against a tape measure.
                text = (f"{item['label']} {item['conf'] * 100:.0f}% "
                        f"{(x * x + y * y + z * z) ** 0.5:.1f}m "
                        f"[{x:.1f},{y:+.1f},{z:+.1f}]")
            cv2.putText(image, text, (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)

            sample = item["sample"]
            if sample is not None:
                # Where depth was actually read, and how many pixels survived
                # the range gate. A box sitting over sky shows an empty patch
                # here, which is the difference between "detector is wrong" and
                # "stereo had nothing to match".
                u, v, half_w, half_h, valid = sample
                patch_color = (0, 0, 255) if item["pos"] is None else (255, 255, 255)
                cv2.rectangle(image, (u - half_w, v - half_h),
                              (u + half_w, v + half_h), patch_color, 1)
                cv2.circle(image, (u, v), 2, patch_color, -1)
                cv2.putText(image, str(valid), (u + half_w + 3, v),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, patch_color, 1)

        ranged = sum(1 for item in drawn if item["pos"] is not None)
        status = (f"{self.fps:.0f}fps  {ranged}/{len(drawn)} ranged  "
                  f"no_depth={self.no_depth}  {self.width}x{self.height}  "
                  f"frame={self.frame_id}")
        cv2.putText(image, status, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1)
        return image

    def _start_stream(self, port, quality):
        """Serve the annotated frames as MJPEG over HTTP.

        Failure to bind is a WARNING, not a fatal error. This is a bring-up
        viewer; a busy port (an oak_view left running, a second detector) must
        not stop the boat from detecting buoys. The detections topic is the
        product — this is a convenience.
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
                body = ("<html><head><title>buoy_detector</title>"
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
                    # Must always run: a leaked viewer count keeps the detector
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
        decode and inference latency into the instant a consumer associates the
        detection with — which is precisely what a fusion node keys on."""
        latency = self.dai.Clock.now() - message.getTimestamp()
        nanoseconds = max(int(latency.total_seconds() * 1e9), 0)
        return (self.get_clock().now() - Duration(nanoseconds=nanoseconds)).to_msg()

    def _image(self, frame, encoding, stamp):
        frame = np.ascontiguousarray(frame)
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.height, message.width = frame.shape[:2]
        message.encoding = encoding
        message.is_bigendian = 0
        message.step = int(frame.strides[0])
        message.data = frame.tobytes()
        return message

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
        # no_depth is the number worth watching in the field: a detector that
        # sees buoys but cannot range them produces empty arrays and looks, from
        # downstream, exactly like a detector that sees nothing.
        self.get_logger().info(
            f"{self.fps:.1f} fps  detections={self.detections}  "
            f"no_depth={self.no_depth}  viewers={self.buffer.viewers}")

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
    run_node(BuoyDetector, args=args)


if __name__ == "__main__":
    main()
