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

CONTAINER REQUIREMENT — READ THIS BEFORE DEPLOYING
--------------------------------------------------
This node needs BOTH depthai (the camera) and ultralytics/TensorRT (the engine),
and today NEITHER container has both: `asv` asserts at build time that depthai is
absent, and the sensor container carries no CUDA stack. Co-locating detection
with the device is exactly the split this repo drew on purpose, so running this
node means deliberately redrawing it — add depthai to `asv` (and delete the
Dockerfile guard that forbids it) or add the CUDA/TensorRT stack to the sensor
image. That is an architecture decision, not a build fix, and it should be made
explicitly rather than discovered when the import fails on the water.

FRAME CONVENTION
----------------
Positions are REP-103 body axes in `camera_link`: x forward, y left, z up. The
camera's own optical frame is z forward, x right, y down; the conversion happens
here, once, so no consumer ever has to remember which convention it is holding.
If your URDF puts camera_link somewhere other than the RGB sensor's optical
centre, that offset belongs in TF, not in this node.
"""
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
from crusader_sensors import oak_pipeline

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
}


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
        self.frames = 0
        self.detections = 0
        self.no_depth = 0
        self.last_health_frames = 0
        self.last_health_time = time.monotonic()

        self._load_model()
        self._open_device()

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
            position = self._position((x1, y1, x2, y2), depth_frame)
            if position is None:
                # No usable depth: sky behind the box, featureless water, or the
                # object beyond stereo range. A detection without a position is
                # not something fusion can use, so it is counted, not published.
                self.no_depth += 1
                continue

            detection = Detection3D()
            detection.label = self.labels[class_index]
            detection.confidence = float(box.conf[0])
            detection.x, detection.y, detection.z = position
            detection.bbox = [max(0, x1), max(0, y1), max(0, x2), max(0, y2)]
            array.detections.append(detection)

        self.detections += len(array.detections)
        # Published even when empty — see Detection3DArray.msg. "Alive and saw
        # nothing" and "dead" must not look the same to a consumer.
        self.pub_detections.publish(array)

    def _position(self, box, depth_frame):
        """(x, y, z) in camera_link metres, or None if the box has no depth.

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
            return None

        u = int((x1 + x2) / 2)
        v = int(y1 + SAMPLE_V_RATIO * (y2 - y1))
        half_w = max(HALF_W_LIMITS[0], min(HALF_W_LIMITS[1], (x2 - x1) // 8))
        half_h = max(HALF_H_LIMITS[0], min(HALF_H_LIMITS[1], (y2 - y1) // 6))

        patch = depth_frame[max(0, v - half_h):min(height, v + half_h + 1),
                            max(0, u - half_w):min(width, u + half_w + 1)]
        valid = patch[(patch >= self.p["range_min_m"] * 1000.0)
                      & (patch <= self.p["range_max_m"] * 1000.0)]
        if valid.size < self.p["min_depth_samples"]:
            return None

        forward = float(np.median(valid)) / 1000.0          # mm -> m
        right = (u - self.cx) * forward / self.fx           # optical x
        down = (v - self.cy) * forward / self.fy            # optical y

        # Optical (z fwd, x right, y down) -> REP-103 body (x fwd, y left, z up).
        return forward, -right, -down

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
            self.get_logger().warn(
                f"no frames since last check (total={self.frames}) — camera "
                "stalled, unplugged, or inference wedged")
            return

        # no_depth is the number worth watching in the field: a detector that
        # sees buoys but cannot range them produces empty arrays and looks, from
        # downstream, exactly like a detector that sees nothing.
        self.get_logger().info(
            f"{delivered / elapsed:.1f} fps  "
            f"detections={self.detections}  no_depth={self.no_depth}")

    def destroy_node(self):
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
