"""oakd_publisher — the OAK-D driver node: RGB + aligned depth onto ROS topics.

Publishes:
  * `oak/rgb`    sensor_msgs/Image, bgr8   — the ISP-scaled colour frame
  * `oak/depth`  sensor_msgs/Image, 16UC1  — millimetres, aligned to the RGB frame

Both are relative names, so `ros2 run ... --ros-args -r __ns:=/bow` (or the
`rgb_topic`/`depth_topic` params) moves the whole pair without touching code.

WHY THIS NODE EXISTS AT ALL
---------------------------
`depthai_ros_driver` publishes RGB and depth too, but it does not guarantee that
a given RGB frame and a given depth frame are the SAME instant: they arrive as
independent streams and the consumer is left to match headers. Everything
downstream of this node (buoy ranging) reads a depth pixel at an RGB bounding
box, and a half-frame of yaw at 30fps is metres of error at 20m. So the pairing
is done ON THE DEVICE — a dai.node.Sync with a hard threshold — and the two
images leave here already matched. If a group is incomplete, nothing is
published for that instant; a dropped frame is cheap, a mismatched pair is not.

The pipeline (1200p, ISP 1/3 -> 640x400, depth aligned to CAM_A and output at
the same size) is ported from the prequal-proven gate_navigator in
InspirationRobotics/robotx_2026. Keeping the geometry identical means the RGB
intrinsics apply to the depth image unchanged, which is the property that makes
`depth[v, u]` legal for an (u, v) taken off an RGB detection box.

ONLY ONE OAK-D CLIENT
---------------------
Runs in `asv`, which carries depthai for this package. The device admits exactly
one client, so this node and `buoy_detector` CONTEND: whichever starts first gets
the camera and the other fails to open it. That is why neither is in
core.launch.py — which one runs is an operator choice per session, not a
constant. Prefer `buoy_detector` unless you specifically want raw pixels; this
node costs ~38 MB/s at 30fps to publish frames nothing but a human reads.

`depthai` is imported inside `_open_device`, not at module scope, so `colcon
build` and CI's import check still pass on a machine with no SDK and no camera.

Parameters come from crusader_bringup's `crusader_params.yaml` like every other
node here, so a value can never drift between a code default and the launched
file (crusader_common/config.py resolves $CRUSADER_PARAMS -> installed share ->
source tree).

Manual check:
  ros2 topic hz /oak/rgb
  python3 tools/oak_view.py --topic /oak/rgb        # raw topic: needs cv2+numpy
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
from crusader_perception import oak_pipeline

# Frames published per poll tick. At the default 10 ms poll that is 300/s of
# capacity against a 30 fps camera — headroom to catch up after a hiccup, with a
# hard ceiling so the callback always returns to the executor.
MAX_GROUPS_PER_TICK = 3

RGB_ENCODING = "bgr8"
# 16UC1 in millimetres is what StereoDepth emits and what depth_image_proc and
# every OpenCV consumer expect. Do NOT "helpfully" convert to 32FC1 metres here:
# that doubles the wire size of every frame for a division the consumer can do
# on the handful of pixels it actually reads.
DEPTH_ENCODING = "16UC1"

PARAM_SPEC = {
    "rgb_topic": dict(read_only=True,
                      description="colour Image topic (relative name)"),
    "depth_topic": dict(read_only=True,
                        description="aligned depth Image topic (relative name)"),
    "frame_id": dict(read_only=True,
                     description="header.frame_id for BOTH images — depth is "
                                 "aligned to the RGB camera, so they share a frame"),
    "fps": dict(read_only=True, lo=1.0, hi=60.0,
                description="camera frame rate [Hz]"),
    "isp_denominator": dict(read_only=True, lo=1, hi=8,
                            description="ISP downscale 1/N of 1920x1200 "
                                        "(3 -> 640x400, the stereo-native size)"),
    "sync_threshold_ms": dict(read_only=True, lo=1, hi=200,
                              description="max RGB/depth timestamp gap the "
                                          "device Sync will pair [ms]"),
    "subpixel": dict(read_only=True,
                     description="StereoDepth subpixel mode (finer far-range depth)"),
    "lr_check": dict(read_only=True,
                     description="StereoDepth left/right check (rejects occlusions)"),
    "poll_period_s": dict(read_only=True, lo=0.001, hi=1.0,
                          description="output-queue poll period [s]; must be well "
                                      "under 1/fps or frames queue up"),
    "queue_size": dict(read_only=True, lo=1, hi=30,
                       description="device output queue depth (non-blocking)"),
    "health_period_s": dict(read_only=True, lo=1.0, hi=60.0,
                            description="frame-arrival health check period [s]"),
}


class OakDPublisher(Node):

    def __init__(self):
        super().__init__("oakd_publisher")
        p = declare_from_config(self,
                                crsd_config.node_params("oakd_publisher"),
                                PARAM_SPEC)

        self.frame_id = p["frame_id"]
        self.width, self.height = oak_pipeline.output_size(p["isp_denominator"])

        self.pub_rgb = self.create_publisher(Image, p["rgb_topic"],
                                             qos_profile_sensor_data)
        self.pub_depth = self.create_publisher(Image, p["depth_topic"],
                                               qos_profile_sensor_data)

        self.device = None
        self.queue = None
        self.frames = 0
        self.incomplete = 0
        self.last_health_frames = 0
        self.last_health_time = time.monotonic()

        self._open_device(p)

        # Sensor-data QoS (BEST_EFFORT) on both. A RELIABLE subscriber matches a
        # BEST_EFFORT publisher not at all — `ros2 topic list` looks perfect and
        # zero frames flow. Consumers must use qos_profile_sensor_data.
        self.get_logger().info(
            f"publishing {self.width}x{self.height} @ {p['fps']:.0f}fps: "
            f"{p['rgb_topic']} ({RGB_ENCODING}), "
            f"{p['depth_topic']} ({DEPTH_ENCODING}, mm, aligned)")

        self.create_timer(p["poll_period_s"], self._drain)
        self.create_timer(p["health_period_s"], self._health)

    # ---------------- device ----------------
    def _open_device(self, p):
        """Build the pipeline and open the camera. Failure raises: run_node lets
        the exception propagate, so a missing/held camera is a loud startup
        failure rather than a node that sits there publishing nothing."""
        import depthai as dai                     # function-local: see docstring

        pipeline, self.width, self.height = oak_pipeline.build_rgbd(
            isp_denominator=p["isp_denominator"], fps=p["fps"],
            subpixel=p["subpixel"], lr_check=p["lr_check"],
            sync_threshold_ms=p["sync_threshold_ms"])

        self.dai = dai
        self.device = dai.Device(pipeline)
        # blocking=False: the camera must never stall waiting for this node, and
        # a backlog would mean publishing the past.
        self.queue = self.device.getOutputQueue("rgbd", maxSize=p["queue_size"],
                                                blocking=False)

        self.get_logger().info(
            f"OAK-D open: mxid={self.device.getMxId()}, "
            f"usb={self.device.getUsbSpeed().name}")
        warning = oak_pipeline.usb_warning(self.device)
        if warning:
            self.get_logger().warn(warning)

    # ---------------- frame pump ----------------
    def _drain(self):
        """Publish up to MAX_GROUPS_PER_TICK complete groups.

        The bound is not an optimization, it is the difference between a node
        and a hang. Draining "everything waiting" looks right until publishing
        is slower than the camera: the queue then never empties, this callback
        never returns, and a single-threaded executor runs NOTHING else — no
        health log, no parameter service, no clean Ctrl+C. The node still
        publishes, so from outside it looks alive and merely slow, which is the
        worst way to be broken. Bounded, a backlog costs latency instead of
        control, and the health line reports the rate that reveals it.
        """
        for _ in range(MAX_GROUPS_PER_TICK):
            group = self.queue.tryGet()
            if group is None:
                return

            messages = {name: message for name, message in group}
            rgb_message = messages.get("rgb")
            depth_message = messages.get("depth")

            if rgb_message is None or depth_message is None:
                # Sync emitted a partial group: publishing half a pair would
                # hand a consumer an RGB frame with someone else's depth.
                self.incomplete += 1
                self.get_logger().warn(
                    f"incomplete sync group {sorted(messages)} — dropped",
                    throttle_duration_sec=5.0)
                continue

            rgb_frame = rgb_message.getCvFrame()
            depth_frame = depth_message.getFrame()

            if rgb_frame.shape[:2] != depth_frame.shape[:2]:
                # The alignment contract is broken; every downstream depth
                # lookup would silently read the wrong pixel.
                self.get_logger().error(
                    "RGB/depth size mismatch: "
                    f"rgb={rgb_frame.shape[:2]} depth={depth_frame.shape[:2]} — "
                    "check isp_denominator against stereo.setOutputSize",
                    throttle_duration_sec=5.0)
                continue

            # One stamp for the pair, taken from the RGB message: they are the
            # same instant by construction, and a consumer matching on
            # header.stamp must find them equal.
            stamp = self._ros_stamp(rgb_message)
            self.pub_rgb.publish(self._image(rgb_frame, RGB_ENCODING, stamp))
            self.pub_depth.publish(self._image(depth_frame, DEPTH_ENCODING, stamp))
            self.frames += 1

    def _ros_stamp(self, message):
        """Device timestamp -> ROS time.

        dai stamps frames on the host steady clock (dai.Clock.now()), which is
        not the ROS clock. Stamping with "now" instead would fold the whole
        USB/decode latency into the pose the consumer associates with the frame.
        So measure that latency and subtract it from the current ROS time.
        """
        latency = self.dai.Clock.now() - message.getTimestamp()
        nanoseconds = max(int(latency.total_seconds() * 1e9), 0)
        return (self.get_clock().now() - Duration(nanoseconds=nanoseconds)).to_msg()

    def _image(self, frame, encoding, stamp):
        frame = np.ascontiguousarray(frame)       # step == strides[0] demands it
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
        """A camera that stops delivering looks exactly like a camera that was
        never plugged in — from the consumer's side, both are silence."""
        now = time.monotonic()
        delivered = self.frames - self.last_health_frames
        elapsed = max(now - self.last_health_time, 1e-6)
        self.last_health_frames = self.frames
        self.last_health_time = now

        if delivered == 0:
            self.get_logger().warn(
                f"no frames since last check (total={self.frames}, "
                f"incomplete={self.incomplete}) — camera stalled or unplugged")
            return

        # The publish rate AT THE SOURCE. Without this line the only number
        # anyone has is `ros2 topic hz`, which measures a Python subscriber and
        # the DDS transport as much as it measures this node — for 768 kB raw
        # frames the two disagree hard, and the gap sends people hunting for a
        # camera fault that is really a transport one.
        self.get_logger().info(
            f"{delivered / elapsed:.1f} fps published "
            f"(total={self.frames}, incomplete={self.incomplete})")

    def destroy_node(self):
        # Close the device explicitly. The OAK-D admits ONE client: a lingering
        # handle means the next start of this node fails to open the camera.
        if self.device is not None:
            try:
                self.device.close()
            except Exception as e:
                self.get_logger().warn(f"device close failed: {e}")
            self.device = None
        super().destroy_node()


def main(args=None):
    run_node(OakDPublisher, args=args)


if __name__ == "__main__":
    main()
