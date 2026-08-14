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

WHICH CONTAINER THIS RUNS IN
----------------------------
The SENSOR container, never `asv`. `asv` asserts at build time that depthai is
absent (Dockerfile), because the OAK-D admits exactly one client and perception
grabbing the device would take it away from this node. The package still builds
everywhere — `depthai` is imported inside `_open_device`, not at module scope,
so `colcon build` and an import smoke test pass in a container that has no SDK
and no camera.

Parameters come from crusader_bringup's `crusader_params.yaml` like every other
node here: one params file for the boat, whichever container a node lives in, so
a value can never drift between a code default and the launched file. The sensor
container therefore needs crusader_bringup built in its workspace or
$CRUSADER_PARAMS pointed at the file (crusader_common/config.py resolves env ->
installed share -> source tree, and the mounted repo satisfies the last one).

Manual check (from any ROS container on the boat network):
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

# The OAK-D LR sensor is 1920x1200; every ISP scale is 1/N of that. Both stereo
# cameras and the depth output are held to the SAME size as the colour frame —
# that equality is what the aligned-depth contract rests on, so it is asserted
# at runtime rather than assumed.
SENSOR_WIDTH = 1920
SENSOR_HEIGHT = 1200

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
        self.width = SENSOR_WIDTH // p["isp_denominator"]
        self.height = SENSOR_HEIGHT // p["isp_denominator"]

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
        import depthai as dai                     # sensor container only
        from datetime import timedelta

        pipeline = dai.Pipeline()

        rgb = pipeline.create(dai.node.ColorCamera)
        rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
        rgb.setIspScale(1, p["isp_denominator"])
        rgb.setInterleaved(False)
        rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        rgb.setFps(p["fps"])

        # OAK-D LR: the stereo pair are colour sensors too, so they are
        # ColorCamera nodes scaled identically to CAM_A.
        left = pipeline.create(dai.node.ColorCamera)
        left.setCamera("left")
        left.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
        left.setIspScale(1, p["isp_denominator"])
        left.setFps(p["fps"])

        right = pipeline.create(dai.node.ColorCamera)
        right.setCamera("right")
        right.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
        right.setIspScale(1, p["isp_denominator"])
        right.setFps(p["fps"])

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setLeftRightCheck(p["lr_check"])
        stereo.setSubpixel(p["subpixel"])
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)   # depth in RGB pixels
        stereo.setOutputSize(self.width, self.height)

        left.isp.link(stereo.left)
        right.isp.link(stereo.right)

        # Pair the two streams ON THE DEVICE. Anything the device cannot pair
        # within the threshold never leaves it — see the module docstring.
        sync = pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(milliseconds=p["sync_threshold_ms"]))
        rgb.isp.link(sync.inputs["rgb"])
        stereo.depth.link(sync.inputs["depth"])

        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("rgbd")
        sync.out.link(xout.input)

        self.dai = dai
        self.device = dai.Device(pipeline)
        # blocking=False: the camera must never stall waiting for this node, and
        # a backlog would mean publishing the past.
        self.queue = self.device.getOutputQueue("rgbd", maxSize=p["queue_size"],
                                                blocking=False)

        speed = self.device.getUsbSpeed()
        self.get_logger().info(
            f"OAK-D open: mxid={self.device.getMxId()}, usb={speed.name}")
        if speed not in (dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS):
            # USB2 does not carry 640x400 RGB + subpixel depth at 30fps. It will
            # "work" at a fraction of the rate, which reads as a perception bug
            # three layers downstream — so say it here, at the source.
            self.get_logger().warn(
                f"link negotiated {speed.name}, not SUPER — expect dropped "
                "frames. Check the cable and the port (blue/SS).")

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
