"""perception_node — OAK-D LR capture -> TensorRT detect -> depth associate ->
/crsd/detections_body (DetectionArray, frame="body").

Deliberately ONE process (plan §3.5 modules composed in-process): shipping 1080p
frames over DDS at 30 fps would eat the latency budget; only the small
DetectionArray leaves this node. A throttled annotated debug image is optional.

NOTE: depthai is imported only inside the _start_camera function for CI purposes
as it is only available in the asv container, installed in the Jetson. 

Startup order (all fail loudly, node exits nonzero — never degrades silently):
  1. oakd_guard.assert_usb_super()      — USB2 would silently halve throughput
  2. Detector(engine)                   — refuses on missing/unmapped classes
  3. depthai pipeline with depth ALIGNED to RGB
  4. intrinsics read from device calibration (not hardcoded)

Health: /crsd/perception_health (std_msgs/String, JSON from PipelineStats) at
1 Hz; WARN when over the 100 ms / 15 fps budget. Preflight and the G2 procedure
read this topic.

NOTE (bench TODO): camera socket mapping below uses the standard
ColorCamera/StereoDepth setup; the OAK-D LR exposes three color sensors, so
verify CAM_A/CAM_B/CAM_C socket assignment on first bench bring-up.
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from interfaces.msg import Detection, DetectionArray

from rx26_asv.api.common import config as crsd_config
from rx26_asv.api.common.node_main import run_node
from rx26_asv.api.common.param_utils import declare_from_config, make_set_callback
from rx26_asv.api.perception.depth_association import CameraModel, associate
from rx26_asv.api.perception.detector import Detector
from rx26_asv.api.perception.oakd_guard import assert_usb_super
from rx26_asv.api.perception.pipeline_stats import PipelineStats

PARAM_SPEC = {
    "engine_path": dict(read_only=True, description="per-Jetson TensorRT engine"),
    "mount_offset_x": dict(read_only=True, lo=-2.0, hi=2.0,
                           description="camera in BODY frame [m]"),
    "mount_offset_y": dict(read_only=True, lo=-2.0, hi=2.0),
    "conf_threshold": dict(read_only=False, lo=0.05, hi=0.95,
                           description="detector confidence floor"),
    "latency_budget_ms": dict(read_only=False, lo=20.0, hi=1000.0),
    "fps_floor": dict(read_only=False, lo=1.0, hi=60.0),
}
DYNAMIC_RANGES = {k: (v["lo"], v["hi"]) for k, v in PARAM_SPEC.items()
                  if not v["read_only"]}


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        p = declare_from_config(self, crsd_config.node_params("perception_node"),
                                PARAM_SPEC)

        self.det_pub = self.create_publisher(DetectionArray,
                                             "/crsd/detections_body", 10)
        self.health_pub = self.create_publisher(String,
                                                "/crsd/perception_health", 10)

        self.stats = PipelineStats(latency_budget_s=p["latency_budget_ms"] / 1000.0,
                                   fps_floor=p["fps_floor"])
        self.mount = (p["mount_offset_x"], p["mount_offset_y"])

        speed = assert_usb_super()                       # 1. fail loudly on USB2
        self.get_logger().info(f"OAK-D at {speed}")

        self.detector = Detector(                        # 2. refuses silent config
            p["engine_path"], conf=p["conf_threshold"])
        self.get_logger().info("detector loaded")

        self.add_on_set_parameters_callback(
            make_set_callback(self, DYNAMIC_RANGES, self._apply_params))

        self._device, self._q_rgb, self._q_depth, self.cam_model = self._start_camera()
        self.get_logger().info(f"intrinsics: fx={self.cam_model.fx:.1f} "
                               f"cx={self.cam_model.cx:.1f}")

        self.create_timer(0.001, self._pump)             # tight non-blocking pump
        self.create_timer(1.0, self._publish_health)

    def _apply_params(self, changes):
        """Dynamic-param application — every accepted set takes real effect."""
        if "conf_threshold" in changes:
            self.detector.conf = changes["conf_threshold"]
        if "latency_budget_ms" in changes:
            self.stats.latency_budget_s = changes["latency_budget_ms"] / 1000.0
        if "fps_floor" in changes:
            self.stats.fps_floor = changes["fps_floor"]

    # ---------- depthai ----------

    def _start_camera(self):
        import depthai as dai
        pipeline = dai.Pipeline()

        rgb = pipeline.create(dai.node.ColorCamera)
        rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        rgb.setPreviewSize(1280, 720)
        rgb.setPreviewKeepAspectRatio(True)

        mono_l = pipeline.create(dai.node.MonoCamera)
        mono_r = pipeline.create(dai.node.MonoCamera)
        mono_l.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_r.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)   # 3. depth aligned to RGB
        stereo.setOutputSize(1280, 720)
        mono_l.out.link(stereo.left)
        mono_r.out.link(stereo.right)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        rgb.preview.link(xout_rgb.input)
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        device = dai.Device(pipeline)
        q_rgb = device.getOutputQueue("rgb", maxSize=2, blocking=False)
        q_depth = device.getOutputQueue("depth", maxSize=2, blocking=False)

        # 4. intrinsics from device calibration, at the preview resolution
        calib = device.readCalibration()
        M = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 1280, 720)
        cam = CameraModel(fx=M[0][0], fy=M[1][1], cx=M[0][2], cy=M[1][2])
        return device, q_rgb, q_depth, cam

    # ---------- main loop ----------

    def _pump(self):
        frame_msg = self._q_rgb.tryGet()
        depth_msg = self._q_depth.tryGet()
        if frame_msg is None or depth_msg is None:
            return
        capture_t = time.monotonic()   # host-side; device timestamps refine later
        frame = frame_msg.getCvFrame()
        depth = depth_msg.getFrame()

        boxes = [(b.label, b.confidence, b.bbox)
                 for b in self.detector.infer(frame)]
        dets = associate(boxes, depth, self.cam_model, self.mount)

        out = DetectionArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.frame = "body"
        for d in dets:
            m = Detection()
            m.label = d.label
            m.x, m.y = float(d.x), float(d.y)
            m.radius = float(d.radius)
            m.confidence = d.confidence
            m.source = Detection.SOURCE_PERCEPTION
            out.detections.append(m)
        self.det_pub.publish(out)
        self.stats.record_frame(capture_t, time.monotonic(),
                                dropped_no_depth=len(boxes) - len(dets))

    def _publish_health(self):
        snap = self.stats.snapshot()
        self.health_pub.publish(String(data=json.dumps(snap)))
        if self.stats.frames and not snap["healthy"]:
            self.get_logger().warn(f"perception over budget: {snap}")

    def destroy_node(self):
        try:
            self._device.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    run_node(PerceptionNode, args=args)


if __name__ == "__main__":
    main()
