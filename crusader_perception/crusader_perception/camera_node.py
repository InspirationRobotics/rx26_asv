"""camera_node: the OAK-D's one owner and the camera's one picture.

  source  oak    the device (depthai v2): colour and depth aligned and paired on-device
          topic  oak/rgb (bgr8) + oak/depth (16UC1 mm), e.g. the sim's sim_camera
  model   none | buoy | dock, a dynamic parameter. Loaded and unloaded on a worker thread;
          frames, pictures and health keep flowing while it loads.
            buoy -> /crsd/oak/detections  Detection3DArray, every frame (buoy_model)
            dock -> /dock/observations    DockObservation, every frame (dock_model)
  out     /crsd/camera/image/compressed  the annotated picture, only while subscribed
          /crsd/camera/raw/compressed    the frame the model saw, full size, only while
                                         subscribed: what a recording wants (frames with
                                         boxes burned in would train on the boxes)
          /crsd/camera/health            JSON every health_period_s

The 17 camera controls (oak_controls) are dynamic parameters, sent to the running device.
depthai, ultralytics and cv2 are imported inside functions, so this module imports on a
machine with none of them.

NOT in core.launch.py: start it from the ground station's Nodes tab.
"""
import json
import math
import threading
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from crusader_common.node_main import declare_params, on_param_set, run_node
from crusader_perception import (buoy_model, dock_model, oak_controls, oak_detector_core,
                                 oak_pipeline)

PARAMS = {
    "source": "oak",                    # oak | topic
    "model": "buoy",                    # none | buoy | dock
    "fps": 15.0,
    "isp_denominator": 3,               # stereo pair and depth, 1/N of 1920x1200
    "rgb_isp_denominator": 1,           # the colour camera alone: more pixels on a buoy
                                        # without paying for depth at that size
    "queue_size": 4,
    "topic_hfov_deg": 82.0,             # source topic: the colour camera's field of view
    "image_max_width": 960,
    "image_quality": 70,
    "image_max_hz": 5.0,
    "health_period_s": 5.0,
    **oak_detector_core.PARAMS,
    **dock_model.PARAMS,
    **oak_controls.defaults(),
}
RANGES = {
    "image_max_width": (160, 1920), "image_quality": (10, 100), "image_max_hz": (0.5, 30.0),
    **oak_controls.ranges(),
}
DYNAMIC = ("model", "image_max_width", "image_quality", "image_max_hz",
           *oak_controls.defaults())
MODELS = {"buoy": buoy_model.BuoyModel, "dock": dock_model.DockModel}

# How long teardown waits for depthai to let go of the camera: under the ground station's
# five-second grace before SIGKILL, so the node gets to say what happened.
DEVICE_CLOSE_TIMEOUT_S = 3.0
PAIR_TOLERANCE_S = 0.1                  # source topic: depth this close to the colour frame
IMAGE_FRAME = "oak_rgb_camera_optical_frame"


@dataclass
class Frame:
    bgr: np.ndarray
    depth: np.ndarray                   # uint16 mm aligned to bgr's field of view, or None
    stamp: object                       # builtin_interfaces/Time: when the light arrived
    t: float                            # seconds on the camera's clock, for timing layers
    intrinsics: tuple                   # (fx, fy, cx, cy) at bgr's size


class OakSource:
    """The device. One pipeline (oak_pipeline), the camera profile applied at build and then
    through the control queue on every change."""

    def __init__(self, node, p, profile):
        import depthai as dai
        if not hasattr(dai.node, "XLinkOut"):
            raise RuntimeError(f"depthai {dai.__version__} is the v3 API (no XLinkOut); "
                               "oak_pipeline targets v2: pin depthai==2.x in the Dockerfile")
        self.dai, self.node, self.p = dai, node, p
        pipeline, self.width, self.height = oak_pipeline.build_rgbd(
            isp_denominator=p["isp_denominator"], fps=p["fps"], controls=profile,
            rgb_isp_denominator=p["rgb_isp_denominator"])
        self.device = dai.Device(pipeline)
        self.queue = self.device.getOutputQueue("rgbd", p["queue_size"], blocking=False)
        # Depth 1, non-blocking: only the newest control matters while a slider is dragged.
        self.control = self.device.getInputQueue(oak_pipeline.CONTROL_STREAM, maxSize=1,
                                                 blocking=False)
        self.intrinsics = oak_pipeline.rgb_intrinsics(self.device, self.width, self.height)
        self.measured = {}
        self.usb = self.device.getUsbSpeed().name
        log = node.get_logger()
        log.info(f"OAK-D open: mxid={self.device.getMxId()}, usb={self.usb}, "
                 f"{self.width}x{self.height}")
        warning = oak_pipeline.usb_warning(self.device)
        if warning:
            log.warn(warning)

    def read(self):
        try:
            group = self.queue.tryGet()
        except RuntimeError as e:
            # X_LINK_ERROR: the USB link fell over, almost always on bandwidth.
            depth_w, depth_h = oak_pipeline.output_size(self.p["isp_denominator"])
            mb_s = (self.width * self.height * 1.5 + depth_w * depth_h * 2) * self.p["fps"] / 1e6
            raise RuntimeError(f"{e} | the camera link dropped carrying {self.width}x"
                               f"{self.height} colour plus depth at {self.p['fps']:g} fps, about "
                               f"{mb_s:.0f} MB/s: raise rgb_isp_denominator or lower fps") from e
        if group is None:
            return None
        msgs = dict(group)
        rgb, depth = msgs.get("rgb"), msgs.get("depth")
        if rgb is None or depth is None:
            return None
        try:                            # what the camera did, not what it was asked
            self.measured = {"exposure_us": round(rgb.getExposureTime().total_seconds() * 1e6),
                             "iso": rgb.getSensitivity(), "colour_k": rgb.getColorTemperature()}
        except Exception:  # noqa: BLE001 - a missing readout costs the readout, not the frame
            pass
        bgr = rgb.getCvFrame()
        if bgr.shape[1] != self.width:
            # The depth ratio would be wrong and every range plausible but false.
            self.node.get_logger().error(f"colour frame is {bgr.shape[1]} px wide, not "
                                         f"{self.width}: dropped", throttle_duration_sec=5.0)
            return None
        # Stamped when the light arrived, not when inference finished: fusion keys on it.
        latency = self.dai.Clock.now() - rgb.getTimestamp()
        stamp = (self.node.get_clock().now()
                 - Duration(nanoseconds=max(int(latency.total_seconds() * 1e9), 0))).to_msg()
        return Frame(bgr, depth.getFrame(), stamp, rgb.getTimestamp().total_seconds(),
                     self.intrinsics)

    def send(self, profile):
        """Push the whole profile to the running camera; raise when depthai refuses."""
        control = self.dai.CameraControl()
        refused = oak_controls.apply(self.dai, control, profile)
        if refused:
            raise RuntimeError("; ".join(refused))
        self.control.send(control)

    def close(self):
        """Let go of the camera, or give up loudly: a device that has streamed for hours can
        sit in an uninterruptible USB wait here, and a node that will not die costs more than
        a leaked handle (the next start may need the cable replugged)."""
        device, self.device = self.device, None
        if device is None:
            return
        done = threading.Event()

        def _close():
            try:
                device.close()
            except Exception as e:  # noqa: BLE001
                self.node.get_logger().warn(f"device close failed: {e}")
            finally:
                done.set()

        threading.Thread(target=_close, daemon=True).start()
        if not done.wait(DEVICE_CLOSE_TIMEOUT_S):
            self.node.get_logger().error(
                f"OAK-D did not close within {DEVICE_CLOSE_TIMEOUT_S:.0f}s: abandoning it. If "
                "the next start cannot open it, replug the cable")


class TopicSource:
    """oak/rgb + oak/depth from another node, paired by stamp. Camera controls are accepted
    and not applied: there is no device to send them to."""

    usb = None
    measured = {}

    def __init__(self, node, p):
        self.node = node
        self.hfov = math.radians(p["topic_hfov_deg"])
        self._cv = threading.Condition()
        self._depth = None              # (t, array)
        self._next = None
        node.create_subscription(Image, "oak/rgb", self._on_rgb, qos_profile_sensor_data)
        node.create_subscription(Image, "oak/depth", self._on_depth, qos_profile_sensor_data)

    @staticmethod
    def _t(stamp):
        return stamp.sec + 1e-9 * stamp.nanosec

    def _on_depth(self, msg):
        if msg.encoding != "16UC1":
            self.node.get_logger().error(f"oak/depth is {msg.encoding}, not 16UC1 mm",
                                         throttle_duration_sec=10.0)
            return
        a = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.width)
        with self._cv:
            self._depth = (self._t(msg.header.stamp), a)

    def _on_rgb(self, msg):
        if msg.encoding != "bgr8":
            self.node.get_logger().error(f"oak/rgb is {msg.encoding}, not bgr8",
                                         throttle_duration_sec=10.0)
            return
        bgr = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
        t = self._t(msg.header.stamp)
        w, h = msg.width, msg.height
        f = (w / 2.0) / math.tan(self.hfov / 2.0)
        with self._cv:
            depth = self._depth
            depth = depth[1] if depth and abs(depth[0] - t) <= PAIR_TOLERANCE_S else None
            self._next = Frame(bgr, depth, msg.header.stamp, t, (f, f, w / 2.0, h / 2.0))
            self._cv.notify()

    def read(self):
        with self._cv:
            if self._next is None:
                self._cv.wait(0.1)
            frame, self._next = self._next, None
        return frame

    def send(self, profile):
        pass

    def close(self):
        pass


class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")
        self.p = declare_params(self, PARAMS, dynamic=DYNAMIC, ranges=RANGES)
        if self.p["source"] not in ("oak", "topic"):
            raise ValueError(f"source must be oak or topic, not {self.p['source']!r}")
        self._check_model(self.p["model"])
        self.pubs = {name: self.create_publisher(m.msg_type, m.topic, qos_profile_sensor_data
                                                 if name == "buoy" else 10)
                     for name, m in MODELS.items()}
        self.view_pub = self.create_publisher(CompressedImage, "/crsd/camera/image/compressed", 2)
        self.raw_pub = self.create_publisher(CompressedImage, "/crsd/camera/raw/compressed", 2)
        self.health_pub = self.create_publisher(String, "/crsd/camera/health", 10)

        self._model = None
        self._wanted = None
        self.model_state = "none"
        self._run_lock = threading.Lock()   # held while a frame runs through the model
        self._load_lock = threading.Lock()  # one load at a time
        self._fatal = None
        self._stop = threading.Event()
        self.frames = self.model_errors = 0
        self.fps, self._last_frames, self._last_t = 0.0, 0, time.monotonic()
        self._image_t = 0.0

        profile = self._profile()
        self.source = (OakSource(self, self.p, profile) if self.p["source"] == "oak"
                       else TopicSource(self, self.p))
        self.get_logger().info(f"camera: {oak_controls.summary(profile)}")
        on_param_set(self, self._apply)
        self._set_model(self.p["model"])
        self._thread = threading.Thread(target=self._frames, daemon=True)
        self._thread.start()
        self.create_timer(0.5, self._check_fatal)
        self.create_timer(self.p["health_period_s"], self._health)

    # ------------------------------------------------------------------ parameters

    def _profile(self, values=None):
        values = self.p if values is None else values
        return {c.name: values[c.name] for c in oak_controls.ALL}

    @staticmethod
    def _check_model(name):
        if name != "none" and name not in MODELS:
            raise ValueError(f"model must be one of none, {', '.join(MODELS)}; not {name!r}")

    def _apply(self, changes):
        """Validated before self.p moves, and the camera profile committed only once the
        device has taken it, so the parameters never show a value in force nowhere."""
        if "model" in changes:
            self._check_model(changes["model"])
        error = oak_controls.validate(changes)
        if error:
            raise ValueError(error)
        controls = [k for k in changes if k in oak_controls.BY_NAME]
        if controls:
            profile = self._profile(dict(self.p, **changes))
            self.source.send(profile)       # every setting, so the camera is the profile
            self.get_logger().info(f"camera: {oak_controls.summary(profile)}")
        self.p.update(changes)
        if "model" in changes:
            self._set_model(changes["model"])

    def _set_model(self, name):
        self._wanted = name
        threading.Thread(target=self._load, args=(name,), daemon=True).start()

    def _load(self, name):
        with self._load_lock:
            if self._wanted != name:
                return                      # superseded before it started
            with self._run_lock:
                old, self._model = self._model, None
                self.model_state = "none" if name == "none" else f"loading {name}"
            if old is not None:
                old.close()
                self.get_logger().info(f"{old.name} model unloaded")
                del old
                free_gpu()
            if name == "none":
                return
            try:
                model = MODELS[name](dict(self.p), self.get_logger())
            except Exception as e:  # noqa: BLE001
                self.model_state = f"{name} failed: {e}"
                self.get_logger().error(f"{name} model did not load ({e}): no detections")
                return
            if self._wanted != name:
                model.close()
                return
            with self._run_lock:
                self._model = model
            self.model_state = f"{name} ready"

    # ------------------------------------------------------------------ frames

    def _frames(self):
        while not self._stop.is_set() and rclpy.ok():
            try:
                frame = self.source.read()
            except Exception as e:  # noqa: BLE001
                self._fatal = e
                return
            if frame is None:
                if isinstance(self.source, OakSource):
                    self._stop.wait(0.005)
                continue
            self.frames += 1
            with self._run_lock:
                self._run(frame)

    def _run(self, frame):
        """One frame through the model and onto the pictures. Under _run_lock, and a method
        so no reference to the model outlives the frame (an unloaded model must be free)."""
        model = self._model
        if model is not None:
            try:
                self.pubs[model.name].publish(model.run(frame))
            except Exception as e:  # noqa: BLE001 - one bad frame must not stop the camera
                self.model_errors += 1
                self.get_logger().error(f"{model.name} model: {type(e).__name__}: {e}",
                                        throttle_duration_sec=5.0)
                model = None
        try:
            self._publish_images(frame, model)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"picture not published: {e}", throttle_duration_sec=10.0)

    def _publish_images(self, frame, model):
        want_view = self.view_pub.get_subscription_count() > 0
        want_raw = self.raw_pub.get_subscription_count() > 0
        now = time.monotonic()
        if not (want_view or want_raw) or now - self._image_t < 1.0 / self.p["image_max_hz"]:
            return
        self._image_t = now
        import cv2
        quality = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.p["image_quality"])]
        if want_raw:
            self.raw_pub.publish(self._jpeg(cv2, frame.bgr, frame.stamp, quality))
        if not want_view:
            return
        image = frame.bgr.copy()            # a viewer never alters what the model saw
        lines = [f"{self.p['source']} {image.shape[1]}x{image.shape[0]} {self.fps:.1f} fps  "
                 f"model: {self.model_state}"]
        if model is not None:
            lines.append(model.annotate(image))
        if isinstance(self.source, OakSource):
            m = self.source.measured
            lines.append(f"measured exp {m.get('exposure_us', 0)}us ISO {m.get('iso', 0)} wb "
                         f"{m.get('colour_k', 0)}K | set {oak_controls.summary(self.p)}")
        width = int(self.p["image_max_width"])
        if image.shape[1] > width:
            image = cv2.resize(image, (width, round(image.shape[0] * width / image.shape[1])),
                               interpolation=cv2.INTER_AREA)
        y = 16
        for line in lines:
            cv2.putText(image, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3,
                        cv2.LINE_AA)
            cv2.putText(image, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255),
                        1, cv2.LINE_AA)
            y += 16
        self.view_pub.publish(self._jpeg(cv2, image, frame.stamp, quality))

    @staticmethod
    def _jpeg(cv2, image, stamp, quality):
        ok, buf = cv2.imencode(".jpg", image, quality)
        if not ok:
            raise RuntimeError("JPEG encode failed")
        m = CompressedImage()
        m.header.stamp = stamp
        m.header.frame_id = IMAGE_FRAME
        m.format = "jpeg"
        m.data = buf.tobytes()
        return m

    def _check_fatal(self):
        if self._fatal is not None:
            raise RuntimeError(f"camera source failed: {self._fatal}") from self._fatal

    # ------------------------------------------------------------------ health

    def _health(self):
        now = time.monotonic()
        delivered = self.frames - self._last_frames
        self.fps = delivered / max(now - self._last_t, 1e-6)
        self._last_frames, self._last_t = self.frames, now
        model = self._model
        payload = {
            "source": self.p["source"], "model": self.p["model"], "model_state": self.model_state,
            "fps": round(self.fps, 2), "frames": self.frames, "model_errors": self.model_errors,
            "usb": self.source.usb, "measured": self.source.measured,
            "set": oak_controls.summary(self.p),
            "viewers": {"image": self.view_pub.get_subscription_count(),
                        "raw": self.raw_pub.get_subscription_count()},
            "model_health": model.health() if model is not None else None,
        }
        self.health_pub.publish(String(data=json.dumps(payload)))
        log = self.get_logger()
        if delivered == 0:
            log.warn(f"no frames for {self.p['health_period_s']:g}s (total {self.frames}): "
                     "camera stalled, unplugged, or nothing on oak/rgb")
            return
        for text in model.warnings(self.fps) if model is not None else ():
            log.warn(text, throttle_duration_sec=30.0)
        log.info(f"{self.fps:.1f} fps, model {self.model_state}, "
                 f"{oak_controls.summary(self.p)}")

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        with self._run_lock:
            model, self._model = self._model, None
        if model is not None:
            model.close()
        self.source.close()
        super().destroy_node()


def free_gpu():
    """Give an unloaded model's GPU memory back before the next one loads."""
    import gc
    gc.collect()
    try:
        import torch
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    except ImportError:
        pass


def main(args=None):
    run_node(CameraNode, args=args)


if __name__ == "__main__":
    main()
