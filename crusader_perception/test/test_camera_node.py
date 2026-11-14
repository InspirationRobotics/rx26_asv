"""camera_node with source topic and stand-in models: no depthai, no ultralytics, no camera.
Needs rclpy, a built crusader_msgs and cv2; skips itself without them."""
import json
import os
import tempfile
import time
import unittest

import numpy as np

try:
    import cv2
    import rclpy
    from rclpy.parameter import Parameter
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import Image
    from crusader_msgs.msg import Detection3D, Detection3DArray, DockObservation
    from crusader_perception import camera_node as cn
    HAVE_ROS = True
except Exception:                                   # no ROS, msgs not built, or no cv2
    HAVE_ROS = False


class _Rec:
    def __init__(self, watchers=1):
        self.msgs = []
        self.watchers = watchers

    def publish(self, m):
        self.msgs.append(m)

    def get_subscription_count(self):
        return self.watchers


class FakeBuoy:
    name = "buoy"
    topic = "/crsd/oak/detections"
    msg_type = None                     # Detection3DArray, set once ROS has imported
    loads = 0

    def __init__(self, p, log):
        FakeBuoy.loads += 1
        self.closed = False
        self.seen = []

    def run(self, frame):
        self.seen.append(frame)
        a = Detection3DArray()
        a.header.stamp = frame.stamp
        a.detections.append(Detection3D(label="red_circle", x=float(frame.depth is not None)))
        return a

    def annotate(self, image):
        image[:10, :10] = 255
        return "fake model"

    def health(self):
        return {"fake": True}

    def warnings(self, fps):
        return []

    def close(self):
        self.closed = True


class FakeDock(FakeBuoy):
    """The dock model's live settings: what camera_node pushes onto it, and when."""
    name = "dock"
    topic = "/dock/observations"
    msg_type = None                     # DockObservation, set once ROS has imported
    loads = 0

    def __init__(self, p, log):
        FakeDock.loads += 1
        self.closed = False
        self.seen = []
        self.live = []                  # every set_params: (dock_indicator, dock_cv_sat_min)
        self.weights = p["dock_weights"]  # the detector this load was asked for
        self.set_params(p)

    def set_params(self, p):
        self.live.append((p["dock_indicator"], p["dock_cv_sat_min"]))

    def run(self, frame):
        self.seen.append(frame)
        return DockObservation()


def image(w, h, sec, encoding="bgr8"):
    m = Image()
    m.header.stamp = Time(sec=sec)
    m.width, m.height, m.encoding = w, h, encoding
    if encoding == "16UC1":
        m.step = 2 * w
        m.data = np.full((h, w), 5000, np.uint16).tobytes()
    else:
        m.step = 3 * w
        m.data = np.zeros((h, w, 3), np.uint8).tobytes()
    return m


def wait(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


@unittest.skipUnless(HAVE_ROS, "needs rclpy, a built crusader_msgs and cv2")
class CameraNodeTopicSource(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=["--ros-args", "-p", "source:=topic", "-p", "model:=none",
                         "-p", "image_max_hz:=30.0"])
        cls.models = dict(cn.MODELS)
        FakeBuoy.msg_type = Detection3DArray
        FakeDock.msg_type = DockObservation
        cn.MODELS["buoy"] = FakeBuoy
        cn.MODELS["dock"] = FakeDock

    @classmethod
    def tearDownClass(cls):
        cn.MODELS.clear()
        cn.MODELS.update(cls.models)
        rclpy.try_shutdown()

    def setUp(self):
        self.node = cn.CameraNode()
        self.view, self.raw, self.det = _Rec(), _Rec(0), _Rec()
        self.node.view_pub, self.node.raw_pub = self.view, self.raw
        self.node.pubs["buoy"] = self.det
        self.node.pubs["dock"] = _Rec()

    def tearDown(self):
        self.node.destroy_node()

    def setp(self, name, kind, value):
        return self.node.set_parameters([Parameter(name, kind, value)])[0]

    def feed(self, sec=1, depth_sec=1, w=1280, h=800):
        n = self.node.frames
        self.node.source._on_depth(image(w // 2, h // 2, depth_sec, "16UC1"))
        self.node.source._on_rgb(image(w, h, sec))
        self.assertTrue(wait(lambda: self.node.frames > n))

    def test_picture_only_for_watchers_and_scaled(self):
        self.feed()
        self.assertTrue(wait(lambda: len(self.view.msgs) == 1))
        self.assertEqual(self.raw.msgs, [])          # nobody watching the raw frame
        m = self.view.msgs[0]
        self.assertEqual(m.format, "jpeg")
        self.assertEqual(m.header.stamp.sec, 1)      # the frame's stamp, not the encode time
        shown = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(shown.shape[:2], (600, 960))    # image_max_width
        self.raw.watchers = 1
        time.sleep(0.05)
        self.feed(sec=2, depth_sec=2)
        self.assertTrue(wait(lambda: len(self.raw.msgs) == 1))
        full = cv2.imdecode(np.frombuffer(bytes(self.raw.msgs[0].data), np.uint8),
                            cv2.IMREAD_COLOR)
        self.assertEqual(full.shape[:2], (800, 1280))    # the raw frame is never scaled

    def test_model_loads_runs_and_unloads(self):
        self.assertTrue(self.setp("model", Parameter.Type.STRING, "buoy").successful)
        self.assertTrue(wait(lambda: self.node.model_state == "buoy ready"))
        model = self.node._model
        self.feed()
        self.assertTrue(wait(lambda: len(self.det.msgs) == 1))
        self.assertEqual(self.det.msgs[0].detections[0].x, 1.0)     # depth was paired
        self.feed(sec=3, depth_sec=1)                                # depth 2 s old
        self.assertTrue(wait(lambda: len(self.det.msgs) == 2))
        self.assertEqual(self.det.msgs[1].detections[0].x, 0.0)     # not paired
        fx, fy, cx, cy = model.seen[0].intrinsics
        self.assertAlmostEqual(fx, 640.0 / np.tan(np.radians(41.0)), places=3)
        self.assertEqual((cx, cy), (640.0, 400.0))
        self.assertTrue(self.setp("model", Parameter.Type.STRING, "none").successful)
        self.assertTrue(wait(lambda: model.closed and self.node._model is None))
        self.assertEqual(self.node.model_state, "none")

    def test_parameters_are_checked(self):
        self.assertFalse(self.setp("model", Parameter.Type.STRING, "boat").successful)
        self.assertTrue(self.setp("ae_compensation", Parameter.Type.INTEGER, 2).successful)
        self.assertFalse(self.setp("ae_compensation", Parameter.Type.INTEGER, 20).successful)
        self.assertFalse(self.setp("awb_mode", Parameter.Type.STRING, "sunny").successful)
        self.assertEqual(self.node.p["awb_mode"], "auto")
        self.assertFalse(self.setp("fps", Parameter.Type.DOUBLE, 30.0).successful)  # read-only

    def test_dock_indicator_is_live(self):
        """dock_indicator and the dock_cv_* settings reach the running dock model between
        two frames, with no reload; a bad one is refused whole; a later load starts on them."""
        self.assertTrue(self.setp("model", Parameter.Type.STRING, "dock").successful)
        self.assertTrue(wait(lambda: self.node.model_state == "dock ready"))
        model, loads = self.node._model, FakeDock.loads
        self.assertEqual(model.live[-1], ("yolo", 0.40))            # the defaults, at load
        self.assertTrue(self.setp("dock_indicator", Parameter.Type.STRING, "cv").successful)
        self.assertEqual(model.live[-1], ("cv", 0.40))
        self.assertTrue(self.setp("dock_cv_sat_min", Parameter.Type.DOUBLE, 0.3).successful)
        self.assertEqual(model.live[-1], ("cv", 0.3))
        self.assertIs(self.node._model, model)                       # the same model: no reload
        self.assertEqual(FakeDock.loads, loads)
        self.feed()
        self.assertTrue(wait(lambda: len(model.seen) == 1))          # and it still runs
        n = len(model.live)
        for name, kind, value in (
                ("dock_indicator", Parameter.Type.STRING, "hsv"),
                ("dock_cv_sat_min", Parameter.Type.DOUBLE, 1.5),
                ("dock_cv_ratio", Parameter.Type.DOUBLE, 0.5),
                ("dock_cv_roi", Parameter.Type.DOUBLE_ARRAY, [0.1, 0.9, 0.5]),
                ("dock_cv_red_hue", Parameter.Type.DOUBLE_ARRAY, [340.0, 400.0])):
            self.assertFalse(self.setp(name, kind, value).successful, name)
        self.assertEqual(len(model.live), n)                         # nothing pushed
        self.assertEqual((self.node.p["dock_indicator"], self.node.p["dock_cv_sat_min"]),
                         ("cv", 0.3))
        self.assertTrue(self.setp("model", Parameter.Type.STRING, "none").successful)
        self.assertTrue(wait(lambda: model.closed and self.node._model is None))
        self.assertTrue(self.setp("dock_indicator", Parameter.Type.STRING, "yolo").successful)
        self.assertTrue(self.setp("model", Parameter.Type.STRING, "dock").successful)
        self.assertTrue(wait(lambda: self.node.model_state == "dock ready"))
        self.assertEqual(self.node._model.live[-1], ("yolo", 0.3))   # the settings at load

    def test_dock_weights_reload(self):
        """dock_weights picks the detector: a change reloads the dock model once, with the new
        weights; the same value does not; a bad path is refused whole; off the dock model it is
        only kept, and the next dock load starts on it. Waits after an unload are long: the
        first free_gpu() imports torch, seconds on the Jetson."""
        with tempfile.TemporaryDirectory() as d:
            old = os.path.join(d, "old.pt")
            open(old, "wb").close()
            self.assertTrue(self.setp("model", Parameter.Type.STRING, "dock").successful)
            self.assertTrue(wait(lambda: self.node.model_state == "dock ready", timeout=30.0))
            first = self.node._model
            self.assertEqual(first.weights, "")                   # the new one by default
            self.assertTrue(self.setp("dock_weights", Parameter.Type.STRING, old).successful)
            self.assertTrue(wait(lambda: self.node._model is not None
                                 and self.node._model is not first
                                 and self.node.model_state == "dock ready", timeout=30.0))
            self.assertEqual(self.node._model.weights, old)
            self.assertTrue(first.closed)
            second = self.node._model
            self.assertTrue(self.setp("dock_weights", Parameter.Type.STRING, old).successful)
            time.sleep(0.2)
            self.assertIs(self.node._model, second)               # same value: no reload
            self.assertEqual(self.node.model_state, "dock ready")
            for bad in (os.path.join(d, "missing.pt"), os.path.join(d, "old.onnx")):
                self.assertFalse(self.setp("dock_weights", Parameter.Type.STRING, bad).successful)
            self.assertEqual(self.node.p["dock_weights"], old)
            self.assertIs(self.node._model, second)
            self.assertTrue(self.setp("model", Parameter.Type.STRING, "buoy").successful)
            self.assertTrue(wait(lambda: self.node.model_state == "buoy ready", timeout=30.0))
            buoy = self.node._model
            self.assertTrue(self.setp("dock_weights", Parameter.Type.STRING, "").successful)
            time.sleep(0.2)
            self.assertIs(self.node._model, buoy)                 # off the dock model: kept only
            self.assertEqual(self.node.model_state, "buoy ready")
            self.assertTrue(self.setp("model", Parameter.Type.STRING, "dock").successful)
            self.assertTrue(wait(lambda: self.node.model_state == "dock ready", timeout=30.0))
            self.assertEqual(self.node._model.weights, "")        # the load starts on it

    def test_weights_path(self):
        from crusader_perception import dock_model as dm
        p = {"dock_ffcv_dir": "/m/ffcv", "dock_weights": ""}
        self.assertEqual(dm.weights_path(p), "/m/ffcv/best.pt")
        with tempfile.TemporaryDirectory() as d:
            pt = os.path.join(d, "a.pt")
            open(pt, "wb").close()
            self.assertEqual(dm.weights_path(dict(p, dock_weights=pt)), pt)
            with self.assertRaises(ValueError):
                dm.weights_path(dict(p, dock_weights=os.path.join(d, "b.pt")))
            with self.assertRaises(ValueError):
                dm.weights_path(dict(p, dock_weights=os.path.join(d, "a.engine")))

    def test_health(self):
        rec = _Rec()
        self.node.health_pub = rec
        self.feed()
        self.node._health()
        h = json.loads(rec.msgs[0].data)
        self.assertEqual((h["source"], h["model"], h["model_state"]), ("topic", "none", "none"))
        self.assertGreaterEqual(h["frames"], 1)

    def test_wrong_encoding_is_dropped(self):
        n = self.node.frames
        self.node.source._on_rgb(image(64, 40, 1, "rgb8"))
        time.sleep(0.2)
        self.assertEqual(self.node.frames, n)


if __name__ == "__main__":
    unittest.main()
