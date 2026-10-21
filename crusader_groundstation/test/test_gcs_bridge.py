"""gcs_bridge in-process, with a laptop speaking gcs_wire over a real socket. Needs rclpy and a
built workspace (crusader_msgs, crusader_bt for missions_list); skips itself without them.

    ROS_DOMAIN_ID=57 python3 crusader_groundstation/test/test_gcs_bridge.py
"""
import itertools
import json
import os
import socket
import sys
import tempfile
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, ".."))
try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String
    from crusader_msgs.msg import LatLonHead, PassagePlan
    from crusader_groundstation import gcs_wire
    from crusader_groundstation.gcs_bridge import MANUAL_VERSION_BASE, GcsBridge
    HAVE_ROS = True
except Exception:                                   # no ROS, or the msgs are not built
    HAVE_ROS = False


@unittest.skipUnless(HAVE_ROS, "needs rclpy and a built workspace")
class Bridge(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        yaml_path = os.path.join(REPO, "crusader_bringup", "config", "crusader_params.yaml")
        rclpy.init(args=["--ros-args", "-p", "port:=0", "-p", f"record_dir:={cls.tmp}",
                         "-p", f"params_file:={yaml_path}"])
        cls.node = GcsBridge()
        cls.probe = rclpy.create_node("gcs_bridge_probe")
        cls.plans = []
        cls.probe.create_subscription(PassagePlan, "/crsd/passage_plan", cls.plans.append, 10)
        cls.pose = cls.probe.create_publisher(LatLonHead, "/crsd/pose", 10)
        cls.bt = cls.probe.create_publisher(String, "/crsd/bt_status", 10)
        cls.ex = MultiThreadedExecutor()
        cls.ex.add_node(cls.node)
        cls.ex.add_node(cls.probe)
        threading.Thread(target=cls.ex.spin, daemon=True).start()
        port = cls.node._server.getsockname()[1]
        cls.sock = socket.create_connection(("127.0.0.1", port), timeout=10)
        cls.frames, cls.ids = [], itertools.count(1)
        threading.Thread(target=cls._read, daemon=True).start()

    @classmethod
    def _read(cls):
        try:
            while True:
                cls.frames.append(gcs_wire.read_frame(cls.sock))
        except (ConnectionError, OSError):
            pass

    @classmethod
    def tearDownClass(cls):
        cls.sock.close()
        cls.ex.shutdown()
        cls.node.destroy_node()
        cls.probe.destroy_node()
        rclpy.try_shutdown()

    def wait_frame(self, pred, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for h, b in reversed(list(self.frames)):
                if pred(h):
                    return h, b
            time.sleep(0.05)
        self.fail("no matching frame arrived")

    def req(self, op, timeout=8.0, **fields):
        rid = next(self.ids)
        self.sock.sendall(gcs_wire.encode(dict(fields, t="req", id=rid, op=op)))
        return self.wait_frame(lambda h: h.get("id") == rid, timeout)

    def test_hello_and_state(self):
        hello, _ = self.wait_frame(lambda h: h["t"] == "hello")
        self.assertIn("run_mission", hello["ops"])
        state, _ = self.wait_frame(lambda h: h["t"] == "state")
        self.assertIsNone(state["fcu"]["armed"])          # unknown, not False
        self.assertTrue(state["power"]["locked"])          # unknown arm state locks power

    def test_pose_reaches_state(self):
        m = LatLonHead(latitude=21.3, longitude=-157.9, heading=45.0, ground_speed=1.5)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            self.pose.publish(m)
            h = next((h for h, _ in reversed(self.frames) if h["t"] == "state"), None)
            if h and h["boat"]["lat"] == 21.3:
                break
            time.sleep(0.1)
        self.assertEqual((h["boat"]["lat"], h["boat"]["hdg"]), (21.3, 45.0))

    def test_protected_node_cannot_be_stopped(self):
        h, _ = self.req("node", action="stop", name="telemetry_bridge")
        self.assertFalse(h["ok"])
        self.assertIn("safety stack", h["msg"])
        h, _ = self.req("node", action="restart", name="rc_watchdog")
        self.assertFalse(h["ok"])
        h, _ = self.req("node", action="start", name="no_such_node")
        self.assertFalse(h["ok"])

    def test_params_of_itself(self):
        deadline = time.monotonic() + 5
        while "/gcs_bridge" not in self.node._graph_full and time.monotonic() < deadline:
            time.sleep(0.1)
        h, _ = self.req("params_list", node="/gcs_bridge")
        self.assertTrue(h["ok"], h["msg"])
        port = next(p for p in h["data"]["params"] if p["name"] == "port")
        self.assertTrue(port["read_only"])
        self.assertEqual(port["value"], 0)
        h, _ = self.req("params_set", node="/gcs_bridge", values={"port": 1})
        self.assertFalse(h["ok"])                          # read-only, refused by the node

    def test_manual_field(self):
        plan = {"buoys": [{"lat": 21.30010, "lon": -157.9, "beacon": 4},
                          {"lat": 21.30020, "lon": -157.9, "beacon": 2}],
                "entry": [21.30010, -157.9]}
        h, _ = self.req("manual_field", plan=plan)
        self.assertTrue(h["ok"], h["msg"])
        deadline = time.monotonic() + 3
        while not self.plans and time.monotonic() < deadline:
            time.sleep(0.05)
        m = self.plans[-1]
        self.assertGreater(m.plan_version, MANUAL_VERSION_BASE)
        self.assertEqual([b.beacon for b in m.buoys], [4, 2])
        self.assertAlmostEqual(m.entry_latitude, 21.3001)
        f, _ = self.wait_frame(lambda h: h["t"] == "plan" and h.get("source") == "manual")
        self.assertEqual(len(f["buoys"]), 2)
        h, _ = self.req("manual_field", plan={"buoys": [{"lat": 99, "lon": 0}]})
        self.assertFalse(h["ok"])
        h, _ = self.req("manual_field", clear=True)
        self.assertTrue(h["ok"])
        time.sleep(0.3)
        self.assertEqual(len(self.plans[-1].buoys), 0)

    def test_power_locked_without_arm_state(self):
        h, _ = self.req("power", verb="reboot", hostname=socket.gethostname())
        self.assertFalse(h["ok"])

    def test_mission_without_a_runner(self):
        h, _ = self.req("run_mission", name="competition", tier=1)
        self.assertFalse(h["ok"])
        self.assertIn("bt_runner_node", h["msg"])
        h, _ = self.req("missions_list")
        self.assertTrue(h["ok"], h["msg"])
        self.assertIn("competition", h["data"]["names"])

    def test_tree_follows_bt_status(self):
        status = {"state": "RUNNING", "name": "competition", "running_node": "CircleBuoy",
                  "elapsed_s": 3.0, "nodes": []}
        self.bt.publish(String(data=json.dumps(status)))
        f, _ = self.wait_frame(lambda h: h["t"] == "tree" and h.get("running_node") == "CircleBuoy")
        self.assertTrue(f["ok"])

    def test_record_session(self):
        h, _ = self.req("record", kind="session", action="start")
        self.assertTrue(h["ok"], h["msg"])
        h, _ = self.req("record_list")
        self.assertIsNotNone(h["data"]["live"])
        name = h["data"]["live"]["name"]
        time.sleep(0.6)
        h, _ = self.req("record", kind="session", action="stop")
        self.assertTrue(h["ok"], h["msg"])
        h, _ = self.req("record_download", name=name)
        self.assertTrue(h["ok"], h["msg"])
        chunks = [b for f, b in self.frames if f["t"] == "file" and f["id"] == h["id"]]
        self.assertEqual(b"".join(chunks)[:2], b"\x1f\x8b")   # gzip
        h, _ = self.req("record_delete", name=name)
        self.assertTrue(h["ok"], h["msg"])

    def test_pictures_need_their_nodes(self):
        h, _ = self.req("cam", w=320)
        self.assertEqual((h["t"], h["ok"]), ("result", False))
        h, _ = self.req("lidar")
        self.assertFalse(h["ok"])

    def test_unknown_op(self):
        h, _ = self.req("fly")
        self.assertFalse(h["ok"])


if __name__ == "__main__":
    unittest.main()
