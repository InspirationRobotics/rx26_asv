"""gcs_bridge in-process, with a laptop speaking gcs_wire over a real socket. Needs rclpy and a
built workspace (crusader_msgs, crusader_bt for missions_list); skips itself without them.

    ROS_DOMAIN_ID=57 python3 crusader_groundstation/test/test_gcs_bridge.py
"""
import collections
import io
import itertools
import json
import os
import shutil
import socket
import sys
import tarfile
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, ".."))
try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String
    from crusader_msgs.msg import LatLonHead, PassagePlan
    from crusader_groundstation import gcs_bridge, gcs_wire
    from crusader_groundstation.gcs_bridge import (MANUAL_VERSION_BASE, OPS_WORKERS, Connection,
                                                   GcsBridge)
    HAVE_ROS = True
except Exception:                                   # no ROS, or the msgs are not built
    HAVE_ROS = False


class SlowLink:
    """A laptop's end of the WiFi, as the socket a Connection sends on: sendall waits while the
    link is held shut (clear `carrying`) and records the frames it carries, in wire order."""

    def __init__(self):
        self.frames = []
        self.carrying, self.stalled, self.closed = (threading.Event() for _ in range(3))

    def sendall(self, data):
        if not self.carrying.is_set():
            self.stalled.set()
            self.carrying.wait()
        if self.closed.is_set():
            raise OSError("link closed")
        self.frames.append(gcs_wire.read_frame(types.SimpleNamespace(recv=io.BytesIO(data).read)))

    def recv(self, n):
        self.closed.wait()
        return b""

    def shutdown(self, how):
        self.closed.set()
        self.carrying.set()

    def close(self):
        self.shutdown(None)


class Peaked(collections.deque):
    """A lane that remembers the most it ever held."""

    peak = 0

    def append(self, item):
        super().append(item)
        self.peak = max(self.peak, len(self))


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

    # -- a download on a slow link: a second laptop whose link the test holds shut

    def until(self, pred, what, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not pred():
            if time.monotonic() > deadline:
                self.fail(f"never saw {what}")
            time.sleep(0.01)

    def laptop(self):
        link = SlowLink()
        conn = Connection(link, ("slow-laptop", 0), self.node._on_frame, self.node.get_logger())
        with conn._cv:
            conn._bulk = Peaked()
        self.addCleanup(conn.close)
        return link, conn

    def session(self, mb=4):
        """A session of random bytes, which gzip cannot shrink: mb MB is mb * 4 chunks."""
        name = f"dl{next(self.ids)}"
        path = os.path.join(self.tmp, name)
        os.makedirs(path)
        with open(os.path.join(path, "frames.bin"), "wb") as f:
            f.write(os.urandom(mb << 20))
        self.addCleanup(shutil.rmtree, path, True)
        return name

    def download(self, conn, name):
        """The op on the bridge's own pool, as a laptop's request would run it."""
        return self.node._pool.submit(self.node._run_op, conn, {
            "t": "req", "id": 7, "op": "record_download", "name": name})

    def full_bulk(self, link, conn):
        self.until(lambda: link.stalled.is_set() and len(conn._bulk) == conn.BULK_QUEUE,
                   "the sender stuck on a chunk and the bulk lane full")

    def sent(self, link, t, **fields):
        return [h for h, _ in list(link.frames)
                if h["t"] == t and all(h.get(k) == v for k, v in fields.items())]

    def assert_workers_free(self):
        barrier = threading.Barrier(OPS_WORKERS)
        for job in [self.node._pool.submit(barrier.wait, 5) for _ in range(OPS_WORKERS)]:
            job.result(timeout=7)

    def test_control_frames_overtake_a_download(self):
        link, conn = self.laptop()
        name = self.session()
        self.download(conn, name)
        self.full_bulk(link, conn)
        conn.send({"t": "state", "stamp": 1.0})
        conn.send({"t": "result", "id": 99, "ok": True, "msg": "", "data": None})
        link.carrying.set()
        self.until(lambda: self.sent(link, "result", id=7), "the download's result", 10.0)
        kinds = [(h["t"], h.get("id")) for h, _ in link.frames]
        self.assertEqual(kinds[:3], [("file", 7), ("state", None), ("result", 99)])
        self.assertEqual(kinds[-2:], [("file", 7), ("result", 7)])    # the result ends the file
        self.assertTrue(link.frames[-2][0]["last"])
        self.assertEqual([h["seq"] for h in self.sent(link, "file")],
                         list(range(len(kinds) - 3)))
        data = b"".join(b for h, b in link.frames if h["t"] == "file")
        with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tar:
            self.assertEqual(tar.getmember(f"{name}/frames.bin").size, 4 << 20)

    def test_download_memory_stays_bounded(self):
        link, conn = self.laptop()
        self.download(conn, self.session())
        self.full_bulk(link, conn)
        time.sleep(0.5)                 # unthrottled, the producer would have queued it all by now
        self.assertEqual(len(conn._bulk), conn.BULK_QUEUE)
        link.carrying.set()
        self.until(lambda: self.sent(link, "result", id=7), "the download's result", 10.0)
        self.assertEqual(conn._bulk.peak, conn.BULK_QUEUE)
        self.assertGreater(len(self.sent(link, "file")), 2 * conn.BULK_QUEUE)

    def test_closed_link_aborts_download_and_frees_the_worker(self):
        link, conn = self.laptop()
        job = self.download(conn, self.session())
        self.full_bulk(link, conn)
        t0 = time.monotonic()
        conn.close()
        job.result(timeout=5)
        self.assertLess(time.monotonic() - t0, 2.0)
        self.assertFalse(conn._bulk)
        self.assert_workers_free()

    def test_stalled_link_aborts_download_with_an_error_result(self):
        link, conn = self.laptop()
        name = self.session()
        with mock.patch.object(gcs_bridge, "BULK_WAIT_S", 1.0):
            t0 = time.monotonic()
            self.download(conn, name).result(timeout=10)
            took = time.monotonic() - t0
        self.assertLess(took, 1.8)      # one wait: tarfile's own close() must not wait again
        link.carrying.set()
        self.until(lambda: self.sent(link, "result", id=7), "the error result")
        result = self.sent(link, "result", id=7)[0]
        self.assertFalse(result["ok"])
        self.assertIn("aborted", result["msg"])
        kinds = [h["t"] for h, _ in link.frames]
        self.assertNotIn("file", kinds[kinds.index("result"):])      # no chunk follows the error
        self.assertTrue(conn.alive)
        self.assert_workers_free()

    def test_full_control_lane_drops_the_oldest_state_first(self):
        link, conn = self.laptop()
        conn.send({"t": "hello"})
        self.until(link.stalled.is_set, "the sender stuck on the first frame")
        conn.send({"t": "state", "n": 0})
        for _ in range(conn.QUEUE - 1):
            conn.send({"t": "log", "items": []})
        self.assertTrue(conn.send({"t": "state", "n": 1}))            # evicts state 0
        self.assertTrue(conn.send({"t": "result", "id": 5, "ok": True}))    # evicts state 1
        self.assertFalse(conn.send({"t": "result", "id": 6, "ok": True}))   # nothing to evict
        self.assertEqual(conn.dropped, 3)
        link.carrying.set()
        self.until(lambda: len(link.frames) == conn.QUEUE + 1, "the control lane drained")
        self.assertEqual(self.sent(link, "state"), [])
        self.assertEqual([h["id"] for h in self.sent(link, "result")], [5])


if __name__ == "__main__":
    unittest.main()
