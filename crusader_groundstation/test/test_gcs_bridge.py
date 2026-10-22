"""gcs_bridge in-process, with a laptop speaking gcs_wire over a real socket. Needs rclpy and a
built workspace (crusader_msgs, crusader_bt for missions_list); skips itself without them.

    ROS_DOMAIN_ID=57 python3 crusader_groundstation/test/test_gcs_bridge.py
"""
import array
import collections
import contextlib
import io
import itertools
import json
import math
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
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from crusader_common import geo
    from crusader_msgs.msg import Cluster3D, Cluster3DArray, LatLonHead, PassagePlan
    from crusader_groundstation import gcs_bridge, gcs_wire, mission_xml
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


def clusters_msg(*specs):
    """specs: (x forward, y left, extent_x, extent_y, n_points), in base_link."""
    msg = Cluster3DArray()
    msg.header.frame_id = "base_link"
    for x, y, ex, ey, n in specs:
        c = Cluster3D(x=x, y=y, extent_x=ex, extent_y=ey, n_points=n, range=math.hypot(x, y))
        msg.clusters.append(c)
    return msg


def jpeg_msg(data):
    return CompressedImage(format="jpeg", data=array.array("B", data))


@unittest.skipUnless(HAVE_ROS, "needs rclpy and a built workspace")
class Placement(unittest.TestCase):
    """The pure part of the obstacles op: no node, no topics."""

    def test_heading_is_clockwise_from_north(self):
        lat0, lon0 = 1.3, 103.8
        # heading -> where 10 m ahead and 5 m to the left land, as (east, north) metres
        cases = {0: ((0, 10), (-5, 0)), 90: ((10, 0), (0, 5)), 180: ((0, -10), (5, 0)),
                 270: ((-10, 0), (0, -5)), 45: ((7.0711, 7.0711), (-3.5355, 3.5355))}
        for heading, (ahead, left) in cases.items():
            items = gcs_bridge.place_clusters([(10, 0, 0.3, 0.3, 40), (0, 5, 1.0, 2.0, 120)],
                                              lat0, lon0, heading)
            for item, want in zip(items, (ahead, left)):
                with self.subTest(heading=heading, want=want):
                    got = geo.latlon_to_xy(item["lat"], item["lon"], (lat0, lon0))
                    self.assertAlmostEqual(got[0], want[0], places=3)
                    self.assertAlmostEqual(got[1], want[1], places=3)

    def test_radius_covers_the_box_and_n_is_kept(self):
        a, b = gcs_bridge.place_clusters([(10, 0, 0.3, 0.3, 40), (0, 5, 1.0, 2.0, 120)],
                                         1.3, 103.8, 0.0)
        self.assertEqual((a["r"], a["n"]), (0.21, 40))
        self.assertEqual((b["r"], b["n"]), (1.12, 120))

    def test_a_cache_with_nothing_to_use_says_why(self):
        cache = gcs_bridge.StreamCache(1.0)
        self.assertEqual(gcs_bridge.stale_reason("LiDAR", cache, 10.0, ": is it up?"),
                         "no LiDAR heard: is it up?")
        cache.set([], 10.0)
        self.assertEqual(gcs_bridge.stale_reason("LiDAR", cache, 10.5), "")
        self.assertEqual(gcs_bridge.stale_reason("LiDAR", cache, 13.04), "LiDAR stale (3.0 s)")


@unittest.skipUnless(HAVE_ROS, "needs rclpy and a built workspace")
class Bridge(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        cls.missions = tempfile.mkdtemp()
        yaml_path = os.path.join(REPO, "crusader_bringup", "config", "crusader_params.yaml")
        rclpy.init(args=["--ros-args", "-p", "port:=0", "-p", f"record_dir:={cls.tmp}",
                         "-p", f"user_trees_dir:={cls.missions}", "-p", f"params_file:={yaml_path}"])
        cls.node = GcsBridge()
        cls.probe = rclpy.create_node("gcs_bridge_probe")
        cls.plans = []
        cls.probe.create_subscription(PassagePlan, "/crsd/passage_plan", cls.plans.append, 10)
        cls.pose = cls.probe.create_publisher(LatLonHead, "/crsd/pose", 10)
        cls.bt = cls.probe.create_publisher(String, "/crsd/bt_status", 10)
        cls.clusters = cls.probe.create_publisher(Cluster3DArray, gcs_bridge.CLUSTERS_TOPIC, 10)
        cls.cam = {"image": cls.probe.create_publisher(CompressedImage, gcs_bridge.IMAGE_TOPIC, 2),
                   "raw": cls.probe.create_publisher(CompressedImage, gcs_bridge.RAW_TOPIC, 2)}
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
        shutil.rmtree(cls.missions, True)

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
        h, _ = self.req("run_mission", name="task1_advanced")
        self.assertFalse(h["ok"])
        self.assertIn("bt_runner_node", h["msg"])
        h, _ = self.req("missions_list")
        self.assertTrue(h["ok"], h["msg"])
        self.assertIn("task1_advanced", h["data"]["names"])

    # ---------------------------------------------------------------- the mission files

    def mission_files(self):
        """What is in the user's directory, archive included, as relative paths."""
        return sorted(os.path.relpath(os.path.join(d, f), self.missions)
                      for d, _dirs, files in os.walk(self.missions) for f in files)

    def bt_says(self, state, name=None):
        """bt_runner_node's word, delivered: the bridge now thinks `name` is (not) running."""
        want = name if state == "RUNNING" else None
        deadline = time.monotonic() + 5
        while self.node._running_mission() != want and time.monotonic() < deadline:
            self.bt.publish(String(data=json.dumps({"state": state, "name": name})))
            time.sleep(0.05)
        self.assertEqual(self.node._running_mission(), want)

    def need_bt(self):
        if self.node._bt_install()[0] is None:
            self.skipTest("crusader_bt is not installed")
        shutil.rmtree(self.missions, True)
        self.bt_says("IDLE")

    def catalogue(self):
        h, _ = self.req("mission_catalogue")
        self.assertTrue(h["ok"], h["msg"])
        return h["data"]

    def mission(self, *blocks):
        """Mission XML for blocks, built the way the laptop builds it, from the boat's catalogue."""
        return mission_xml.build_xml(list(blocks) or [{"id": "AlwaysSuccess"}], self.catalogue())

    def upload(self, name, xml):
        h, _ = self.req("mission_upload", name=name, xml=xml)
        return h

    def test_mission_catalogue(self):
        self.need_bt()
        cat = self.catalogue()
        self.assertEqual(sorted(cat), ["builtins", "nodes", "subtrees", "user"])
        nodes = {n["id"]: n for n in cat["nodes"]}
        self.assertTrue({"Sequence", "AlwaysSuccess", "IsAutonomous", "NavigateTo"} <= set(nodes))
        goal = next(p for p in nodes["NavigateTo"]["ports"] if p["name"] == "goal")
        self.assertFalse(goal["required"])                  # read only for target="port"
        self.assertTrue(all("required" in p for n in cat["nodes"] for p in n["ports"]))
        tiers = {s["id"]: s for s in cat["subtrees"] if s.get("tier")}
        self.assertEqual((tiers["Task1Advanced"]["task"], tiers["Task1Advanced"]["tier"]), (1, "advanced"))
        lookahead = next(p for p in tiers["Task1Advanced"]["ports"] if p["name"] == "lookahead_m")
        self.assertEqual((lookahead["type"], lookahead["default"]), ("double", "2.5"))
        shared = [s for s in cat["subtrees"] if not s.get("tier")]
        self.assertIn({"id": "Task1PlanDrive", "ports": []}, shared)    # known, not a block to offer
        self.assertIn("task1_advanced", cat["builtins"])
        self.assertEqual(cat["user"], [])
        task1 = {"id": "Task1Advanced", "subtree": True}
        self.assertEqual(mission_xml.validate(self.mission(task1), cat, "mine"), [])

    def test_missions_list_keeps_names_and_says_whose(self):
        self.need_bt()
        self.assertTrue(self.upload("wp_mine", self.mission())["ok"])
        h, _ = self.req("missions_list")
        names, builtin, user = h["data"]["names"], h["data"]["builtin"], h["data"]["user"]
        self.assertEqual((user, names), (["wp_mine"], builtin + user))
        self.assertIn("task1_advanced", builtin)

    def test_mission_upload_get_replace_delete(self):
        self.need_bt()
        first, second = self.mission(), self.mission({"id": "AlwaysFailure"})
        h = self.upload("wp_mine", first)
        self.assertTrue(h["ok"], h["msg"])
        self.assertEqual((h["msg"], h["data"]), ("wp_mine saved", {"name": "wp_mine", "archived": None}))
        self.assertEqual(self.mission_files(), ["wp_mine.xml"])      # no temp file is left behind
        h, _ = self.req("mission_get", name="wp_mine")
        self.assertEqual(h["data"], {"name": "wp_mine", "xml": first, "builtin": False})
        h, _ = self.req("mission_get", name="task1_advanced")
        self.assertTrue(h["data"]["builtin"])
        self.assertIn("Task1Advanced", h["data"]["xml"])
        h, _ = self.req("mission_get", name="nope")
        self.assertFalse(h["ok"])
        self.assertIsNone(h["data"])

        h = self.upload("wp_mine", second)                           # replaced: the old is archived
        self.assertTrue(h["ok"], h["msg"])
        old = h["data"]["archived"]
        self.assertRegex(old, r"^wp_mine\.\d{8}-\d{6}\.xml$")
        self.assertEqual(self.mission_files(), [f".archive/{old}", "wp_mine.xml"])
        with open(os.path.join(self.missions, ".archive", old), encoding="utf-8") as f:
            self.assertEqual(f.read(), first)
        h, _ = self.req("mission_get", name="wp_mine")
        self.assertEqual(h["data"]["xml"], second)
        h, _ = self.req("missions_list")
        self.assertEqual(h["data"]["user"], ["wp_mine"])             # the archive is not a mission

        h, _ = self.req("mission_delete", name="wp_mine")            # moved, never removed
        self.assertTrue(h["ok"], h["msg"])
        self.assertEqual(h["msg"], "wp_mine deleted")
        gone = h["data"]["archived"]
        self.assertEqual(self.mission_files(), sorted([f".archive/{old}", f".archive/{gone}"]))
        with open(os.path.join(self.missions, ".archive", gone), encoding="utf-8") as f:
            self.assertEqual(f.read(), second)
        h, _ = self.req("missions_list")
        self.assertEqual(h["data"]["user"], [])

    def test_mission_upload_refusals_store_nothing(self):
        self.need_bt()
        cat = self.catalogue()
        kept = self.mission({"id": "AlwaysSuccess", "name": "keeper"})
        self.assertTrue(self.upload("wp_keep", kept)["ok"])
        wp = {"id": "NavigateTo", "ports": {"target": "fix", "lat": "1.30", "lon": "103.80"}}
        task1, task3 = ({"id": "Task1Advanced", "subtree": True}, {"id": "Task3Core", "subtree": True})
        cases = {                                   # what is wrong -> (name, xml, part of the reason)
            "a built-in name": ("task1_advanced", self.mission(), "built-in"),
            "a name that is a path": ("../escape", self.mission(), "mission name"),
            "an upper case name": ("Mine", self.mission(), "mission name"),
            "a 0/0 waypoint": ("wp_zero", self.mission(
                dict(wp, ports={"target": "fix", "lat": "0.0", "lon": "0.0"})), "both 0"),
            "a waypoint with no lat": ("wp_nolat", self.mission(
                dict(wp, ports={"target": "fix", "lon": "103.8"})), "needs a lat"),
            "two tiers of one task": ("wp_tiers", self.mission(
                task1, {"id": "Task1Disruptive", "subtree": True}), "one tier"),
            "an unknown block": ("wp_unknown", self.mission({"id": "NoSuchBlock"}), "unknown block"),
            "an <include>": ("wp_include", self.mission().replace(
                "<BehaviorTree", '<include path="lib/task1_advanced.xml"/><BehaviorTree', 1), "<include>"),
            "not XML": ("wp_junk", "this is not xml", "not well-formed"),
            "not text": ("wp_binary", 12345, "not text"),
        }
        for what, (name, xml, reason) in cases.items():
            h = self.upload(name, xml)
            self.assertFalse(h["ok"], what)
            self.assertTrue(h["msg"].startswith("not stored: "), (what, h["msg"]))
            self.assertIn(reason, h["msg"], what)
            self.assertTrue(h["data"]["problems"], what)
            self.assertTrue(all(set(p) == {"path", "msg"} for p in h["data"]["problems"]), what)
        self.assertEqual(self.mission_files(), ["wp_keep.xml"])      # nothing new, nothing archived
        self.assertEqual(mission_xml.validate(kept, cat, "wp_keep"), [])
        h = self.upload("wp_many", self.mission(task1, task3, {"id": "NoSuchBlock"},
                                                {"id": "NoSuchBlockEither"}, {"id": "AndAThird"},
                                                {"id": "AndAFourth"}))
        self.assertRegex(h["msg"], r"\(and \d+ more\)$")           # the status line carries three

    def test_mission_upload_refusal_is_one_shape(self):
        self.assertEqual(gcs_bridge._refusal("not stored", ["a", "b", "c"]), "not stored: a; b; c")
        self.assertEqual(gcs_bridge._refusal("not stored", list("abcde")),
                         "not stored: a; b; c (and 2 more)")

    def test_mission_upload_refused_when_bt_cpp_will_not_load_it(self):
        self.need_bt()
        old = self.mission()
        self.assertTrue(self.upload("wp_keep", old)["ok"])
        bad = '<root BTCPP_format="4"><BehaviorTree ID="Main"><NoSuchLeaf/></BehaviorTree></root>'
        with mock.patch.object(gcs_bridge.mission_xml, "problems", return_value=[]):
            h = self.upload("wp_keep", bad)             # the laptop's rules let it through
        self.assertFalse(h["ok"])
        self.assertIn("BehaviorTree.CPP will not load it", h["msg"])
        self.assertIn("NoSuchLeaf", h["msg"])
        self.assertEqual(self.mission_files(), ["wp_keep.xml"])      # staged file gone, old untouched
        h, _ = self.req("mission_get", name="wp_keep")
        self.assertEqual(h["data"]["xml"], old)

    def test_mission_changes_are_refused_while_a_mission_runs(self):
        self.need_bt()
        self.assertTrue(self.upload("wp_run", self.mission())["ok"])
        self.bt_says("RUNNING", "wp_run")
        h = self.upload("wp_other", self.mission())                 # any running mission, not just that one
        self.assertFalse(h["ok"])
        self.assertIn("wp_run is running", h["msg"])
        h, _ = self.req("mission_delete", name="wp_run")
        self.assertFalse(h["ok"])
        self.assertIn("is running", h["msg"])
        self.assertEqual(self.mission_files(), ["wp_run.xml"])
        self.bt_says("IDLE")
        self.assertTrue(self.upload("wp_other", self.mission())["ok"])
        self.assertTrue(self.req("mission_delete", name="wp_run")[0]["ok"])

    def test_mission_delete_refusals(self):
        self.need_bt()
        for name, reason in (("task1_advanced", "built in"), ("nope", "no mission of yours"),
                             ("../x", "no mission of yours"), ("", "no mission of yours")):
            h, _ = self.req("mission_delete", name=name)
            self.assertFalse(h["ok"], name)
            self.assertIn(reason, h["msg"], name)
        self.assertEqual(self.mission_files(), [])

    def test_mission_ops_need_the_installed_crusader_bt(self):
        with mock.patch.object(GcsBridge, "_bt_install", return_value=(None, None)):
            for op, fields in (("missions_list", {}), ("mission_catalogue", {}),
                               ("mission_get", {"name": "x"}),
                               ("mission_upload", {"name": "x", "xml": "<root/>"}),
                               ("mission_delete", {"name": "x"})):
                h, _ = self.req(op, **fields)
                self.assertEqual((h["ok"], h["msg"]), (False, "crusader_bt is not installed"), op)

    def test_mission_nodes_are_asked_again_only_for_another_build(self):
        self.need_bt()
        self.node._nodes_cache = None
        with mock.patch.object(self.node, "_bt_check", wraps=self.node._bt_check) as check:
            first = self.node._nodes()
            self.assertIs(self.node._nodes(), first)
            self.assertEqual([c.args for c in check.call_args_list], [("--models",)])
            self.node._nodes_cache = ((0, 0), first)                # a rebuilt bt_tree_check
            self.assertEqual(self.node._nodes(), first)
            self.assertEqual(check.call_count, 2)

    def test_tree_follows_bt_status(self):
        status = {"state": "RUNNING", "name": "task1_advanced", "running_node": "CircleBuoy",
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

    # -- the on-demand topics: leased by a laptop that keeps asking, dropped soon after

    LEASED = {"image", "raw", "preview", "clusters"}

    @contextlib.contextmanager
    def publishing(self, *pairs, hz=10.0):
        """(publisher, msg) pairs sent at hz on a thread, as the sensor nodes would."""
        stop = threading.Event()

        def run():
            while not stop.is_set():
                for pub, msg in pairs:
                    pub.publish(msg)
                stop.wait(1.0 / hz)

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        try:
            yield
        finally:
            stop.set()
            thread.join(2)
            time.sleep(0.2)             # what was in flight reaches the bridge before we go on

    def release_leases(self):
        """Every lease expired now, and the bridge's subscriptions dropped with them."""
        for key in list(self.node._leases):     # values only: the node iterates this dict
            self.node._leases[key] = 0.0
        self.until(lambda: not self.LEASED & set(self.node._subs), "the leases dropped")
        # a real lease outlives the clusters' freshness; an early expiry must not let the last
        # clusters heard before the drop pass for fresh after the next subscription
        self.node._clusters.set([], time.monotonic() - 3600.0)

    def fake_rescale(self):
        """-> the (jpeg, w, q) each picture was handed to rescale_jpeg with; no cv2 needed."""
        calls = []

        def rescale(jpeg, width, quality):
            calls.append((jpeg, width, quality))
            return jpeg, width, 9

        patcher = mock.patch.object(gcs_bridge, "rescale_jpeg", rescale)
        patcher.start()
        self.addCleanup(patcher.stop)
        return calls

    def boat_pose(self, heading=90.0):
        return LatLonHead(latitude=1.3, longitude=103.8, heading=heading, ground_speed=0.0)

    def test_obstacles_are_placed_around_the_boat(self):
        self.addCleanup(self.release_leases)
        lat, lon = 1.3, 103.8
        msg = clusters_msg((10.0, 0.0, 0.3, 0.3, 40), (0.0, 5.0, 1.0, 2.0, 120))
        with self.publishing((self.pose, self.boat_pose(90.0)), (self.clusters, msg)):
            h, _ = self.req("obstacles")
        self.assertTrue(h["ok"], h["msg"])
        d = h["data"]
        self.assertEqual(d["reason"], "")
        self.assertLess(max(d["clusters_age"], d["pose_age"]), 1.0)
        ahead, left = sorted(d["items"], key=lambda i: i["n"])
        # facing east: 10 m ahead is 10 m east, 5 m to the left is 5 m north
        self.assertAlmostEqual(ahead["lat"], lat, places=7)
        metres_per_deg_lon = geo.M_PER_DEG * math.cos(math.radians(lat))
        self.assertAlmostEqual(ahead["lon"], lon + 10 / metres_per_deg_lon, places=7)
        self.assertAlmostEqual(left["lat"], lat + 5 / geo.M_PER_DEG, places=7)
        self.assertAlmostEqual(left["lon"], lon, places=7)
        self.assertEqual((ahead["r"], left["r"]), (0.21, 1.12))

    def test_no_clusters_is_an_answer_not_a_blank(self):
        self.addCleanup(self.release_leases)
        with self.publishing((self.pose, self.boat_pose()), (self.clusters, clusters_msg())):
            h, _ = self.req("obstacles")
        self.assertTrue(h["ok"], h["msg"])
        self.assertEqual((h["data"]["items"], h["data"]["reason"]), ([], ""))

    def test_obstacles_are_blank_when_the_pose_is_stale(self):
        self.addCleanup(self.release_leases)
        self.node._pose.set((1.3, 103.8, 90.0, 0.0), time.monotonic() - 5.0)
        with self.publishing((self.clusters, clusters_msg((10.0, 0.0, 0.3, 0.3, 40)))):
            h, _ = self.req("obstacles")
        d = h["data"]
        self.assertTrue(h["ok"], h["msg"])
        self.assertEqual(d["items"], [])
        self.assertIn("boat position stale", d["reason"])
        self.assertNotIn("LiDAR", d["reason"])
        self.assertGreaterEqual(d["pose_age"], 5.0)
        self.assertLess(d["clusters_age"], 1.0)

    def test_obstacles_are_blank_when_the_clusters_are_old(self):
        self.addCleanup(self.release_leases)
        self.node._clusters.set([(10.0, 0.0, 0.3, 0.3, 40)], time.monotonic() - 5.0)
        with self.publishing((self.pose, self.boat_pose())):    # lidar_node has gone quiet
            h, _ = self.req("obstacles")
        d = h["data"]
        self.assertTrue(h["ok"], h["msg"])
        self.assertEqual(d["items"], [])
        self.assertIn("LiDAR clusters stale", d["reason"])
        self.assertNotIn("boat position", d["reason"])
        self.assertGreaterEqual(d["clusters_age"], 5.0)
        self.assertLess(d["pose_age"], 1.0)

    def test_obstacles_are_blank_without_a_heading(self):
        self.addCleanup(self.release_leases)
        with self.publishing((self.pose, self.boat_pose(float("nan"))),
                             (self.clusters, clusters_msg((10.0, 0.0, 0.3, 0.3, 40)))):
            h, _ = self.req("obstacles")
        self.assertEqual(h["data"]["items"], [])
        self.assertEqual(h["data"]["reason"], "boat heading unknown")

    def test_obstacles_are_subscribed_only_while_asked(self):
        self.release_leases()
        self.until(lambda: self.probe.count_subscribers(gcs_bridge.CLUSTERS_TOPIC) == 0,
                   "nobody subscribed to the clusters before the first ask")
        msg = clusters_msg((10.0, 0.0, 0.3, 0.3, 40))
        with mock.patch.object(gcs_bridge, "LEASE_S", 0.6):
            with self.publishing((self.pose, self.boat_pose()), (self.clusters, msg)):
                h, _ = self.req("obstacles")
                self.assertEqual(len(h["data"]["items"]), 1)
                self.assertIn("clusters", self.node._subs)
                self.until(lambda: self.probe.count_subscribers(gcs_bridge.CLUSTERS_TOPIC) == 1,
                           "the bridge subscribed to the clusters")
                self.until(lambda: "clusters" not in self.node._subs,
                           "the subscription dropped after the last ask", 3.0)
        self.until(lambda: self.probe.count_subscribers(gcs_bridge.CLUSTERS_TOPIC) == 0,
                   "the clusters unsubscribed in the graph")

    def test_cam_stream_is_annotated_unless_raw_is_asked_for(self):
        self.release_leases()
        self.addCleanup(self.release_leases)
        calls = self.fake_rescale()
        with self.publishing((self.cam["image"], jpeg_msg(b"annotated")),
                             (self.cam["raw"], jpeg_msg(b"raw"))):
            for fields in ({}, {"stream": "annotated"}):
                with self.subTest(fields=fields):
                    h, blob = self.req("cam", w=400, q=70, **fields)
                    self.assertEqual((h["t"], h["stream"], blob),
                                     ("cam", "annotated", b"annotated"))
                    self.assertEqual(calls[-1], (b"annotated", 400, 70))
            self.assertNotIn("raw", self.node._subs)    # annotated viewers leave raw alone
            h, blob = self.req("cam", w=400, q=70, stream="raw")
            self.assertEqual((h["t"], h["stream"], blob), ("cam", "raw", b"raw"))
            self.assertEqual(calls[-1], (b"raw", 400, 70))     # scaled and encoded as the other is
            self.assertIn("raw", self.node._subs)

    def test_cam_refuses_a_stream_it_does_not_know(self):
        self.release_leases()
        for bad in ("sepia", "", None):
            with self.subTest(stream=bad):
                h, _ = self.req("cam", stream=bad)
                self.assertEqual((h["t"], h["ok"]), ("result", False))
                self.assertIn("annotated or raw", h["msg"])
        self.assertFalse(self.LEASED & set(self.node._subs))     # refused before anything leased

    def test_no_raw_picture_says_which_one(self):
        self.release_leases()
        self.addCleanup(self.release_leases)
        h, _ = self.req("cam", stream="raw")
        self.assertFalse(h["ok"])
        self.assertIn("no raw picture", h["msg"])

    def stop_recording(self):
        name = self.node.recorder.session and self.node.recorder.session.name
        self.req("record", kind="session", action="stop")
        if name:
            shutil.rmtree(os.path.join(self.tmp, name), True)

    def test_raw_stays_subscribed_while_recording_or_viewing_needs_it(self):
        self.release_leases()
        self.addCleanup(self.release_leases)
        self.addCleanup(self.stop_recording)
        calls = self.fake_rescale()
        with mock.patch.object(gcs_bridge, "LEASE_S", 1.0), \
                self.publishing((self.cam["raw"], jpeg_msg(b"raw"))):
            h, _ = self.req("record", kind="session", action="start")
            self.assertTrue(h["ok"], h["msg"])
            self.until(lambda: "raw" in self.node._subs, "the recorder's raw subscription")
            frames = self.node.recorder.session.frame_stats
            self.until(lambda: frames()[0].get("camera", 0) >= 1, "a recorded frame")
            h, blob = self.req("cam", stream="raw")             # recording AND viewing
            self.assertEqual(blob, b"raw")
            time.sleep(1.3)                                     # the viewer's lease has run out
            self.assertIn("raw", self.node._subs)
            before = frames()[0]["camera"]
            self.until(lambda: frames()[0]["camera"] > before, "recording on without a viewer")
            self.req("cam", stream="raw")                       # viewing again; the recording ends
            h, _ = self.req("record", kind="session", action="stop")
            self.assertTrue(h["ok"], h["msg"])
            self.assertIn("raw", self.node._subs)               # the viewer's lease still holds it
            self.until(lambda: "raw" not in self.node._subs, "the raw subscription dropped", 4.0)
            self.assertEqual(calls[-1][0], b"raw")
            h, blob = self.req("cam", stream="raw")             # viewing alone
            self.assertEqual(blob, b"raw")
            self.assertIn("raw", self.node._subs)
            self.until(lambda: "raw" not in self.node._subs, "a lone viewer's lease dropped", 4.0)

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
