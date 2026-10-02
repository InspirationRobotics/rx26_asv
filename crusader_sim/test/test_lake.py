"""Offline tests for lake mode: goal_client, lake_goal, LakePanel (the dead-man, the field <-> course
round trip, the checkpoint labels, the routes), panel_feed's lake layers.

No ROS, no sim, no boat. Needs python3 with PyYAML and pymavlink (the WSL host has both):

    cd ~/robotx_ws/src/rx26_asv/crusader_sim && python3 -m unittest discover -s test -p "test_lake.py" -v

lake_goal's main() runs against a fake rclpy (sys.modules) that delivers FCU states and a SIGINT on
cue. The panel tests talk to a FakeBoat on a loopback port the OS picks (never 14555/14556).
"""
import ast
import contextlib
import io
import json
import os
import signal
import sys
import tempfile
import threading
import time
import types
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lake_testkit as K                                                      # noqa: E402

from crusader_sim import course as C                                          # noqa: E402
from crusader_sim import goal_client as G                                     # noqa: E402
from crusader_sim import lake_goal as LG                                      # noqa: E402
from crusader_sim import lake_panel as LP                                     # noqa: E402
from crusader_sim import panel_feed as PF                                     # noqa: E402
from crusader_sim import task1_panel as T                                     # noqa: E402

SRC = os.path.join(K.PKG, "crusader_sim")


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


# ------------------------------------------------------------------ fakes for goal_client / lake_goal

class Fut:
    def __init__(self):
        self._v, self._done = None, False

    def done(self):
        return self._done

    def result(self):
        return self._v

    def resolve(self, v):
        self._v, self._done = v, True


def result_msg(outcome=0, detail="ok"):
    return NS(result=NS(outcome=outcome, detail=detail, buoys_classified=7, buoys_passed_correctly=4,
                        elapsed_s=61.0))


class Handle:
    """A goal handle: cancel_goal_async resolves the result as CANCELLED (3) on the next spin,
    the way bt_runner does, unless `stuck`."""

    def __init__(self, accepted=True, stuck=False):
        self.accepted, self.stuck, self.cancels = accepted, stuck, 0
        self.res = Fut()
        self.cancel_pending = False

    def get_result_async(self):
        return self.res

    def cancel_goal_async(self):
        self.cancels += 1
        self.cancel_pending = not self.stuck
        f = Fut()
        f.resolve(NS(return_code=0))
        return f

    def tick(self):
        if self.cancel_pending:
            self.cancel_pending = False
            self.res.resolve(result_msg(3, "cancelled"))


class Client:
    def __init__(self, handle, accept_at=1):
        self.handle, self.accept_at, self.goals = handle, accept_at, []
        self.fut, self.spins, self.fb = Fut(), 0, None

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        self.fb = feedback_callback
        return self.fut

    def spin(self, _t):
        self.spins += 1
        if self.spins == self.accept_at and not self.fut.done():
            self.fut.resolve(self.handle)
        self.handle.tick()


class GoalClientTest(unittest.TestCase):
    def run_it(self, handle, stop=None, accept_at=1, on_spin=None, **kw):
        c = Client(handle, accept_at)
        lines = []
        stop = stop or threading.Event()

        def spin(t):
            c.spin(t)
            if on_spin:
                on_spin(c, c.spins, stop)
        out = G.run_goal(c, NS(g=1), spin, stop, say=lines.append, **kw)
        return out, c, lines

    def test_result_without_a_stop(self):
        h = Handle()
        out, c, lines = self.run_it(h, on_spin=lambda c, n, s: n == 4 and h.res.resolve(result_msg(0)))
        self.assertEqual(out.status, "result")
        self.assertFalse(out.cancelled)
        self.assertEqual(h.cancels, 0)
        self.assertTrue(any(l.startswith("[result] outcome 0") for l in lines))

    def test_a_stop_cancels_once_and_waits_for_the_trees_answer(self):
        h = Handle()
        out, c, lines = self.run_it(h, on_spin=lambda c, n, s: n == 3 and s.set())
        self.assertEqual(h.cancels, 1)
        self.assertEqual(out.status, "result")
        self.assertTrue(out.cancelled)
        self.assertEqual(out.result.outcome, 3)
        self.assertTrue(any("CANCELLING" in l for l in lines))

    def test_a_stop_before_acceptance_cancels_the_moment_the_goal_exists(self):
        h = Handle()
        stop = threading.Event()
        stop.set()
        out, c, lines = self.run_it(h, stop=stop, accept_at=3)
        self.assertEqual(h.cancels, 1)
        self.assertEqual(out.result.outcome, 3)

    def test_an_unconfirmed_cancel_is_reported_not_hidden(self):
        h = Handle(stuck=True)
        t = [0.0]

        def clock():
            t[0] += 1.0
            return t[0]
        out, c, lines = self.run_it(h, on_spin=lambda c, n, s: n == 2 and s.set(), cancel_wait_s=5.0, clock=clock)
        self.assertEqual(out.status, "cancel_unconfirmed")
        self.assertEqual(h.cancels, 1)
        self.assertTrue(any("not confirmed" in l for l in lines))

    def test_rejected_and_never_answered(self):
        out, _, lines = self.run_it(Handle(accepted=False))
        self.assertEqual(out.status, "rejected")
        t = [0.0]

        def clock():
            t[0] += 1.0
            return t[0]
        out, _, _ = self.run_it(Handle(), accept_at=10 ** 6, accept_wait_s=5.0, clock=clock)
        self.assertEqual(out.status, "not_accepted")

    def test_feedback_line_is_what_the_panel_parses(self):
        lines = []
        fb = G.feedback_printer(lines.append)
        msg = NS(feedback=NS(phase="TRANSIT", progress=0.4321, buoys_resolved=3, buoys_known=10,
                             plan_version=2, warning=""))
        fb(msg)
        fb(msg)                                      # an identical tick prints nothing
        self.assertEqual(len(lines), 1)
        m = T.TREE_RE.match(lines[0])
        self.assertIsNotNone(m, lines[0])
        self.assertEqual((m.group(1), m.group(3), m.group(4), m.group(5)), ("TRANSIT", "3", "10", "2"))

    def test_install_stop_turns_sigint_and_sigterm_into_the_event(self):
        old = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
        try:
            for sig in (signal.SIGINT, signal.SIGTERM):
                stop = G.install_stop()
                self.assertFalse(stop.is_set())
                os.kill(os.getpid(), sig)
                self.assertTrue(stop.wait(2.0), "signal %s did not set the stop event" % sig)
        finally:
            for s, h in old.items():
                signal.signal(s, h)


class FakeRos:
    """sys.modules stand-ins for rclpy / crusader_msgs, so lake_goal.main() runs off-ROS.
    `fcu` = [(mode, armed), ...] delivered to the FCU subscription on the first spin; `on_spin(n, ros)`
    is called on every spin (the test sends a signal from there)."""
    MODULES = ("rclpy", "rclpy.action", "rclpy.node", "crusader_msgs", "crusader_msgs.action", "crusader_msgs.msg")

    def __init__(self, fcu=(("GUIDED", True),), accept=True, on_spin=None, result_at=None, outcome=0):
        self.fcu, self.accept, self.on_spin, self.result_at, self.outcome = fcu, accept, on_spin, result_at, outcome
        self.spins, self.subs, self.pubs, self.goals, self.handle = 0, [], [], [], Handle(accepted=accept)
        self.fut = Fut()

    def __enter__(self):
        ros = self
        rclpy = types.ModuleType("rclpy")
        rclpy.init = lambda *a, **k: None
        rclpy.shutdown = lambda *a, **k: None

        def spin_once(node, timeout_sec=0.0):
            ros.spins += 1
            time.sleep(0.001)          # a real spin waits; lake_goal's FCU wait is wall-clock
            if ros.spins > 5000:       # a broken guard must FAIL the test, not hang the run
                raise AssertionError("the fake goal never finished: lake_goal sent a goal it should not have, or never cancelled")
            if ros.spins == 1:
                for mode, armed in ros.fcu:
                    for cb in [cb for t, cb in ros.subs if t == LG.FCU_TOPIC]:
                        cb(NS(mode=mode, armed=armed))
            if ros.goals and not ros.fut.done():
                ros.fut.resolve(ros.handle)
            ros.handle.tick()
            if ros.result_at and ros.spins == ros.result_at:
                ros.handle.res.resolve(result_msg(ros.outcome))
            if ros.on_spin:
                ros.on_spin(ros.spins, ros)
        rclpy.spin_once = spin_once

        class Node:
            def __init__(self, name):
                self.name = name

            def create_subscription(self, typ, topic, cb, qos):
                ros.subs.append((topic, cb))

            def create_publisher(self, typ, topic, qos):
                ros.pubs.append(topic)

            def create_client(self, *a, **k):
                ros.pubs.append("client")

            def destroy_node(self):
                pass

        class ActionClient:
            def __init__(self, node, typ, topic):
                self.topic = topic

            def wait_for_server(self, timeout_sec=0.0):
                return True

            def send_goal_async(self, goal, feedback_callback=None):
                ros.goals.append(goal)
                return ros.fut

        class SafePassage:
            class Goal:
                def __init__(self, **kw):
                    self.__dict__.update(kw)

            class Result:
                OUTCOME_SUCCESS = 0

        mods = {"rclpy": rclpy, "rclpy.action": types.ModuleType("rclpy.action"),
                "rclpy.node": types.ModuleType("rclpy.node"), "crusader_msgs": types.ModuleType("crusader_msgs"),
                "crusader_msgs.action": types.ModuleType("crusader_msgs.action"),
                "crusader_msgs.msg": types.ModuleType("crusader_msgs.msg")}
        mods["rclpy.action"].ActionClient = ActionClient
        mods["rclpy.node"].Node = Node
        mods["crusader_msgs.action"].SafePassage = SafePassage
        mods["crusader_msgs.msg"].FcuStatus = NS
        self.saved = {m: sys.modules.get(m) for m in self.MODULES}
        self.old_sig = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
        sys.modules.update(mods)
        return self

    def __exit__(self, *exc):
        for m, v in self.saved.items():
            if v is None:
                sys.modules.pop(m, None)
            else:
                sys.modules[m] = v
        for s, h in self.old_sig.items():
            signal.signal(s, h)


def run_main(args, ros):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        rc = LG.main(args)
    return rc, out.getvalue()


class LakeGoalTest(unittest.TestCase):
    def test_fcu_problem_table(self):
        self.assertIsNone(LG.fcu_problem("GUIDED", True))
        self.assertIsNone(LG.fcu_problem("guided", True))
        self.assertIn("NOT ARMED", LG.fcu_problem("GUIDED", False))
        self.assertIn("not GUIDED", LG.fcu_problem("HOLD", True))
        self.assertIn("MANUAL", LG.fcu_problem("MANUAL", True))
        self.assertIn("no /crsd/fcu_status", LG.fcu_problem(None, None))
        self.assertIn("no /crsd/fcu_status", LG.fcu_problem("GUIDED", None))

    def test_parse_approach(self):
        self.assertEqual(LG.parse_approach("1.30,103.85"), (1.30, 103.85))
        self.assertEqual(LG.parse_approach("none"), (0.0, 0.0))
        for bad in ("1.3", "a,b", "91,0", "0,181", "nan,1"):
            with self.assertRaises(ValueError, msg=bad):
                LG.parse_approach(bad)

    def test_refuses_when_not_armed_and_sends_nothing(self):
        with FakeRos(fcu=(("GUIDED", False),)) as ros:
            rc, out = run_main(["--fcu-wait-s", "1"], ros)
        self.assertEqual(rc, 3)
        self.assertIn("REFUSED", out)
        self.assertEqual(ros.goals, [])
        self.assertEqual(ros.pubs, [])

    def test_refuses_when_not_guided(self):
        for mode in ("HOLD", "MANUAL", "AUTO", "ACRO"):
            with FakeRos(fcu=((mode, True),)) as ros:
                rc, out = run_main(["--fcu-wait-s", "1"], ros)
            self.assertEqual(rc, 3, mode)
            self.assertEqual(ros.goals, [], mode)

    def test_refuses_with_no_status_at_all(self):
        with FakeRos(fcu=()) as ros:
            rc, out = run_main(["--fcu-wait-s", "0.3"], ros)
        self.assertEqual(rc, 3)
        self.assertIn("no /crsd/fcu_status", out)
        self.assertEqual(ros.goals, [])

    def test_the_newest_status_decides(self):
        with FakeRos(fcu=(("GUIDED", True), ("HOLD", True))) as ros:       # the pilot switched
            rc, _ = run_main(["--fcu-wait-s", "1"], ros)
        self.assertEqual(rc, 3)
        self.assertEqual(ros.goals, [])

    def test_sends_the_disruptive_goal_when_armed_and_guided(self):
        with FakeRos(result_at=4) as ros:
            rc, out = run_main(["--approach", "1.31,103.86", "--fcu-wait-s", "1"], ros)
        self.assertEqual(rc, 0, out)
        g = ros.goals[0]
        self.assertEqual((g.tier, g.approach_latitude, g.approach_longitude, g.timeout_s, g.orbit_radius_m),
                         (2, 1.31, 103.86, 600.0, 0.0))
        self.assertIn("[result] outcome 0", out)
        self.assertEqual(ros.pubs, [], "lake_goal must create no publisher and no client of any kind")

    def test_a_goal_that_fails_exits_1(self):
        with FakeRos(result_at=4, outcome=4) as ros:
            rc, out = run_main(["--fcu-wait-s", "1"], ros)
        self.assertEqual(rc, 1)

    def test_sigint_cancels_the_goal(self):
        def on_spin(n, ros):
            if n == 5:
                os.kill(os.getpid(), signal.SIGINT)
        with FakeRos(on_spin=on_spin) as ros:
            rc, out = run_main(["--fcu-wait-s", "1"], ros)
        self.assertEqual(ros.handle.cancels, 1)
        self.assertEqual(rc, 1)
        self.assertIn("CANCELLING", out)
        self.assertIn("outcome 3", out)

    def test_sigterm_cancels_the_goal(self):
        def on_spin(n, ros):
            if n == 5:
                os.kill(os.getpid(), signal.SIGTERM)
        with FakeRos(on_spin=on_spin) as ros:
            rc, out = run_main(["--fcu-wait-s", "1"], ros)
        self.assertEqual(ros.handle.cancels, 1)
        self.assertIn("outcome 3", out)

    def test_task1_goal_uses_the_same_cancel(self):
        src = read(os.path.join(SRC, "task1_goal.py"))
        self.assertIn("run_goal", src)
        self.assertIn("install_stop", src)


# ------------------------------------------------------------------ what lake_goal / lake_panel may not do

FORBIDDEN = ("operator_tools", "set_mode", "rc_override", "arducopter_arm", "COMPONENT_ARM_DISARM",
             "force_disarm", "mavutil", "mavlink_connection", "pymavlink", "14550", "14551", "14552")


def code_text(path):
    """Everything a module's CODE mentions: names, attributes, imports, string and number constants —
    not its docstrings or comments, which are free to say what the code must not do."""
    tree = ast.parse(read(path))
    doc_nodes = set()
    for n in ast.walk(tree):
        if isinstance(n, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)) and n.body:
            first = n.body[0]
            if isinstance(first, ast.Expr) and isinstance(getattr(first, "value", None), ast.Constant):
                doc_nodes.add(first.value)
    out = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Name):
            out.append(n.id)
        elif isinstance(n, ast.Attribute):
            out.append(n.attr)
        elif isinstance(n, ast.alias):
            out.append(n.name)
        elif isinstance(n, ast.ImportFrom):
            out.append(n.module or "")
        elif isinstance(n, ast.Constant) and n not in doc_nodes:
            out.append(str(n.value))
    return "\n".join(out)


class SafetyBoundaryTest(unittest.TestCase):
    def test_lake_goal_and_lake_panel_cannot_arm_set_modes_or_open_mavlink(self):
        for mod in ("lake_goal.py", "lake_panel.py", "goal_client.py"):
            text = code_text(os.path.join(SRC, mod))
            for word in FORBIDDEN:
                self.assertNotIn(word, text, "%s mentions %s in code" % (mod, word))

    def test_lake_page_has_no_such_control_and_says_it_is_not_an_estop(self):
        html = read(os.path.join(SRC, "lake_panel.html"))
        for word in ("set_mode", "rc_override", "/api/arm", "/api/disarm", "/api/launch", "/api/stop_sim",
                     "/api/attach", "14550", "14551"):
            self.assertNotIn(word, html)
        self.assertIn("NOT AN E-STOP", html)
        self.assertIn("RC SB switch", html)
        self.assertIn("Switch SC to HOLD", html)

    def test_the_lake_panels_routes_are_exactly_its_actions(self):
        self.assertNotIn("launch", LP.ACTIONS)
        self.assertNotIn("attach", LP.ACTIONS)
        self.assertNotIn("stop_sim", LP.ACTIONS)
        self.assertIsNone(T.Panel.ACTIONS, "the sim panel keeps every act_* as a route")
        for n in LP.ACTIONS:
            self.assertTrue(callable(getattr(LP.LakePanel, "act_" + n, None)), n)

    def test_the_sim_scripts_cannot_run_in_lake_mode(self):
        boat = K.FakeBoat()
        try:
            p = K.make_panel(boat)
            with self.assertRaises(RuntimeError):
                p._script("gz_sim_up.sh")
        finally:
            boat.stop()


# ------------------------------------------------------------------ the lake panel

class PanelCase(unittest.TestCase):
    def setUp(self):
        self.boat = K.FakeBoat()
        self.panel = K.make_panel(self.boat)
        self.feed = K.Feed(self.panel)
        self.feed.fcu()
        self.feed.pose(0.0, 0.0)
        self.feed.push()
        self._threads = []

    def tearDown(self):
        self.panel.quit.set()
        self.panel.shutdown()
        self.boat.stop()

    def loop_in_thread(self):
        t = threading.Thread(target=self.panel.loop, daemon=True)
        t.start()
        self._threads.append(t)

    def commit_field(self, field=None):
        r = self.panel.act_layout({"buoys": field or K.FIELD})
        self.assertTrue(r["ok"], r)
        r = self.panel.act_commit({})
        self.assertTrue(r["ok"], r)

    def wait_for(self, fn, timeout=3.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if fn():
                return True
            time.sleep(0.02)
        return False


class FieldAndCourseTest(PanelCase):
    def test_the_field_is_built_from_a_pin_a_typed_latlon_and_a_track(self):
        self.feed.pose(12.0, 3.0)
        self.feed.push()
        r = self.panel.act_pin({"state": "flash_blue"})
        self.assertTrue(r["ok"], r)
        la, lo = C.enu_to_latlon(40.0, -2.0, K.DATUM)
        r = self.panel.act_add_latlon({"lat": la, "lon": lo, "state": "steady_blue"})
        self.assertTrue(r["ok"], r)
        st = self.panel.state({})
        b = st["layout"]["buoys"]
        self.assertEqual([x["state"] for x in b], ["flash_blue", "steady_blue"])
        self.assertAlmostEqual(b[0]["x"], 12.0, delta=0.02)
        self.assertAlmostEqual(b[1]["x"], 40.0, delta=0.02)
        self.assertAlmostEqual(b[1]["lat"], la, delta=1e-6)
        self.assertEqual(st["layout"]["errors"], [])
        self.assertEqual(st["mode"], "build")

    def test_pin_without_a_fresh_pose_is_refused_not_guessed(self):
        panel = K.make_panel(self.boat)             # a panel that never heard the feed
        r = panel.act_pin({"state": "off"})
        self.assertFalse(r["ok"])
        self.assertIn("no fresh boat pose", r["error"])

    def test_ten_buoys_at_most_and_one_metre_apart(self):
        for i in range(10):
            self.assertTrue(self.panel.act_add_latlon(
                dict(zip(("lat", "lon"), C.enu_to_latlon(5.0 * i, 0.0, K.DATUM)), state="off"))["ok"])
        r = self.panel.act_add_latlon(dict(zip(("lat", "lon"), C.enu_to_latlon(99.0, 9.0, K.DATUM)), state="off"))
        self.assertFalse(r["ok"])
        self.assertIn("limit", r["error"])
        self.assertFalse(self.panel.act_layout({"buoys": [{"x": 0, "y": 0, "state": "off"}] * 11})["ok"])
        p2 = K.make_panel(self.boat)
        self.assertTrue(p2.act_layout({"buoys": [{"x": 0, "y": 0, "state": "off"}]})["ok"])
        r = p2.act_add_latlon(dict(zip(("lat", "lon"), C.enu_to_latlon(0.4, 0.0, K.DATUM)), state="off"))
        self.assertFalse(r["ok"])
        self.assertIn("within", r["error"])

    def test_commit_needs_exactly_one_entry_and_one_exit(self):
        self.panel.act_layout({"buoys": K.FIELD[1:]})
        r = self.panel.act_commit({})
        self.assertFalse(r["ok"])
        self.assertIn("ENTRY", r["error"])
        self.assertEqual(self.panel.state({})["mode"], "build")

    def test_commit_refuses_a_feed_with_another_origin(self):
        self.feed.push(origin={"lat": 1.2806, "lon": 103.8557})      # the sim's origin: not this datum
        self.panel.act_layout({"buoys": K.FIELD})
        r = self.panel.act_commit({})
        self.assertFalse(r["ok"])
        self.assertIn("ONE LAKE_DATUM", r["error"])

    def test_commit_then_stop_uav_returns_the_field_as_last_sent(self):
        self.commit_field()
        self.assertTrue(self.wait_for(lambda: self.boat.n_plans() >= 1))
        self.assertEqual(self.panel.state({})["mode"], "live")
        self.assertFalse(self.panel.act_layout({"buoys": []})["ok"], "positions are locked while committed")
        self.panel.act_stage({"id": 1, "state": "flash_green"})
        self.assertTrue(self.panel.act_send({})["ok"])
        r = self.panel.act_stop_uav({})
        self.assertTrue(r["ok"])
        st = self.panel.state({})
        self.assertEqual(st["mode"], "build")
        self.assertFalse(st["radio"]["up"])
        self.assertEqual(st["layout"]["buoys"][1]["state"], "flash_green", "the field as last SENT comes back")

    def test_course_round_trip_through_the_yaml(self):
        self.panel.act_layout({"buoys": K.FIELD})
        self.panel.act_approach({"x": 5.5, "y": -1.25})
        r = self.panel.act_save({"name": "lake_a"})
        self.assertTrue(r["ok"], r)
        c = C.load(r["path"])
        self.assertEqual(c["origin"], K.DATUM)
        self.assertEqual(c["approach"], {"x": 5.5, "y": -1.25})
        self.assertEqual(c["tier"], "disruptive")
        buoys, note = T.layout_of(c)
        self.assertEqual(len(buoys), len(K.FIELD))
        for got, want in zip(buoys, K.FIELD):
            self.assertEqual(got["state"], want["state"])
            self.assertAlmostEqual(got["x"], want["x"], delta=0.01)
            self.assertAlmostEqual(got["y"], want["y"], delta=0.01)
        # the course the sim replays puts each buoy at the SAME lat/lon the lake panel sends
        from crusader_sim.sim_uav import plan_from_course
        plan, entry, ex = plan_from_course(c)
        la, lo = C.enu_to_latlon(K.FIELD[0]["x"], K.FIELD[0]["y"], K.DATUM)
        self.assertAlmostEqual(entry[0], la, delta=1e-7)
        self.assertAlmostEqual(entry[1], lo, delta=1e-7)
        # ... and loading it back (a fresh panel, any origin in the file) places it from the datum
        p2 = K.make_panel(self.boat)
        p2.dir = self.panel.dir
        self.assertTrue(p2.act_load({"name": "lake_a"})["ok"])
        st = p2.state({})
        self.assertEqual([b["state"] for b in st["layout"]["buoys"]], [b["state"] for b in K.FIELD])
        self.assertEqual(st["approach"]["x"], 5.5)

    def test_course_saved_after_commit_holds_the_sent_colours_and_the_boats_start(self):
        self.feed.pose(2.0, 1.0, heading_deg=0.0)           # facing north = ENU yaw 90
        self.feed.push()
        self.commit_field()
        self.panel.act_stage({"id": 1, "state": "flash_green"})
        self.panel.act_send({})
        r = self.panel.act_save({"name": "lake_b"})
        c = C.load(r["path"])
        self.assertEqual(c["elements"][1]["beacon"], "flash_green")
        self.assertAlmostEqual(c["boat_start"]["x"], 2.0, delta=0.02)
        self.assertAlmostEqual(c["boat_start"]["yaw_deg"], 90.0, delta=0.2)
        self.assertEqual(self.panel.download("lake_b")[1], "lake_b.yaml")
        self.assertIsNone(self.panel.download("nope"))

    def test_a_course_from_the_sims_directory_loads_from_the_datum(self):
        r = self.panel.act_template({"name": "task1_core"})
        self.assertTrue(r["ok"], r)
        b = self.panel.state({})["layout"]["buoys"]
        core = C.buoys(C.load("task1_core"))
        self.assertEqual(len(b), 10)
        self.assertAlmostEqual(b[0]["x"], core[0][1])
        self.assertAlmostEqual(b[0]["y"], core[0][2])

    def test_the_field_in_progress_survives_a_restart_for_the_same_datum_only(self):
        self.panel.act_layout({"buoys": K.FIELD})
        again = K.make_panel(self.boat, tmp=self.panel.dir)
        self.assertEqual(len(again.state({})["layout"]["buoys"]), len(K.FIELD))
        from crusader_sim.lake_panel import LakePanel
        other = NS(**dict(vars(again.a), origin={"lat": 1.4, "lon": 103.9}))
        self.assertEqual(len(LakePanel(other).state({})["layout"]["buoys"]), 0)

    def test_sim_course_yaml_is_unchanged_by_the_generalisation(self):
        text = T.course_yaml(T.course_of("x", [{"x": 1.0, "y": 2.0, "state": "off"}]))
        self.assertIn("boat_start: {x: 0.0, y: 0.0, yaw_deg: 0.0}", text)
        self.assertNotIn("approach", text)
        self.assertIn("origin: {lat: 1.2806000, lon: 103.8557000}", text)


class CheckpointLabelTest(PanelCase):
    def test_labels_for_two_gates(self):
        L = lambda seq, n: T.checkpoint_text(seq, n)[0]
        self.assertEqual(L(1, 2), "ENTRY orbit done - confirm gate 1")
        self.assertEqual(L(2, 2), "gate 1 cleared - confirm gate 2")
        self.assertEqual(L(3, 2), "EXIT gate - confirm exit")
        self.assertEqual(L(1, 0), "ENTRY orbit done - confirm exit")
        self.assertEqual(L(4, 3), "EXIT gate - confirm exit")

    def test_the_panel_counts_gates_from_the_boats_report_else_from_the_field(self):
        self.commit_field()
        self.assertEqual(self.panel._gates(), 2)                  # min(2 red, 2 green) of the sent field
        self.feed.passage({"buoys": [], "n_gates": 3, "gates_cleared": 0, "single_count": 0})
        self.feed.push()
        self.assertEqual(self.panel._gates(), 3)                  # the boat's own count wins
        self.feed.passage({"buoys": []})                          # a tree that predates the key
        self.feed.push()
        self.assertEqual(self.panel._gates(), 2)

    def test_the_boats_asks_get_the_labels(self):
        self.commit_field()
        self.loop_in_thread()
        self.assertTrue(self.wait_for(lambda: self.boat.n_plans() >= 1))
        for seq in (1, 2, 3):
            self.boat.ask(seq)
            self.assertTrue(self.wait_for(lambda s=seq: any(r["seq"] == s for r in self.panel.state({})["checkpoints"])))
        labels = {r["seq"]: r["label"] for r in self.panel.state({})["checkpoints"]}
        self.assertEqual(labels[1], "ENTRY orbit done - confirm gate 1")
        self.assertEqual(labels[2], "gate 1 cleared - confirm gate 2")
        self.assertEqual(labels[3], "EXIT gate - confirm exit")


class DeadmanTest(PanelCase):
    """The field is resent only while a browser polls /api/state; then it stops, and resumes."""

    def setUp(self):
        super().setUp()
        self._resend, self._dm = T.RESEND_S, LP.DEADMAN_S
        T.RESEND_S, LP.DEADMAN_S = 0.2, 0.5
        self.polling = threading.Event()
        self.polling.set()
        self.stop_poll = threading.Event()
        threading.Thread(target=self._poller, daemon=True).start()
        self.loop_in_thread()

    def tearDown(self):
        self.stop_poll.set()
        T.RESEND_S, LP.DEADMAN_S = self._resend, self._dm
        super().tearDown()

    def _poller(self):
        while not self.stop_poll.is_set():
            if self.polling.is_set():
                self.panel.state({})
                self.feed.push()                      # keep the synthetic feed fresh as the real one is
            time.sleep(0.05)

    def test_resends_stop_without_a_poll_and_resume_with_one(self):
        self.commit_field()
        self.assertTrue(self.wait_for(lambda: self.boat.n_plans() >= 1))
        n0 = self.boat.n_plans()
        time.sleep(1.2)
        self.assertGreaterEqual(self.boat.n_plans() - n0, 3, "resends while polled: every 0.2 s")
        self.polling.clear()                          # the browser goes away
        time.sleep(LP.DEADMAN_S + 0.4)                # let the dead-man trip and any in-flight resend land
        n1 = self.boat.n_plans()
        time.sleep(1.5)
        self.assertEqual(self.boat.n_plans(), n1, "no resend may go out while nothing polls")
        self.polling.set()                            # back
        self.assertTrue(self.wait_for(lambda: self.boat.n_plans() > n1, 2.0), "resends resume")
        log = "\n".join(self.panel.logs["radio"].since(0)["lines"])
        self.assertIn("DEAD-MAN TRIPPED", log)
        self.assertIn("dead-man released", log)

    def test_auto_ack_is_held_back_with_the_resends(self):
        self.commit_field()
        self.assertTrue(self.wait_for(lambda: self.boat.n_plans() >= 1))
        self.assertTrue(self.panel.act_auto_ack({"on": True})["ok"])
        self.polling.clear()
        time.sleep(LP.DEADMAN_S + 0.4)
        self.boat.ask(1)
        time.sleep(1.2)
        self.assertEqual(self.boat.n_acks(), 0, "an unattended panel must not ACK for the UAV")
        self.polling.set()
        self.assertTrue(self.wait_for(lambda: self.boat.n_acks() >= 1, 3.0), "the ACK goes once a browser is back")

    def test_auto_ack_defaults_off(self):
        self.assertFalse(self.panel.state({})["radio"]["auto_ack"])

    def test_state_reports_the_gap_the_dead_man_saw(self):
        self.polling.clear()
        time.sleep(0.8)
        st = self.panel.state({})
        self.assertGreaterEqual(st["deadman"]["gap_s"], 0.7)
        self.assertEqual(st["deadman"]["window_s"], 0.5)


class StartAbortTest(PanelCase):
    def start_state(self, **fcu):
        self.feed.fcu(**fcu)
        self.feed.push()
        return self.panel.state({})["start_block"]

    def test_start_is_blocked_until_the_pilot_has_armed_and_chosen_guided(self):
        self.commit_field()
        self.assertIn("NOT ARMED", self.start_state(mode="GUIDED", armed=False))
        self.assertIn("not GUIDED", self.start_state(mode="HOLD", armed=True))
        self.assertIn("not GUIDED", self.start_state(mode="MANUAL", armed=True))
        self.assertIsNone(self.start_state(mode="GUIDED", armed=True))
        r = self.panel.act_start({})
        self.assertTrue(r["ok"], r)

    def test_start_refuses_without_a_committed_field_or_a_status(self):
        r = self.panel.act_start({})
        self.assertFalse(r["ok"])
        self.assertIn("COMMIT", r["error"])
        self.commit_field()
        p = K.make_panel(self.boat)                                  # never heard a feed: no FCU status
        p.act_layout({"buoys": K.FIELD})
        p.act_commit({})
        r = p.act_start({})
        self.assertFalse(r["ok"])
        self.assertIn("FCU", r["error"])
        p.act_stop_uav({})
        p.shutdown()

    def test_start_pure_function(self):
        fresh = lambda mode, armed: {"status": "fresh", "data": {"mode": mode, "armed": armed}}
        self.assertIsNone(LP.start_problem(fresh("GUIDED", True), True, True, False, True))
        self.assertIn("COMMIT", LP.start_problem(fresh("GUIDED", True), False, True, False, True))
        self.assertIn("already running", LP.start_problem(fresh("GUIDED", True), True, True, True, True))
        self.assertIn("dead-man", LP.start_problem(fresh("GUIDED", True), True, True, False, False))
        self.assertIn("FCU", LP.start_problem({"status": "stale", "data": None}, True, True, False, True))
        self.assertIn("FCU", LP.start_problem(None, True, True, False, True))

    def test_start_runs_lake_goal_with_the_approach_and_abort_signals_it(self):
        self.commit_field()
        self.start_state()
        self.panel.act_approach({"x": 8.0, "y": 1.0})
        self.panel.a.dry_run = False                    # only to see the real command line
        argv = self.panel._mission_argv()
        self.panel.a.dry_run = True
        self.assertEqual(argv[1:4], ["-u", "-m", "crusader_sim.lake_goal"])
        self.assertEqual(argv[argv.index("--timeout-s") + 1], "600")
        la, lo = C.enu_to_latlon(8.0, 1.0, K.DATUM)
        self.assertEqual(argv[argv.index("--approach") + 1], "%.7f,%.7f" % (la, lo))
        self.assertTrue(self.panel.act_start({})["ok"])
        self.assertTrue(self.wait_for(lambda: self.panel.state({})["mission"]["running"]))
        self.assertIn("START", "\n".join(self.panel.logs["mission"].since(0)["lines"]))
        self.assertIsNone(self.panel.state({})["mission"]["aborted_at"])
        self.assertTrue(self.panel.act_abort({})["ok"])
        self.assertTrue(self.wait_for(lambda: not self.panel.state({})["mission"]["running"], 5.0), "the child got SIGINT")
        st = self.panel.state({})
        self.assertIsNotNone(st["mission"]["aborted_at"])
        self.assertIn("switch SC to HOLD", "\n".join(self.panel.logs["mission"].since(0)["lines"]))

    def test_abort_with_nothing_running_still_says_so(self):
        r = self.panel.act_abort({})
        self.assertTrue(r["ok"])
        self.assertIsNotNone(self.panel.state({})["mission"]["aborted_at"])

    def test_the_approach_point_none_means_skip_the_drive(self):
        self.assertEqual(self.panel._approach_arg(), "none")
        self.panel.act_approach({"x": 3.0, "y": 4.0})
        self.assertNotEqual(self.panel._approach_arg(), "none")
        self.panel.act_approach({"clear": True})
        self.assertEqual(self.panel._approach_arg(), "none")


class HttpTest(PanelCase):
    def setUp(self):
        super().setUp()
        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), T.make_handler(self.panel))
        self.srv.daemon_threads = True
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.base = "http://127.0.0.1:%d" % self.srv.server_address[1]

    def tearDown(self):
        self.srv.shutdown()
        self.srv.server_close()
        super().tearDown()

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=5) as r:
            return r.status, r.headers, r.read()

    def post(self, path, body=None):
        req = urllib.request.Request(self.base + path, data=json.dumps(body or {}).encode(), method="POST",
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())

    def test_pages_and_shared_assets(self):
        st, h, body = self.get("/")
        self.assertIn(b"LAKE MODE", body)
        self.assertIn(b"NOT AN E-STOP", body)
        for p, ct in (("/panel_common.js", "javascript"), ("/panel_common.css", "css")):
            st, h, body = self.get(p)
            self.assertIn(ct, h["Content-Type"])
            self.assertGreater(len(body), 1000)

    def test_the_sims_launch_attach_and_stop_are_not_routes(self):
        for path in ("/api/launch", "/api/attach", "/api/stop_sim"):
            with self.assertRaises(urllib.error.HTTPError, msg=path) as cm:
                self.post(path)
            self.assertEqual(cm.exception.code, 404, path)

    def test_state_is_json_and_polling_it_is_the_dead_mans_heartbeat(self):
        t0 = self.panel.last_poll
        time.sleep(0.05)
        st, h, body = self.get("/api/state")
        d = json.loads(body)
        self.assertTrue(d["lake"])
        self.assertGreater(self.panel.last_poll, t0)
        for k in ("mode", "deadman", "fcu", "boat", "start_block", "approach", "layout", "feed"):
            self.assertIn(k, d)
        self.assertEqual(d["boat"]["x"], 0.0)
        self.assertEqual(d["fcu"]["status"], "fresh")

    def test_download_and_a_bad_name(self):
        self.panel.act_layout({"buoys": K.FIELD})
        r = self.post("/api/save", {"name": "dl_test"})
        self.assertTrue(r["ok"])
        st, h, body = self.get(r["download"])
        self.assertIn(b"robobuoy", body)
        self.assertIn("attachment", h["Content-Disposition"])
        self.assertFalse(self.post("/api/save", {"name": "../x"})["ok"])
        with self.assertRaises(urllib.error.HTTPError):
            self.get("/api/course/missing.yaml")


class SimPanelUnchangedTest(unittest.TestCase):
    def test_sim_panel_checks_and_labels_are_what_they_were(self):
        errs, warns = T.check_layout([{"x": 1.0, "y": 0.0, "state": "flash_blue"}, {"x": 9.0, "y": 0, "state": "steady_blue"}])
        self.assertTrue(any("3 m" in e or "min 3" in e for e in errs), "the sim keeps its 3 m clear zone")
        errs, _ = T.check_layout([{"x": 1.0, "y": 0.0, "state": "flash_blue"}, {"x": 9.0, "y": 0, "state": "steady_blue"}],
                                 start_clear_m=0.0)
        self.assertEqual(errs, [])
        self.assertEqual(T.field_gates(["flash_red", "flash_green", "flash_red"]), 1)


# ------------------------------------------------------------------ panel_feed's lake layers

class FeedLayersTest(unittest.TestCase):
    def test_the_modules_own_selftest_passes(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            fails = PF._selftest()
        self.assertEqual(fails, 0, out.getvalue()[-2000:])

    def test_the_datagram_stays_under_60k_with_every_layer_at_its_cap(self):
        origin = K.DATUM
        f = PF.Feeder(origin, "lake", clock=lambda: 0.0)
        f.on_datum(origin["lat"], origin["lon"])
        la, lo = C.enu_to_latlon(1.0, 1.0, origin)
        f.on_pose(NS(latitude=la, longitude=lo, heading=10.0, ground_speed=1.0), now=0.0)
        f.on_fcu(NS(mode="GUIDED", armed=True, system_status=4), now=0.0)
        hz = [NS(kind=1, source=2, id=i, x=0.0, y=0.0, radius_m=0.0, polygon_x=list(range(40)),
                 polygon_y=[j % 5 for j in range(40)], keepout_m=0.3) for i in range(300)]
        f.on_hazards(NS(header=NS(frame_id="map"), seq=1, hazards=hz), now=0.0)
        trk = [NS(id=i, label="flashing_blue_buoy_with_a_long_name", confidence=0.9, sources=3, latitude=la,
                  longitude=lo, position_stddev=0.2, hits=40, time_since_seen=0.1, confirmed=True) for i in range(200)]
        f.on_targets(NS(targets=trk), now=0.0)
        f.on_passage(json.dumps({"buoys": [{"position": {"latitude": la, "longitude": lo}, "state": "BEACON_STATE_OFF"}] * 100,
                                 "n_gates": 4}), now=0.0)
        pkt = f.packet(now=0.0, costmap=True)
        pkt["costmap"] = {"res_m": 0.1, "stamp": 1.0, "thr": 100, "n": 99999, "age": 0.1,
                          "cells": [[i * 0.1, 0.0] for i in range(PF.MAX_CELLS)], "n_lidar": 5,
                          "lidar": [[i * 0.1, 0.1] for i in range(PF.MAX_LIDAR)]}
        data = PF.fit(pkt)
        self.assertLess(len(data), 60000)
        self.assertEqual(len(pkt["hazards"]["items"]), PF.MAX_HAZARDS)
        rcv = PF.FeedReceiver(clock=lambda: 0.0)
        self.assertTrue(rcv.ingest(data, now=1.0))
        v = rcv.view(now=1.0)
        for layer in ("pose", "fcu", "hazards", "passage", "tracks"):
            self.assertEqual(v[layer]["status"], "fresh", layer)
        self.assertEqual(v["passage"]["data"]["n_gates"], 4)
        self.assertEqual(v["datum"]["lat"], origin["lat"])


if __name__ == "__main__":
    unittest.main()
