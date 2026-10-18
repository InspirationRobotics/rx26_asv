"""The Mission tab off-boat: the page's markup and script, mission_api (library endpoints and the run
rules, with a fake goal port), the body-size rule, and the laptop server answering the library locally
while forwarding start / stop / status.

    python -m pytest crusader_groundstation/test/test_mission_tab.py -q -p no:cacheprovider      (from rx26_asv; no ROS)

What this CANNOT check is the editor's behaviour in a browser (drag, wrap, map clicks, the XML round trip):
run test/fake_gcs.py and drive the page with the browser pane; MIS in the page's console is a seam for it.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "tools", "scripts"))

from crusader_groundstation import gcs_page, mission_api, mission_tab  # noqa: E402
from crusader_groundstation.gcs_server import GcsServer, MAX_BODY, body_limit  # noqa: E402
import fake_gcs  # noqa: E402
import fake_mission_lib  # noqa: E402

PAGE = gcs_page.render(200).decode("utf-8")
LOCAL = "127.0.0.1"
BT_DIR = os.path.join(REPO, "crusader_bt", "behavior_trees")

GOOD = ('<root BTCPP_format="4" main_tree_to_execute="Main"><BehaviorTree ID="Main"><Sequence name="m">'
        '<NavigateTo target="fix" lat="1.28" lon="103.85"/></Sequence></BehaviorTree></root>')
BAD = ('<root BTCPP_format="4" main_tree_to_execute="Main"><BehaviorTree ID="Main"><Sequence>'
       '<NavigateTo target="fix" lat="1.2"/></Sequence></BehaviorTree></root>')


def post(base, path, body, headers=None):
    req = urllib.request.Request(base + path, json.dumps(body).encode(),
                                 dict({"Content-Type": "application/json"}, **(headers or {})))
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())


class TempDirs(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="mission_test_")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.saved = os.path.join(self.tmp, "saved")
        self.run_dir = os.path.join(self.tmp, "run")
        self.bt = os.path.join(self.tmp, "bt")
        os.makedirs(self.bt)
        with open(os.path.join(self.bt, "demo.xml"), "w", encoding="utf-8") as f:
            f.write("<!-- MISSION: a demo -->" + GOOD)

    def lib(self):
        return mission_api.MissionLib(self.bt, self.saved, lib=fake_mission_lib)


# ---------------------------------------------------------------- the page

class Markup(unittest.TestCase):
    def test_the_tab_is_there_and_every_old_one_is_kept(self):
        tabs = re.findall(r'<button class="tab" data-t="(\w+)"', PAGE)
        self.assertIn("mission", tabs)
        for old in ("nodes", "tel", "cam", "lidar", "map", "task1", "tune", "rec", "logs", "radio", "sys"):
            self.assertIn(old, tabs)
        self.assertEqual(tabs[tabs.index("task1") + 1], "mission")
        self.assertIn('<div class="pane" id="p-mission">', PAGE)
        self.assertRegex(PAGE, r"\['nodes',[^\]]*'mission'\]\.forEach")        # show() knows the pane

    def test_the_toolbar_has_every_control_the_design_names(self):
        for ident in ("mpick", "mload", "mnew", "msave", "msaveas", "mval", "mxml", "mdown", "mup", "mstart",
                      "mstop", "mstat", "mq", "madv", "mpalbody", "mtabs", "mwork", "minsp", "mmap",
                      "mm_boat"):
            self.assertIn('id="%s"' % ident, mission_tab.PANE, ident)

    def test_every_placeholder_is_replaced_and_nothing_is_fetched_from_outside(self):
        self.assertNotRegex(PAGE, r"__[A-Z_]+__")
        for text in (mission_tab.CSS, mission_tab.PANE, mission_tab.JS):
            self.assertNotRegex(text, r"https?://|//cdn|@import|<script\s+src|<link\s")

    def test_the_laptop_patcher_anchors_are_not_used_by_the_tab(self):
        # gcs_laptop counts these in the page; the tab must not add one.
        whole = mission_tab.CSS + mission_tab.PANE + mission_tab.JS
        for anchor in ("location.hostname", "<main>", "</style>", "</script>", '<div id="bar">',
                       "NO CONNECTION TO THE BOAT", "if(tab==='tel') renderTel();"):
            self.assertNotIn(anchor, whole)

    def test_colours_are_variables_except_the_two_on_block_text_colours(self):
        hexes = [line for line in mission_tab.CSS.splitlines() if re.search(r"#[0-9a-fA-F]{3,6}\b", line)]
        self.assertEqual(len(hexes), 1, hexes)
        self.assertIn("--onblk:#fff;--onblk-dk:#111", hexes[0])
        self.assertNotIn("innerHTML = ' + ", mission_tab.JS)

    def test_text_from_files_and_the_boat_goes_through_esc(self):
        # Mission XML and validation messages are other people's text: never raw into innerHTML.
        for risky in ("esc(d.label)", "esc(n.attrs.name)", "esc(m)", "esc(d.desc"):
            self.assertIn(risky, mission_tab.JS)

    def test_the_hooks_the_page_calls_exist(self):
        for hook in ("window.misEnter", "window.misLeave", "window.misTick"):
            self.assertIn(hook, mission_tab.JS)
        self.assertIn("if(t==='mission') misEnter(); else misLeave();", PAGE)
        self.assertIn("if(tab==='mission') misTick();", PAGE)

    @unittest.skipUnless(shutil.which("node"), "node is not installed")
    def test_numbers_are_written_the_way_bt_cpp_4_10_types_them(self):
        # BT.CPP types a SubTree literal by its text: a double needs a decimal point, an int must not have one.
        src = re.search(r"function normNum\(.*?\n}\n", mission_tab.JS, re.S).group(0)
        cases = [("double", "6", "6.0"), ("double", "6.", "6.0"), ("double", "6.5", "6.5"), ("double", "-3", "-3.0"),
                 ("double", "{r}", "{r}"), ("double", "", ""), ("double", "1e3", "1e3"),
                 ("int", "8", "8"), ("int", "8.0", "8"), ("int", "8.5", "8.5"), ("int", "{n}", "{n}"),
                 ("string", "6", "6"), ("string", "", "")]
        code = src + "\nconsole.log(JSON.stringify(%s.map(c => normNum(c[0], c[1]))));" % json.dumps(cases)
        r = subprocess.run(["node", "-e", code], capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(json.loads(r.stdout), [c[2] for c in cases])

    def test_every_declared_tunable_is_written_and_only_inside_a_tree(self):
        js = mission_tab.JS
        self.assertIn("function subAttrs(n)", js)
        self.assertIn("inTree && n.tag === 'SubTree' ? subAttrs(n) : n.attrs", js)     # not the TreeNodesModel entry
        self.assertIn("ph = 'nav_* default'", js)                                       # blank default = use the nav_* param

    @unittest.skipUnless(shutil.which("node"), "node is not installed")
    def test_the_javascript_parses(self):
        with tempfile.TemporaryDirectory() as d:
            for i, body in enumerate(re.findall(r"<script>(.*?)</script>", PAGE, re.S)):
                path = os.path.join(d, "s%d.js" % i)
                with open(path, "w", encoding="utf-8") as f:
                    f.write(body)
                r = subprocess.run(["node", "--check", path], capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, r.stderr)


# ---------------------------------------------------------------- the body-size rule

class BodyLimit(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.seen = []
        cls.srv = GcsServer(b"x", lambda layers: {}, lambda p, body: cls.seen.append(p) or {"ok": True, "message": ""})
        cls.srv.start(0, LOCAL)
        cls.base = "http://%s:%d" % (LOCAL, cls.srv._server.server_address[1])

    @classmethod
    def tearDownClass(cls):
        cls.srv.stop()

    def test_limits(self):
        self.assertEqual(body_limit("/node/start"), MAX_BODY)
        self.assertGreater(body_limit("/mission/start"), 500_000)

    def test_a_mission_sized_body_is_accepted_only_on_mission_paths(self):
        # The server answers 413 without reading the body, so a client may see the 413 or a reset
        # connection (Windows does the latter): either way the action must never have run.
        self.assertTrue(post(self.base, "/mission/validate", {"xml": "x" * 60_000})["ok"])
        for path, size in (("/node/start", 60_000), ("/mission/validate", 2 << 20)):
            with self.subTest(path=path):
                with self.assertRaises((urllib.error.HTTPError, ConnectionError, OSError)):
                    post(self.base, path, {"xml": "x" * size})
        self.assertEqual(self.seen, ["/mission/validate"])


# ---------------------------------------------------------------- the library endpoints

class Library(TempDirs):
    def test_catalogue_list_load_validate_save(self):
        lib = self.lib()
        cat = lib.handle("/mission/catalogue", {})
        self.assertTrue(cat["ok"])
        self.assertIn("nodes", cat)
        self.assertEqual(cat["message"], "")
        listing = lib.handle("/mission/list", {})
        self.assertEqual([(m["name"], m["source"], m["description"]) for m in listing["missions"]],
                         [("demo", "prebuilt", "a demo")])
        loaded = lib.handle("/mission/load", {"name": "demo", "source": "prebuilt"})
        self.assertTrue(loaded["ok"])
        self.assertIn("<root", loaded["xml"])
        self.assertTrue(lib.handle("/mission/validate", {"xml": GOOD})["ok"])
        bad = lib.handle("/mission/validate", {"xml": BAD})
        self.assertFalse(bad["ok"])
        self.assertIn("Main > Sequence > NavigateTo: a GPS point needs lon", bad["message"])
        self.assertTrue(lib.handle("/mission/save", {"name": "mine", "xml": GOOD})["ok"])
        self.assertTrue(os.path.isfile(os.path.join(self.saved, "mine.xml")))
        again = lib.handle("/mission/save", {"name": "mine", "xml": GOOD})
        self.assertFalse(again["ok"])                                    # no silent overwrite
        self.assertTrue(lib.handle("/mission/save", {"name": "mine", "xml": GOOD, "overwrite": True})["ok"])

    def test_a_bad_request_is_an_answer_never_an_exception(self):
        lib = self.lib()
        for path, body in (("/mission/load", {"name": "demo", "source": "../etc"}),
                           ("/mission/load", {"source": "saved"}),
                           ("/mission/validate", {}),
                           ("/mission/validate", {"xml": 5}),
                           ("/mission/save", {"name": "x"}),
                           ("/mission/save", {"name": "x", "xml": "y" * (mission_api.MAX_XML + 1)}),
                           ("/mission/nope", {})):
            with self.subTest(path=path, body=str(body)[:40]):
                out = lib.handle(path, body)
                self.assertFalse(out["ok"])
                self.assertTrue(out["message"])

    def test_a_missing_library_fails_the_endpoints_with_a_reason(self):
        lib = mission_api.MissionLib(self.bt, self.saved)
        with mock.patch.object(mission_api.MissionLib, "lib", side_effect=ImportError("no mission_lib")):
            out = lib.handle("/mission/catalogue", {})
        self.assertFalse(out["ok"])
        self.assertIn("mission library failed", out["message"])
        self.assertIn("no mission_lib", out["message"])

    def test_the_library_raising_is_reported_not_propagated(self):
        class Boom:
            @staticmethod
            def catalogue(_):
                raise RuntimeError("a subtree file is torn")
        out = mission_api.MissionLib(self.bt, self.saved, lib=Boom).handle("/mission/catalogue", {})
        self.assertFalse(out["ok"])
        self.assertIn("a subtree file is torn", out["message"])


# ---------------------------------------------------------------- start / stop / status

class Port:
    """mission_api.MissionRun's goal port, scripted."""

    def __init__(self, accept=True, delay=0.0):
        self.accept, self.delay = accept, delay
        self.goals, self.cancels, self.on_feedback, self.on_result = [], 0, None, None
        self.tree_files, self.tree_ok = [], (True, "")

    def set_tree_file(self, path):
        self.tree_files.append(path)
        return self.tree_ok

    def send(self, tier, timeout_s, on_feedback, on_result):
        time.sleep(self.delay)
        if not self.accept:
            return False, "runner said no"
        self.goals.append((tier, timeout_s))
        self.on_feedback, self.on_result = on_feedback, on_result
        return True, ""

    def cancel(self):
        self.cancels += 1
        return True, ""


class Run(TempDirs):
    def make(self, **kw):
        self.port = Port(**{k: v for k, v in kw.items() if k in ("accept", "delay")})
        return mission_api.MissionRun(self.lib(), self.run_dir, self.port.set_tree_file, self.port,
                                      wait_s=kw.get("wait_s", 3.0))

    def start(self, run, xml=GOOD, **over):
        return run.handle("/mission/start", dict({"xml": xml, "name": "my run", "tier": "advanced",
                                                  "timeout_s": 600}, **over))

    def test_a_start_writes_the_file_points_the_runner_at_it_and_sends_the_goal(self):
        run = self.make()
        out = self.start(run)
        self.assertTrue(out["ok"], out)
        path = os.path.join(self.run_dir, "my_run.xml")
        self.assertEqual(self.port.tree_files, [path])
        with open(path, encoding="utf-8") as f:
            self.assertEqual(f.read(), GOOD)
        self.assertEqual(self.port.goals, [(1, 600.0)])                  # advanced = 1
        st = run.status()
        self.assertEqual((st["active"], st["state"], st["name"], st["tier"], st["file"]),
                         (True, "running", "my run", "advanced", path))

    def test_tiers_map_to_the_actions_codes(self):
        self.assertEqual(mission_api.TIERS, {"core": 0, "advanced": 1, "disruptive": 2})
        self.assertEqual(mission_api.OUTCOMES[3], "CANCELLED")

    def test_an_invalid_mission_is_refused_before_anything_is_written_or_sent(self):
        run = self.make()
        out = self.start(run, xml=BAD)
        self.assertFalse(out["ok"])
        self.assertIn("does not validate", out["message"])
        self.assertTrue(out["errors"])
        self.assertFalse(os.path.exists(self.run_dir))
        self.assertEqual((self.port.tree_files, self.port.goals), ([], []))
        st = run.status()
        self.assertEqual((st["active"], st["result"]["outcome"]), (False, "NOT STARTED"))

    def test_one_mission_at_a_time(self):
        run = self.make()
        self.assertTrue(self.start(run)["ok"])
        again = self.start(run, name="other")
        self.assertFalse(again["ok"])
        self.assertIn("already running", again["message"])
        self.assertEqual(len(self.port.goals), 1)

    def test_bad_arguments_are_refused_and_leave_nothing_active(self):
        run = self.make()
        for over in ({"tier": "ludicrous"}, {"tier": None}, {"timeout_s": 0}, {"timeout_s": -5},
                     {"timeout_s": "soon"}, {"timeout_s": float("nan")}, {"timeout_s": 1e9}, {"xml": ""}):
            with self.subTest(over=over):
                self.assertFalse(self.start(run, **over)["ok"])
                self.assertFalse(run.status()["active"])
        self.assertEqual(self.port.goals, [])

    def test_a_name_cannot_leave_the_run_directory(self):
        run = self.make()
        self.assertTrue(self.start(run, name="../../evil name")["ok"])
        files = os.listdir(self.run_dir)
        self.assertEqual(len(files), 1)
        self.assertEqual(files[0], "evil_name.xml")
        self.assertEqual(mission_api.safe_name("..\\.."), "mission")

    def test_a_runner_that_will_not_take_the_file_or_the_goal_is_a_refusal_with_its_reason(self):
        run = self.make()
        self.port.tree_ok = (False, "tree_file: parameter is read-only")
        out = self.start(run)
        self.assertFalse(out["ok"])
        self.assertIn("read-only", out["message"])
        self.assertEqual(self.port.goals, [])                            # the goal never went
        self.assertFalse(run.status()["active"])
        self.assertIn("read-only", run.status()["result"]["detail"])
        run = self.make(accept=False)
        out = self.start(run)
        self.assertIn("runner said no", out["message"])
        self.assertFalse(run.status()["active"])

    def test_feedback_and_the_result_reach_the_status(self):
        run = self.make()
        self.start(run)
        self.port.on_feedback({"phase": "TRANSIT", "progress": 0.5, "warning": "plan stale"})
        st = run.status()
        self.assertEqual((st["feedback"]["phase"], st["feedback"]["warning"]), ("TRANSIT", "plan stale"))
        self.assertIn("age_s", st["feedback"])
        self.port.on_result({"code": 0, "detail": "passed", "elapsed_s": 91.2})
        st = run.status()
        self.assertEqual((st["active"], st["state"], st["feedback"]), (False, "idle", None))
        self.assertEqual((st["result"]["outcome"], st["result"]["detail"]), ("SUCCESS", "passed"))
        self.assertTrue(self.start(run, name="again")["ok"])             # free again, and the old result is gone
        self.assertIsNone(run.status()["result"])

    def test_stop_cancels_the_goal_and_the_runners_result_ends_it(self):
        run = self.make()
        self.assertFalse(run.handle("/mission/stop", {})["ok"])          # nothing running
        self.start(run)
        out = run.handle("/mission/stop", {})
        self.assertTrue(out["ok"])
        self.assertEqual(self.port.cancels, 1)
        self.assertEqual(run.status()["state"], "cancelling")
        self.assertTrue(run.status()["active"])                          # still active until the runner says so
        self.assertFalse(self.start(run, name="x")["ok"])
        self.port.on_result({"code": 3, "detail": "cancelled"})
        self.assertEqual(run.status()["result"]["outcome"], "CANCELLED")

    def test_a_slow_boat_does_not_hold_the_request_and_the_status_says_starting(self):
        run = self.make(delay=0.6, wait_s=0.1)
        t0 = time.monotonic()
        out = self.start(run)
        self.assertLess(time.monotonic() - t0, 0.5)
        self.assertTrue(out["ok"])
        self.assertIn("starting", out["message"])
        self.assertEqual(run.status()["state"], "starting")
        self.assertFalse(self.start(run, name="x")["ok"])                # still one at a time
        deadline = time.monotonic() + 3
        while run.status()["state"] == "starting" and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual(run.status()["state"], "running")

    def test_an_unexpected_failure_is_an_answer(self):
        run = mission_api.MissionRun(self.lib(), self.run_dir, lambda p: 1 / 0, Port(), wait_s=2.0)
        out = self.start(run)
        self.assertFalse(out["ok"])
        self.assertFalse(run.status()["active"])


# ---------------------------------------------------------------- the laptop server

try:
    import gcs_laptop as gl
except ImportError as e:                       # no pymavlink / PyYAML on this machine
    gl = None
    GL_SKIP = "gcs_laptop needs pymavlink and PyYAML: %s" % e


@unittest.skipIf(gl is None, "gcs_laptop is not importable here")
class Laptop(TempDirs):
    """LaptopServer in front of a fake Jetson: library answered here, start / stop / status forwarded."""

    def setUp(self):
        super().setUp()
        self.actions = []
        self.jetson = GcsServer(b"jetson", lambda layers: {"boat": {"ok": True}}, self._jetson_action).start(0, LOCAL)
        self.addCleanup(self.jetson.stop)
        link = gl.JetsonLink("%s:%d" % (LOCAL, self.jetson._server.server_address[1]))
        self.server = gl.LaptopServer(gl.build_page(200), link, lambda: {}, mission=self.lib())
        self.server.start(0, LOCAL)
        self.addCleanup(self.server.stop)
        port = self.server._server.server_address[1]
        self.server.allowed_hosts = gl.allowed_hosts_for(LOCAL, port)
        self.base = "http://%s:%d" % (LOCAL, port)

    def _jetson_action(self, path, payload):
        self.actions.append((path, payload))
        return {"ok": True, "message": "jetson did " + path}

    def test_the_page_served_has_the_mission_tab_and_the_patches_still_apply(self):
        with urllib.request.urlopen(self.base + "/", timeout=5) as r:
            html = r.read().decode("utf-8")
        self.assertIn('data-t="mission"', html)
        self.assertIn('<div id="mavstrip">', html)
        self.assertEqual(html.count("location.hostname"), 2)

    def test_the_library_is_answered_here_and_never_reaches_the_jetson(self):
        self.assertIn("nodes", post(self.base, "/mission/catalogue", {}))
        self.assertEqual(len(post(self.base, "/mission/list", {})["missions"]), 1)
        self.assertTrue(post(self.base, "/mission/validate", {"xml": GOOD})["ok"])
        self.assertTrue(post(self.base, "/mission/load", {"name": "demo", "source": "prebuilt"})["ok"])
        self.assertTrue(post(self.base, "/mission/save", {"name": "laptop_made", "xml": GOOD})["ok"])
        self.assertTrue(os.path.isfile(os.path.join(self.saved, "laptop_made.xml")))     # saved on the laptop
        self.assertEqual(self.actions, [])

    def test_start_stop_and_status_are_forwarded_with_a_whole_mission_in_the_body(self):
        big = GOOD.replace("</Sequence>", "<!--" + "c" * 70_000 + "--></Sequence>")
        out = post(self.base, "/mission/start", {"xml": big, "name": "n", "tier": "core", "timeout_s": 1800})
        self.assertEqual(out, {"ok": True, "message": "jetson did /mission/start"})
        self.assertEqual(post(self.base, "/mission/stop", {})["message"], "jetson did /mission/stop")
        self.assertEqual(post(self.base, "/mission/status", {})["message"], "jetson did /mission/status")
        self.assertEqual([a[0] for a in self.actions], ["/mission/start", "/mission/stop", "/mission/status"])
        self.assertEqual(self.actions[0][1]["xml"], big)
        self.assertFalse(os.path.exists(self.run_dir))                   # the laptop wrote nothing for a start

    def test_other_actions_are_still_forwarded(self):
        post(self.base, "/trail/clear", {"a": 1})
        self.assertEqual(self.actions, [("/trail/clear", {"a": 1})])

    def test_a_request_from_another_site_cannot_save_into_the_repo(self):
        with self.assertRaises(urllib.error.HTTPError) as cm:
            post(self.base, "/mission/save", {"name": "evil", "xml": GOOD}, {"Origin": "http://evil.example"})
        self.assertEqual(cm.exception.code, 403)
        self.assertFalse(os.path.exists(os.path.join(self.saved, "evil.xml")))

    def test_with_the_jetson_down_the_library_still_works_and_a_start_says_nothing_was_sent(self):
        link = gl.JetsonLink("nohost.invalid:8090")
        server = gl.LaptopServer(gl.build_page(200), link, lambda: {}, mission=self.lib())
        server.start(0, LOCAL)
        self.addCleanup(server.stop)
        base = "http://%s:%d" % (LOCAL, server._server.server_address[1])
        self.assertIn("nodes", post(base, "/mission/catalogue", {}))
        out = post(base, "/mission/start", {"xml": GOOD, "name": "n", "tier": "core", "timeout_s": 10})
        self.assertFalse(out["ok"])
        self.assertIn("nothing was sent", out["message"])

    def test_the_default_directories_are_this_checkouts_bt_package(self):
        self.assertEqual(os.path.normpath(gl.MISSION_BT_DIR), os.path.normpath(BT_DIR))
        self.assertEqual(os.path.normpath(gl.MISSION_DIR),
                         os.path.normpath(os.path.join(REPO, "crusader_bt", "missions")))


# ---------------------------------------------------------------- gcs_node, ROS stubbed

class _Future:
    def __init__(self, result=None):
        self._cbs, self._result, self.done = [], result, result is not None

    def add_done_callback(self, cb):
        self._cbs.append(cb)
        if self.done:
            cb(self)

    def finish(self, result):
        self._result, self.done = result, True
        for cb in self._cbs:
            cb(self)

    def result(self):
        return self._result

    def cancel(self):
        pass


class _Handle:
    accepted = True

    def __init__(self):
        self.result_future = _Future()

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        return _Future(True)


class _Client:
    def __init__(self):
        self.goals, self.feedback, self.handle, self.server = [], None, _Handle(), True

    def wait_for_server(self, timeout_sec):
        return self.server

    def send_goal_async(self, goal, feedback_callback):
        self.goals.append(goal)
        self.feedback = feedback_callback
        return _Future(self.handle)


class GcsNode(unittest.TestCase):
    """gcs_node cannot be imported without ROS, so rclpy and the message packages are stubbed just long
    enough to import it: this checks the dispatch and the SafePassage port, not that ROS accepts them."""

    @classmethod
    def setUpClass(cls):
        import types

        class Anything(types.ModuleType):
            def __getattr__(self, name):
                if name.startswith("__"):
                    raise AttributeError(name)
                return mock.MagicMock(name=name)
        names = ["rclpy", "rclpy.action", "rclpy.node", "rclpy.executors", "rcl_interfaces", "rcl_interfaces.msg",
                 "std_msgs", "std_msgs.msg", "std_srvs", "std_srvs.srv", "crusader_msgs", "crusader_msgs.msg",
                 "crusader_msgs.action"]
        stubs = {n: Anything(n) for n in names}
        stubs["rclpy.node"].Node = type("Node", (), {})
        cls.patch = mock.patch.dict(sys.modules, stubs)
        cls.patch.start()
        sys.modules.pop("crusader_groundstation.gcs_node", None)
        sys.path.insert(0, os.path.join(REPO, "crusader_common"))
        try:
            from crusader_groundstation import gcs_node
        except ImportError as e:
            cls.patch.stop()
            raise unittest.SkipTest("gcs_node does not import even with ROS stubbed: %s" % e)
        cls.node = gcs_node

    @classmethod
    def tearDownClass(cls):
        sys.modules.pop("crusader_groundstation.gcs_node", None)
        cls.patch.stop()

    def port(self):
        client = _Client()
        self.node.ActionClient = lambda node, typ, name: client
        self.node.SafePassage.Goal = type("Goal", (), {})
        return self.node.SafePassagePort(object(), "/crsd/safe_passage"), client

    def test_the_goal_carries_the_tier_and_timeout_and_feedback_and_result_come_back(self):
        import types
        port, client = self.port()
        feedback, results = [], []
        self.assertEqual(port.send(2, 900, feedback.append, results.append), (True, ""))
        self.assertEqual((client.goals[0].tier, client.goals[0].timeout_s), (2, 900.0))
        client.feedback(types.SimpleNamespace(feedback=types.SimpleNamespace(
            phase="TRANSIT", progress=0.5, buoys_known=4, buoys_resolved=2, plan_version=3, warning="")))
        self.assertEqual((feedback[0]["phase"], feedback[0]["buoys_resolved"]), ("TRANSIT", 2))
        client.handle.result_future.finish(types.SimpleNamespace(
            result=types.SimpleNamespace(outcome=3, detail="cancelled", elapsed_s=12.34)))
        self.assertEqual((results[0]["code"], results[0]["detail"]), (3, "cancelled"))
        self.assertEqual(port.cancel(), (True, ""))

    def test_no_server_or_a_rejected_goal_is_a_reason(self):
        port, client = self.port()
        client.server = False
        ok, why = port.send(0, 10, print, print)
        self.assertFalse(ok)
        self.assertIn("/crsd/safe_passage", why)
        client.server, client.handle.accepted = True, False
        self.assertFalse(port.send(0, 10, print, print)[0])
        self.assertEqual(port.cancel(), (False, "no goal to cancel"))

    def test_every_mission_path_is_routed_to_the_right_half(self):
        import types
        fake = types.SimpleNamespace(mission=types.SimpleNamespace(handle=lambda p, b: ("lib", p)),
                                     mission_run=types.SimpleNamespace(handle=lambda p, b: ("run", p)))
        for path in mission_api.LIB_PATHS:
            self.assertEqual(self.node.GroundStation._action(fake, path, {}), ("lib", path))
        for path in mission_api.RUN_PATHS:
            self.assertEqual(self.node.GroundStation._action(fake, path, {}), ("run", path))

    def test_the_runners_tree_file_is_set_only_when_it_is_in_the_graph_and_accepts(self):
        import types

        class Tuning:
            reply = [{"name": "tree_file", "ok": True, "reason": ""}]

            def set(self, node, values):
                self.last = (node, values)
                return self.reply
        tuning = Tuning()
        fake = types.SimpleNamespace(p={"mission_runner_node": "/bt_runner_node"}, _graph_full=["/bt_runner_node"],
                                     tuning=tuning)
        self.assertEqual(self.node.GroundStation._mission_set_tree_file(fake, "/r/x.xml"), (True, ""))
        self.assertEqual(tuning.last, ("/bt_runner_node", {"tree_file": "/r/x.xml"}))
        Tuning.reply = [{"name": "tree_file", "ok": False, "reason": "read-only"}]
        ok, why = self.node.GroundStation._mission_set_tree_file(fake, "/r/x.xml")
        self.assertFalse(ok)
        self.assertIn("read-only", why)
        fake._graph_full = []
        self.assertIn("not in the ROS graph", self.node.GroundStation._mission_set_tree_file(fake, "/r/x.xml")[1])


# ---------------------------------------------------------------- the fake the browser uses

class FakeGcs(TempDirs):
    def test_the_fake_serves_the_mission_endpoints_end_to_end(self):
        port = fake_gcs.FakePort(run_s=0.6)
        fake = fake_gcs.Fake(False, self.tmp, self.tmp, fake_mission_lib, self.bt, self.saved, port)
        self.assertTrue(fake.action("/mission/catalogue", {})["ok"])
        out = fake.action("/mission/start", {"xml": GOOD, "name": "f", "tier": "core", "timeout_s": 30})
        self.assertTrue(out["ok"], out)
        deadline = time.monotonic() + 5
        while fake.action("/mission/status", {})["active"] and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertEqual(fake.action("/mission/status", {})["result"]["outcome"], "SUCCESS")
        boat = fake.snapshot(set())["boat"]
        self.assertTrue(boat["lat"] and boat["lon"] and boat["ok"])


if __name__ == "__main__":
    unittest.main()
