"""The ground-station page, off-boat: markup, the couplings the Task 1 / planner changes rely on, and
the page served by the real GcsServer against test/fake_gcs.py's invented state.

    python3 -m unittest discover -s crusader_groundstation/test        (from the repo root; no ROS)

What this CANNOT check is the page's JavaScript behaviour: that is read in a browser against
fake_gcs.py (its docstring says how), by DOM reads, not screenshots.
"""
import json
import os
import re
import sys
import unittest
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

from crusader_groundstation import gcs_page  # noqa: E402
from crusader_groundstation.gcs_server import GcsServer  # noqa: E402
import fake_gcs  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
PAGE = gcs_page.render(200).decode("utf-8")
# The page builds long strings out of adjacent JS literals ('a ' + 'b'); text assertions read it joined.
FLAT = re.sub(r"'\s*\+\s*'", "", PAGE)
BT_RUNNER = os.path.join(REPO, "crusader_bt", "src", "bt_runner_node.cpp")
SIM = os.path.join(REPO, "crusader_sim", "crusader_sim")


def _read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


class Markup(unittest.TestCase):
    def test_task1_tab_sits_next_to_map(self):
        tabs = re.findall(r'<button class="tab" data-t="(\w+)"', PAGE)
        self.assertEqual(tabs[tabs.index("map") + 1], "task1")
        self.assertIn('<div class="pane" id="p-task1">', PAGE)
        self.assertIn("'map','task1','tune'", PAGE)           # show() knows the pane

    def test_the_dead_man_is_said_in_the_tab(self):
        self.assertIn("Leaving this tab stops the UAV heartbeat", FLAT)
        self.assertIn("about 15 s later", FLAT)
        self.assertIn("RC SB switch is the only e-stop", FLAT)

    def test_how_to_start_the_panel_is_in_the_unreachable_message(self):
        self.assertIn("LAKE_DATUM=&lt;lat,lon&gt; bash /root/robotx_ws/src/rx26_asv/"
                      "crusader_sim/scripts/lake_rig_up.sh", PAGE.replace("'\n             + '", ""))
        self.assertIn("docker exec -it asv bash", FLAT)

    def test_no_frame_in_the_static_markup(self):
        # The iframe is created by script only while the tab is in front (task1Enter / task1Leave).
        markup = PAGE.split("<body>", 1)[1].split("<script>", 1)[0]
        self.assertNotIn("<iframe", markup)
        self.assertIn("document.createElement('iframe')", PAGE)

    def test_default_url_is_the_page_host_on_8095_and_storage_is_guarded(self):
        self.assertIn("location.protocol + '//' + location.hostname + ':' + T1_PORT + '/'", PAGE)
        self.assertIn("T1_PORT = 8095", PAGE)
        for fn in ("t1Saved", "t1Store"):
            body = PAGE.split("function %s(" % fn, 1)[1].split("\n}", 1)[0]
            self.assertIn("try{", body, fn)
            self.assertIn("catch(e)", body, fn)

    def test_probe_is_not_a_poll_of_the_panels_state_route(self):
        probe = PAGE.split("function t1Probe(", 1)[1].split("\n}", 1)[0]
        self.assertIn("method:'HEAD'", probe)
        self.assertNotIn("/api/state", PAGE)

    def test_placeholders_are_all_replaced(self):
        self.assertNotRegex(PAGE, r"__[A-Z_]+__")


class Couplings(unittest.TestCase):
    @unittest.skipUnless(os.path.isfile(BT_RUNNER), "crusader_bt not beside this package")
    def test_the_stale_node_mark_is_the_one_bt_runner_writes(self):
        mark = re.search(r"var PLANNER_MARK = '([^']+)';", PAGE).group(1)
        self.assertIn(mark, _read(BT_RUNNER))

    @unittest.skipUnless(os.path.isdir(SIM), "crusader_sim not beside this package")
    def test_the_lake_panel_can_be_framed(self):
        # The Task 1 tab frames the lake panel. A framing header added there would blank the tab with
        # no error, so the code that sends its headers is read here. (X-Frame-Age is a sensor-frame age.)
        for name in ("task1_panel.py", "lake_panel.py"):
            text = _read(os.path.join(SIM, name))
            for header in ("X-Frame-Options", "Content-Security-Policy", "frame-ancestors"):
                self.assertNotIn(header, text, "%s sends %s" % (name, header))


class Served(unittest.TestCase):
    """The real GcsServer, serving the real page, over the fake's state."""

    @classmethod
    def setUpClass(cls):
        cls.fake = fake_gcs.Fake(False, os.path.join(REPO, "crusader_sim", "config", "tuning_profiles"),
                                 os.path.join(os.path.dirname(HERE), "no_such_saved_dir"))
        cls.srv = GcsServer(gcs_page.render(200), cls.fake.snapshot, cls.fake.action)
        cls.srv.start(0, host="127.0.0.1")
        cls.base = "http://127.0.0.1:%d" % cls.srv._server.server_address[1]

    @classmethod
    def tearDownClass(cls):
        cls.srv.stop()

    def post(self, path, body):
        req = urllib.request.Request(self.base + path, json.dumps(body).encode(),
                                     {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())

    def test_page_and_state(self):
        with urllib.request.urlopen(self.base + "/", timeout=5) as r:
            self.assertIn("text/html", r.headers["Content-Type"])
            html = r.read().decode("utf-8")
        self.assertIn('data-t="task1"', html)
        with urllib.request.urlopen(self.base + "/state", timeout=5) as r:
            self.assertIn("/bt_runner_node", json.loads(r.read())["tuning"]["nodes"])

    def test_the_planner_rows_carry_the_mark_and_the_structural_ones_are_fixed(self):
        rows = {p["name"]: p for p in self.post("/params/list", {"node": "/bt_runner_node"})["params"]}
        self.assertIn("applies at the next START", rows["nav_hard_m"]["description"])
        self.assertTrue(rows["nav_hard_m"]["editable"])
        self.assertFalse(rows["nav_mode"]["editable"])
        self.assertFalse(rows["nav_hazard_rate_hz"]["editable"])

    def test_a_refused_set_comes_back_in_the_nodes_words(self):
        j = self.post("/params/set", {"node": "/bt_runner_node", "values": {"nav_gate_step_m": 0}})
        self.assertFalse(j["ok"])
        self.assertIn("must be > 0", j["message"])

    def test_profile_listing_is_served(self):
        j = self.post("/planner/profile/list", {})
        self.assertTrue(j["ok"])
        self.assertEqual({s["origin"] for s in j["sources"]}, {"shipped", "saved"})


if __name__ == "__main__":
    unittest.main()
