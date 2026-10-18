"""tools/scripts/gcs_laptop.py off-boat: the page patcher, the MAVLink strip's state, the passthrough to
a Jetson (a real GcsServer on an ephemeral port stands in for it), and the rule that this program never
transmits MAVLink.

    python -m unittest discover -s crusader_groundstation/test        (from the repo root; no ROS)

Needs pymavlink and PyYAML, which the Windows laptop has; the module skips itself where they are missing.
What this CANNOT check is the strip's rendering in a browser. The JavaScript is only syntax-checked (with
node, when node is installed); to see the strip, start the program and read the page's DOM
(get_page_text / read_page), not a screenshot.
"""
import ast
import contextlib
import http.client
import http.server
import importlib.util
import io
import json
import os
import re
import shutil
import socket
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
SCRIPTS = os.path.join(REPO, "tools", "scripts")
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, os.path.join(REPO, "crusader_groundstation"))

try:
    import gcs_laptop as gl
except ImportError as e:                       # no pymavlink / PyYAML on this machine
    raise unittest.SkipTest("gcs_laptop needs pymavlink and PyYAML: %s" % e)
import team_logos                                 # noqa: E402  (next to gcs_laptop; it imports it too)

from crusader_groundstation import gcs_page          # noqa: E402
from crusader_groundstation.gcs_server import GcsServer  # noqa: E402

mavlink = gl.mavutil.mavlink
LOCAL = "127.0.0.1"
PAGE = gcs_page.render(200).decode("utf-8")


# ---------------------------------------------------------------- helpers

def encode(name, sysid=2, compid=1, **fields):
    """A MAVLink 2 frame as the autopilot would put it on the wire."""
    cls = getattr(mavlink, "MAVLink_%s_message" % name.lower())
    values = {f: fields.get(f, 0) for f in cls.fieldnames}
    if "text" in values:
        values["text"] = str(fields.get("text", "")).encode()
    return cls(**values).pack(mavlink.MAVLink(None, srcSystem=sysid, srcComponent=compid))


def wire(name, **kw):
    """The same frame parsed back, i.e. the message object the reader hands to MavState."""
    dec = mavlink.MAVLink(None)
    dec.robust_parsing = True
    return dec.parse_char(encode(name, **kw))


def free_port():
    with socket.socket() as s:
        s.bind((LOCAL, 0))
        return s.getsockname()[1]


def read_file(path, mode="r", encoding="utf-8"):
    with open(path, mode, **({} if "b" in mode else {"encoding": encoding})) as f:
        return f.read()


def get_json(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return json.loads(r.read())


def post_json(url, body, headers=None):
    req = urllib.request.Request(url, json.dumps(body).encode(),
                                 dict({"Content-Type": "application/json"}, **(headers or {})))
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def heartbeat(**kw):
    """A heartbeat from the boat's autopilot; override any field."""
    return wire("HEARTBEAT", **dict({"type": 11, "autopilot": 3, "base_mode": 65, "custom_mode": 0}, **kw))


def new_state(**kw):
    return gl.MavState(kw.pop("sysid", 2), 7, 1200, stale_s=kw.pop("stale_s", 3.0), source="test")


def gps_frame(**kw):
    """The autopilot's GPS_RAW_INT for a healthy RTK-fixed dual-antenna fix (24 satellites, HDOP 0.60,
    0.02 m horizontal, 0.03 m vertical, yaw 123.4 deg +/- 0.3); override any field."""
    return wire("GPS_RAW_INT", **dict({"fix_type": 6, "satellites_visible": 24, "eph": 60, "h_acc": 20,
                                       "v_acc": 30, "hdg_acc": 30000, "yaw": 12340}, **kw))


def gps_snapshot(feed_at=10.0, snap_at=10.0, **kw):
    """The /mav `gps` section after one GPS_RAW_INT arrived at `feed_at`, read at `snap_at`."""
    s = new_state()
    s.feed(gps_frame(**kw), feed_at)
    return s.snapshot(snap_at)["gps"]


def ekf_frame(**kw):
    """The autopilot's EKF_STATUS_REPORT for a healthy filter: flags 63 (attitude, both horizontal and
    vertical velocity, relative and absolute horizontal position, absolute vertical position), variances
    velocity 0.05, horizontal position 0.12, vertical position 0.03, compass 0.02, terrain 0; override any."""
    return wire("EKF_STATUS_REPORT", **dict({"flags": 63, "velocity_variance": 0.05, "pos_horiz_variance": 0.12,
                                             "pos_vert_variance": 0.03, "compass_variance": 0.02,
                                             "terrain_alt_variance": 0.0}, **kw))


def ekf_snapshot(feed_at=10.0, snap_at=10.0, **kw):
    """The /mav `ekf` section after one EKF_STATUS_REPORT arrived at `feed_at`, read at `snap_at`."""
    s = new_state()
    s.feed(ekf_frame(**kw), feed_at)
    return s.snapshot(snap_at)["ekf"]


PREARM_BIT = mavlink.MAV_SYS_STATUS_PREARM_CHECK


def sys_frame(present=True, enabled=True, health=True, **kw):
    """The autopilot's SYS_STATUS with the pre-arm bit present / enabled / healthy as asked, and the battery
    unmeasured (current and remaining -1; voltage 12600 mV unless overridden)."""
    def bits(on):
        return 1 | (PREARM_BIT if on else 0)                  # the gyro bit rides along so a mask is not all-or-nothing
    return wire("SYS_STATUS", **dict({"onboard_control_sensors_present": bits(present),
                                      "onboard_control_sensors_enabled": bits(enabled),
                                      "onboard_control_sensors_health": bits(health),
                                      "voltage_battery": 12600, "current_battery": -1, "battery_remaining": -1}, **kw))


def prearm_snapshot(**kw):
    s = new_state()
    s.feed(sys_frame(**kw), 10.0)
    return s.snapshot(10.0)["prearm"]


class FakeJetson:
    """A real GcsServer with a canned snapshot: the boat's ground_station as far as the laptop sees it."""

    SNAPSHOT = {"boat": {"ok": True, "lat": 1.3, "lon": 103.8}, "fcu": {"ok": True, "armed": False},
                "nodes": [], "tabs": {"camera": {"source": None}}}

    def __init__(self):
        self.layers, self.actions = [], []
        self.server = GcsServer(b"<html>jetson page</html>", self._snapshot, self._action,
                                self._download).start(0, LOCAL)
        self.port = self.server._server.server_address[1]

    def _snapshot(self, layers):
        self.layers.append(layers)
        return self.SNAPSHOT

    def _action(self, path, payload):
        self.actions.append((path, payload))
        return {"ok": True, "message": "did " + path}

    @staticmethod
    def _download(name, fileobj):
        for i in range(5):
            fileobj.write((name.encode() + b"|") * 20000 + bytes([i]))
        return True

    def stop(self):
        self.server.stop()


class RawJetson:
    """A server that answers every request with one canned status and body (or none at all)."""

    def __init__(self, status=200, body=b"hello", swallow=False):
        outer = self
        self.status, self.body = status, body

        class H(http.server.BaseHTTPRequestHandler):
            def _answer(self):
                n = int(self.headers.get("Content-Length") or 0)
                self.rfile.read(n)
                outer.hits += 1
                if swallow:                      # read the request, then hang up with no reply
                    return
                self.send_response(outer.status)
                self.send_header("Content-Length", str(len(outer.body)))
                self.end_headers()
                self.wfile.write(outer.body)

            do_GET = do_POST = _answer

            def log_message(self, *a):
                pass

        self.hits = 0
        self.httpd = http.server.ThreadingHTTPServer((LOCAL, 0), H)
        self.port = self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True).start()

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()


def explicit(port):
    return gl.JetsonLink("%s:%d" % (LOCAL, port))


#: A Jetson that is not there. A closed LOCAL port is the realistic case but costs ~2 s per attempt on
#: Windows (it retries the SYN), so most tests use a name that fails to resolve at once; one test uses
#: the real refusal.
DEAD = ("nohost.invalid", 8090)


def dead_link():
    return gl.JetsonLink("%s:%d" % DEAD)


# ---------------------------------------------------------------- the page

class PagePatch(unittest.TestCase):
    def test_every_patch_applies_exactly_where_expected(self):
        out = gl.patch_page(PAGE)
        self.assertEqual(out.count("(window.VIEW_HOST||location.hostname)"), 2)
        self.assertEqual(out.count("location.hostname"), 2)       # nothing left unpatched, nothing new
        self.assertIn('"%s"' % gl.BANNER_TEXT, out)
        self.assertNotIn("'NO CONNECTION TO THE BOAT'", out)
        self.assertNotIn('"', gl.BANNER_TEXT)                     # it is put into a JS string literal
        self.assertEqual(out.count('<div id="mavstrip">'), 1)
        self.assertNotRegex(out, r"__[A-Z_]+__")                  # a placeholder left unreplaced

    def test_the_strip_is_a_row_of_the_app_between_the_tab_bar_and_the_panes(self):
        out = gl.patch_page(PAGE)
        self.assertLess(out.index('<div id="bar">'), out.index('<div id="mavstrip">'))
        self.assertLess(out.index('<div id="mavstrip">'), out.index("<main>"))

    def test_the_strip_script_comes_last_and_polls_mav(self):
        out = gl.patch_page(PAGE)
        self.assertTrue(out.endswith("</script></body></html>"))
        last = out.rsplit("<script>", 1)[1]
        self.assertIn("fetch('/mav'", last)
        self.assertIn("window.VIEW_HOST = j.jetson_host", last)

    def test_a_pattern_that_stops_matching_is_a_loud_failure_naming_it(self):
        for what, old, _new, _want in gl.PAGE_PATCHES:
            with self.subTest(what):
                with self.assertRaises(gl.PagePatchError) as cm:
                    gl.patch_page(PAGE.replace(old, "", 1) if PAGE.count(old) == 1
                                  else PAGE.replace(old, "REMOVED"))
                self.assertIn(repr(old), str(cm.exception))

    def test_a_new_use_of_the_pattern_is_a_failure_too(self):
        # A third viewer frame built on location.hostname would silently frame the laptop.
        with self.assertRaises(gl.PagePatchError) as cm:
            gl.patch_page(PAGE + " location.hostname")
        self.assertIn("'location.hostname' occurs 3 time(s), expected 2", str(cm.exception))

    def test_every_mismatch_is_reported_at_once(self):
        with self.assertRaises(gl.PagePatchError) as cm:
            gl.patch_page(PAGE.replace("<main>", "<section>").replace("</style>", ""))
        self.assertIn("'<main>'", str(cm.exception))
        self.assertIn("'</style>'", str(cm.exception))

    def test_the_strip_style_uses_only_the_pages_css_variables(self):
        # The ONE literal colour is the logo badge's white: dark line-art logos need a white ground in
        # both themes, and gcs_page.py (not ours to edit) has no --logo-bg to borrow. This test used to
        # forbid every literal; it now allows exactly that one, in exactly that rule.
        self.assertEqual(re.findall(r"#[0-9a-fA-F]{3,8}\b|rgb\(|hsl\(", gl.STRIP_CSS), ["#fff"])
        self.assertRegex(gl.STRIP_CSS, r"\.logos\{[^}]*background:#fff[;}]")
        for name in set(re.findall(r"var\((--[\w-]+)\)", gl.STRIP_CSS)):
            self.assertIn(name + ":", PAGE, "the strip uses %s, which gcs_page.py does not define" % name)

    def test_the_tile_colours_come_only_from_ok_warn_bad_and_both_themes_define_them(self):
        tiles = "".join(re.findall(r"#mavstrip \.mv-(?:tile|big|dot)[^{]*\{[^}]*\}", gl.STRIP_CSS))
        used = set(re.findall(r"var\((--[\w-]+)\)", tiles))
        self.assertLessEqual(used, {"--ok", "--warn", "--bad", "--dim", "--dot", "--card", "--line"})
        day = re.search(r'html\[data-theme="day"\]\{(.*?)\}', PAGE, re.S).group(1)
        night = re.search(r":root\{(.*?)\}", PAGE, re.S).group(1)
        for name in ("--ok", "--warn", "--bad", "--card", "--dot"):
            self.assertIn(name + ":", day, name + " is not redefined for day mode")
            self.assertIn(name + ":", night, name + " is not defined for night mode")

    # -- the team logos

    def test_the_team_logos_are_the_first_child_of_the_top_bar(self):
        out = gl.patch_page(PAGE)
        self.assertEqual(out.count('<img src="data:image/'), 2)
        self.assertEqual(out.count('<div class="logos">'), 1)
        self.assertNotIn("__LOGOS__", out)
        bar = out.index('<div id="bar">')
        self.assertEqual(out.index('<div class="logos">'), bar + len('<div id="bar">\n  '))
        self.assertLess(out.rindex('<img src="data:image/'), out.index('<button class="tab"'))
        self.assertLess(out.index("</div>", out.index('<div class="logos">')), out.index('<button class="tab"'))
        self.assertLess(out.index('<div class="logos">'), out.index('<div id="mavstrip">'))

    def test_the_logos_are_the_drones_two_files_with_names(self):
        out = gl.patch_page(PAGE)
        self.assertEqual([n for n, _ in team_logos.LOGOS],
                         ["Team Inspiration", "Advancing Science, Technology & Art"])
        self.assertIn('<img src="%s" alt="Team Inspiration" title="Team Inspiration">'
                      % team_logos.LOGOS[0][1], out)
        # the ampersand in the second name must not be left bare inside an attribute
        self.assertIn('alt="Advancing Science, Technology &amp; Art" title="Advancing Science, Technology &amp; Art"', out)
        self.assertTrue(team_logos.LOGOS[0][1].startswith("data:image/jpeg;base64,"))
        self.assertTrue(team_logos.LOGOS[1][1].startswith("data:image/png;base64,"))

    def test_without_team_logos_the_page_still_patches_and_just_has_none(self):
        with mock.patch.dict(sys.modules, {"team_logos": None}):          # `import team_logos` now fails
            self.assertEqual(gl.logos_html(), "")
        patch = gl.logos_patch("")
        self.assertEqual(patch[1], patch[2])                                # a no-op replacement ...
        self.assertEqual(patch[3], 1)                                       # ... that still counts its anchor
        patches = tuple(patch if p[0] == patch[0] else p for p in gl.PAGE_PATCHES)
        with mock.patch.object(gl, "PAGE_PATCHES", patches):
            out = gl.patch_page(PAGE)
        self.assertEqual(out.count('<img src="data:image/'), 0)
        self.assertNotIn('<div class="logos">', out)
        self.assertIn('<div id="mavstrip">', out)

    def test_the_logo_badge_is_white_in_both_themes_and_26px_tall(self):
        rule = re.search(r"\.logos\{[^}]*\}", gl.STRIP_CSS).group(0)
        self.assertIn("background:#fff", rule)
        self.assertNotIn("var(--logo-bg)", gl.STRIP_CSS)                    # gcs_page.py has no such variable
        self.assertRegex(gl.STRIP_CSS, r"\.logos img\{[^}]*height:26px")

    # -- the vehicle's name

    def test_the_brand_sits_right_after_the_logos_and_before_the_tabs(self):
        out = gl.patch_page(PAGE)
        self.assertEqual(out.count(gl.BRAND_HTML), 1)
        self.assertIn(">Crusader</div>", gl.BRAND_HTML)
        logos_end = out.index("</div>", out.index('<div class="logos">')) + len("</div>")
        self.assertEqual(out.index(gl.BRAND_HTML), logos_end)
        self.assertLess(out.index(gl.BRAND_HTML), out.index('<button class="tab"'))

    def test_the_brand_stays_when_the_logos_cannot_be_imported(self):
        with mock.patch.dict(sys.modules, {"team_logos": None}):
            head = gl.bar_head_html()
        self.assertEqual(head, gl.BRAND_HTML)
        patches = tuple(gl.logos_patch(head) if p[1] == gl.BAR_OPEN else p for p in gl.PAGE_PATCHES)
        with mock.patch.object(gl, "PAGE_PATCHES", patches):
            out = gl.patch_page(PAGE)
        self.assertNotIn('<div class="logos">', out)
        self.assertEqual(out.count(gl.BRAND_HTML), 1)

    def test_the_brand_is_styled_like_the_drones_in_the_pages_own_variables(self):
        rule = re.search(r"#bar \.brand\{[^}]*\}", gl.STRIP_CSS).group(0)
        for want in ("font-size:17px", "font-weight:700", "color:var(--strong)"):    # the drone's .brand, gcs_page.py:89
            self.assertIn(want, rule)

    # -- the hook that puts the GPS panel on the Telemetry tab

    def test_the_telemetry_hook_replaces_one_call_and_goes_in_before_the_strip_script(self):
        anchor, replacement = gl.TEL_HOOK
        self.assertEqual(PAGE.count(anchor), 1)
        out = gl.patch_page(PAGE)
        self.assertEqual((out.count(anchor), out.count(replacement)), (0, 1))
        self.assertLess(out.index(replacement), out.rindex("<script>"))      # in the page's script, not the strip's
        self.assertNotIn(anchor, gl.STRIP_JS)                               # or it would be replaced a second time
        names = [p[0] for p in gl.PAGE_PATCHES]
        self.assertLess(names.index("telemetry GPS panel hook"), names.index("strip script"))

    def test_no_replacement_contains_another_patchs_anchor(self):
        for i, (what, _old, new, _want) in enumerate(gl.PAGE_PATCHES):
            for j, (other, old, _new, _n) in enumerate(gl.PAGE_PATCHES):
                if i != j:
                    self.assertNotIn(old, new, "%r would be applied to the text that %r inserts" % (other, what))

    @unittest.skipUnless(shutil.which("node"), "node is not installed")
    def test_the_hook_runs_renderTel_first_and_survives_a_panel_that_throws(self):
        hook = gl.TEL_HOOK[1]
        code = ("var order = [], tab = 'tel', window = {}; function renderTel(){ order.push('tel'); } "
                "console.error = function(){ order.push('logged'); }; "
                + hook + " order.push('|'); "                                # no window.mavTel yet: just renderTel
                "window.mavTel = function(){ order.push('panel'); throw new Error('boom'); }; "
                + hook + " console.log(JSON.stringify(order));")
        r = subprocess.run(["node", "-e", code], capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(json.loads(r.stdout), ["tel", "|", "tel", "panel", "logged"])

    def test_autopilot_text_is_never_put_in_through_innerhtml(self):
        self.assertNotIn("innerHTML", gl.STRIP_JS)
        self.assertNotIn("document.write", gl.STRIP_JS)

    @unittest.skipUnless(shutil.which("node"), "node is not installed")
    def test_the_javascript_parses(self):
        out = gl.patch_page(PAGE)
        with tempfile.TemporaryDirectory() as d:
            for i, body in enumerate(re.findall(r"<script>(.*?)</script>", out, re.S)):
                path = os.path.join(d, "s%d.js" % i)
                with open(path, "w", encoding="utf-8") as f:
                    f.write(body)
                r = subprocess.run(["node", "--check", path], capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, r.stderr)


# ---------------------------------------------------------------- crusader_params.yaml

class ParamsFile(unittest.TestCase):
    def write(self, text):
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        path = os.path.join(d.name, "p.yaml")
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return path

    GOOD = ("telemetry_bridge:\n  ros__parameters:\n    estop_channel: 7\n    estop_threshold: 1200\n"
            "pixhawk_led_status_node:\n  ros__parameters:\n    estop_channel: 7\n    estop_threshold: 1200\n")

    def test_the_boats_yaml_says_sb_on_channel_7_below_1200(self):
        # If the switch ever moves, the strip's docs and the skills need the same edit.
        p = gl.load_params()
        self.assertEqual((p.estop_channel, p.estop_threshold, p.poll_ms), (7, 1200, 200))

    def test_poll_period_comes_from_the_ground_station_block_else_200(self):
        self.assertEqual(gl.load_params(self.write(self.GOOD)).poll_ms, 200)
        p = gl.load_params(self.write(self.GOOD + "ground_station:\n  ros__parameters:\n    poll_period_s: 0.5\n"))
        self.assertEqual(p.poll_ms, 500)

    def test_a_missing_or_malformed_estop_is_an_error_not_a_default(self):
        bad = {
            "no file": os.path.join(tempfile.gettempdir(), "no_such_crusader_params.yaml"),
            "no bridge block": self.write("ground_station:\n  ros__parameters:\n    port: 8090\n"),
            "no channel": self.write("telemetry_bridge:\n  ros__parameters:\n    estop_threshold: 1200\n"),
            "channel is a string": self.write(self.GOOD.replace("estop_channel: 7", "estop_channel: '7'", 1)),
            "channel out of range": self.write(self.GOOD.replace("estop_channel: 7", "estop_channel: 19", 1)),
            "threshold is a bool": self.write(self.GOOD.replace("estop_threshold: 1200", "estop_threshold: true", 1)),
            "bad poll period": self.write(self.GOOD + "ground_station:\n  ros__parameters:\n    poll_period_s: fast\n"),
            "not a mapping": self.write("- just\n- a list\n"),
        }
        for what, path in bad.items():
            with self.subTest(what), self.assertRaises(gl.ConfigError):
                gl.load_params(path)

    def test_the_led_node_must_agree_with_the_bridge(self):
        led_block = "pixhawk_led_status_node:\n  ros__parameters:\n    estop_channel: "
        led_on_8 = self.GOOD.replace(led_block + "7", led_block + "8")
        self.assertNotEqual(led_on_8, self.GOOD)
        with self.assertRaises(gl.ConfigError) as cm:
            gl.load_params(self.write(led_on_8))
        self.assertIn("pixhawk_led_status_node", str(cm.exception))

    def test_the_led_node_block_is_optional(self):
        bridge_only = self.GOOD.split("pixhawk_led_status_node:")[0]
        self.assertEqual(gl.load_params(self.write(bridge_only)).estop_channel, 7)


# ---------------------------------------------------------------- what the autopilot said

class Heartbeat(unittest.TestCase):
    def test_the_boats_modes_have_names(self):
        # ArduRover 4.6.3, FRAME_CLASS 2: the HEARTBEAT type is MAV_TYPE_SURFACE_BOAT.
        for mav_type in (mavlink.MAV_TYPE_SURFACE_BOAT, mavlink.MAV_TYPE_GROUND_ROVER):
            for custom, name in ((0, "MANUAL"), (4, "HOLD"), (10, "AUTO"), (11, "RTL"), (15, "GUIDED")):
                with self.subTest(type=mav_type, mode=name):
                    s = new_state()
                    s.feed(heartbeat(type=mav_type, custom_mode=custom), 10.0)
                    hb = s.snapshot(10.0)["hb"]
                    self.assertEqual((hb["mode"], hb["armed"], hb["type"]), (name, False, mav_type))

    def test_armed_follows_the_safety_armed_bit(self):
        s = new_state()
        s.feed(heartbeat(base_mode=65 | 128), 10.0)
        self.assertTrue(s.snapshot(10.0)["hb"]["armed"])

    def test_only_the_autopilot_of_the_chosen_system_is_the_heartbeat(self):
        s = new_state()
        base = dict(type=11, autopilot=3, base_mode=65, custom_mode=10)
        s.feed(wire("HEARTBEAT", sysid=2, compid=191, **base), 10.0)          # companion computer
        s.feed(wire("HEARTBEAT", sysid=1, compid=1, **base), 10.0)            # another vehicle
        s.feed(wire("HEARTBEAT", sysid=2, compid=1, type=mavlink.MAV_TYPE_GCS,
                    autopilot=8, base_mode=0, custom_mode=0), 10.0)           # a ground station
        hb = s.snapshot(10.0)["hb"]
        self.assertFalse(hb["ok"])
        self.assertIsNone(hb["age"])
        self.assertIsNone(hb["mode"])


class EStop(unittest.TestCase):
    def rc(self, **kw):
        s = new_state()
        s.feed(wire("RC_CHANNELS", **dict({"chancount": 16}, **kw)), 10.0)
        return s.snapshot(10.0)["rc"]

    def test_run_engaged_and_lost(self):
        run = self.rc(chan7_raw=1999)
        self.assertEqual((run["ok"], run["estop_pwm"], run["estop_engaged"]), (True, 1999, False))
        engaged = self.rc(chan7_raw=1000)
        self.assertEqual((engaged["estop_pwm"], engaged["estop_engaged"]), (1000, True))
        lost = self.rc(chan7_raw=0)                    # a lost RC reads 0, which is below the threshold
        self.assertEqual((lost["estop_pwm"], lost["estop_engaged"]), (0, True))

    def test_the_threshold_is_below_not_at_or_below(self):
        self.assertFalse(self.rc(chan7_raw=1200)["estop_engaged"])
        self.assertTrue(self.rc(chan7_raw=1199)["estop_engaged"])

    def test_a_channel_the_autopilot_does_not_report_is_unknown_not_run(self):
        rc = self.rc(chan7_raw=65535)
        self.assertEqual((rc["ok"], rc["estop_pwm"], rc["estop_engaged"]), (True, None, None))

    def test_no_rc_channels_at_all_is_rc_lost(self):
        rc = self.rc(chancount=0, chan7_raw=1999)
        self.assertTrue(rc["estop_engaged"])

    def test_the_channel_comes_from_configuration_and_other_channels_do_not_count(self):
        s = gl.MavState(2, 8, 1200)
        s.feed(wire("RC_CHANNELS", chancount=16, chan7_raw=1000, chan8_raw=1800), 10.0)
        rc = s.snapshot(10.0)["rc"]
        self.assertEqual((rc["estop_channel"], rc["estop_threshold"], rc["estop_engaged"]), (8, 1200, False))

    def test_rssi_255_is_unknown(self):
        self.assertIsNone(self.rc(chan7_raw=1999, rssi=255)["rssi"])
        self.assertEqual(self.rc(chan7_raw=1999, rssi=180)["rssi"], 180)


class Battery(unittest.TestCase):
    def batt(self, **kw):
        s = new_state()
        s.feed(wire("SYS_STATUS", **dict({"current_battery": -1, "battery_remaining": -1}, **kw)), 10.0)
        return s.snapshot(10.0)["batt"]

    def test_zero_and_65535_volts_are_no_sensor_never_zero_volts(self):
        for raw in (0, 65535):
            with self.subTest(raw=raw):
                b = self.batt(voltage_battery=raw)
                self.assertTrue(b["ok"])                      # the message is fresh ...
                self.assertIsNone(b["voltage"])               # ... and says there is no number
                self.assertIsNone(b["current"])
                self.assertIsNone(b["remaining"])

    def test_a_real_reading_is_scaled(self):
        b = self.batt(voltage_battery=12600, current_battery=1520, battery_remaining=87)
        self.assertEqual((b["voltage"], b["current"], b["remaining"]), (12.6, 15.2, 87))


class Gps(unittest.TestCase):
    def gps(self, **kw):
        s = new_state()
        s.feed(wire("GPS_RAW_INT", **kw), 10.0)
        return s.snapshot(10.0)["gps"]

    def test_gps_yaw_states(self):
        self.assertEqual(self.gps(yaw=65535)["yaw_state"], "unavailable")
        self.assertEqual(self.gps(yaw=0)["yaw_state"], "not_provided")
        ok = self.gps(yaw=9050)
        self.assertEqual((ok["yaw_state"], ok["yaw_deg"]), ("ok", 90.5))
        north = self.gps(yaw=36000)
        self.assertEqual((north["yaw_state"], north["yaw_deg"]), ("ok", 0.0))
        self.assertIsNone(self.gps(yaw=50000)["yaw_state"])
        self.assertIsNone(self.gps(yaw=65535)["yaw_deg"])

    def test_fix_satellites_and_accuracy(self):
        g = self.gps(fix_type=6, satellites_visible=22, h_acc=25, yaw=65535)
        self.assertEqual((g["fix_type"], g["fix_name"], g["sats"], g["h_acc_m"]), (6, "RTK FIXED", 22, 0.025))
        blank = self.gps(fix_type=0, satellites_visible=255, h_acc=0)
        self.assertEqual((blank["fix_name"], blank["sats"], blank["h_acc_m"]), ("NO GPS", None, None))


class GpsSentinels(unittest.TestCase):
    """Every GPS_RAW_INT value that means "unknown" is a blank, never a number. The dialect's own words
    (common.xml): eph "If unknown, set to: UINT16_MAX", satellites_visible "If unknown, set to UINT8_MAX";
    h_acc, v_acc and hdg_acc (extension fields) name no sentinel, and an unfilled one is 0."""

    def test_a_full_report_is_scaled_to_si_units(self):
        g = gps_snapshot()
        self.assertEqual((g["fix_type"], g["fix_name"], g["sats"]), (6, "RTK FIXED", 24))
        for key, want in (("hdop", 0.6), ("h_acc_m", 0.02), ("v_acc_m", 0.03), ("hdg_acc_deg", 0.3), ("yaw_deg", 123.4)):
            self.assertAlmostEqual(g[key], want, places=6, msg=key)

    def test_eph_is_blank_at_65535_and_at_0_and_scaled_otherwise(self):
        self.assertIsNone(gps_snapshot(eph=65535)["hdop"])
        self.assertIsNone(gps_snapshot(eph=0)["hdop"])                 # HDOP 0.00 cannot be measured
        self.assertEqual(gps_snapshot(eph=120)["hdop"], 1.2)
        self.assertEqual(gps_snapshot(eph=65534)["hdop"], 655.34)      # one below the sentinel is a number

    def test_satellites_are_blank_at_255_but_zero_is_a_real_count(self):
        self.assertIsNone(gps_snapshot(satellites_visible=255)["sats"])
        self.assertEqual(gps_snapshot(satellites_visible=0)["sats"], 0)
        self.assertIsInstance(gps_snapshot(satellites_visible=9)["sats"], int)

    def test_accuracies_are_blank_at_zero(self):
        g = gps_snapshot(h_acc=0, v_acc=0, hdg_acc=0)
        self.assertEqual((g["h_acc_m"], g["v_acc_m"], g["hdg_acc_deg"]), (None, None, None))
        g = gps_snapshot(h_acc=1, v_acc=1, hdg_acc=1)
        self.assertEqual((g["h_acc_m"], g["v_acc_m"]), (0.001, 0.001))
        self.assertAlmostEqual(g["hdg_acc_deg"], 0.00001, places=9)      # degE5: 1 is a hundred-thousandth of a degree
        self.assertAlmostEqual(gps_snapshot(hdg_acc=100000)["hdg_acc_deg"], 1.0)


#: The drone GUI's verdict, loaded from the monorepo when it is checked out beside the boat. These
#: tests are the guard against the two drifting apart; they skip, saying so, where it is absent.
DRONE_PREFLIGHT = os.path.normpath(os.path.join(
    REPO, "..", "..", "Drone", "rx26_uav", "uav_groundstation", "uav_groundstation", "preflight_core.py"))


class GpsQuality(unittest.TestCase):
    """gps.quality, boundary by boundary. Every comparison is strict, as in the drone's preflight_core."""

    def q(self, fix=6, sats=24, hdop=0.6, acc=0.02):
        return gl.gps_quality(fix, sats, hdop, acc)

    def states(self, cases):
        for label, kw, want in cases:
            with self.subTest(label):
                self.assertEqual(self.q(**kw)[0], want)

    def test_the_constants_are_the_drones(self):
        self.assertEqual((gl.GPS_MIN_FIX, gl.GPS_MIN_SATS, gl.GPS_MAX_HDOP, gl.GPS_GOOD_ACC_M, gl.GPS_POOR_ACC_M,
                          gl.GPS_WARN_SATS, gl.GPS_POOR_SATS, gl.GPS_WARN_HDOP, gl.GPS_POOR_HDOP, gl.RTK_FIXED),
                         (3, 10, 1.2, 0.5, 1.5, 9, 6, 2.0, 3.0, 6))

    def test_no_data_is_unknown_and_a_missing_3d_fix_is_bad(self):
        self.assertEqual(self.q(fix=None), ("unknown", ["no GPS data"]))
        self.states([("fix %d" % f, {"fix": f}, "bad") for f in (0, 1, 2)] + [("fix 3", {"fix": 3}, "ok")])
        self.assertEqual(self.q(fix=2), ("bad", ["no 3D fix"]))

    def test_accuracy_boundaries(self):
        self.states([("0.5 m is good", {"acc": 0.5}, "ok"), ("just over 0.5 m", {"acc": 0.51}, "warn"),
                     ("1.5 m is still only degraded", {"acc": 1.5}, "warn"), ("just over 1.5 m", {"acc": 1.51}, "bad")])

    def test_satellite_boundaries_with_an_accuracy_figure(self):
        self.states([("9 is enough", {"fix": 3, "sats": 9}, "ok"), ("8 warns", {"fix": 3, "sats": 8}, "warn"),
                     ("6 still only warns", {"fix": 3, "sats": 6}, "warn"), ("5 is poor", {"fix": 3, "sats": 5}, "bad"),
                     ("an unknown count never vetoes", {"fix": 3, "sats": None}, "ok")])

    def test_rtk_fixed_is_exempt_from_the_satellite_warning_and_nothing_else(self):
        self.states([("RTK FIXED on 6", {"fix": 6, "sats": 6}, "ok"), ("RTK FIXED on 5 is still poor", {"fix": 6, "sats": 5}, "bad"),
                     ("RTK FLOAT on 8 warns", {"fix": 5, "sats": 8}, "warn"), ("RTK FLOAT on 9", {"fix": 5, "sats": 9}, "ok"),
                     ("RTK FIXED, bad HDOP", {"fix": 6, "hdop": 3.5}, "bad"), ("RTK FIXED, 0.6 m", {"fix": 6, "acc": 0.6}, "warn")])

    def test_hdop_boundaries_with_an_accuracy_figure(self):
        self.states([("2.0 is fine", {"hdop": 2.0}, "ok"), ("just over 2.0", {"hdop": 2.01}, "warn"),
                     ("3.0 still warns", {"hdop": 3.0}, "warn"), ("just over 3.0", {"hdop": 3.01}, "bad"),
                     ("an unknown HDOP never vetoes", {"hdop": None}, "ok")])

    def test_without_an_accuracy_figure_only_satellites_and_hdop_judge_and_never_to_bad(self):
        none = {"acc": None, "fix": 3}
        self.states([("10 sats", dict(none, sats=10), "ok"), ("9 sats", dict(none, sats=9), "warn"),
                     ("2 sats is a warning, not poor", dict(none, sats=2), "warn"),
                     ("HDOP 1.2", dict(none, hdop=1.2), "ok"), ("HDOP 1.21", dict(none, hdop=1.21), "warn"),
                     ("HDOP 9 is a warning, not poor", dict(none, hdop=9.0), "warn"),
                     ("nothing known but a 3D fix", dict(none, sats=None, hdop=None), "ok")])

    def test_a_zero_accuracy_counts_as_no_figure_as_in_the_drone(self):
        self.assertEqual(self.q(acc=0.0, sats=9)[0], "warn")            # judged by the no-accuracy rule: 9 < 10
        self.assertEqual(self.q(acc=0.0, sats=10)[0], "ok")
        self.assertEqual(self.q(acc=0.0, sats=24, hdop=0.6), ("ok", ["accuracy not reported"]))

    def test_good_accuracy_outranks_few_satellites_and_a_high_hdop_up_to_their_own_limits(self):
        # The drone's lesson of 2 Oct: 0.10 m on 11 satellites at HDOP 1.4 is a good fix, not "Degraded".
        self.assertEqual(self.q(fix=3, sats=11, hdop=1.4, acc=0.10), ("ok", []))

    def test_the_reasons_name_what_tripped_and_a_bad_lists_only_the_bad_ones(self):
        self.assertEqual(self.q(acc=2.0, sats=8), ("bad", ["accuracy 2.00 m > 1.5 m"]))
        self.assertEqual(self.q(fix=3, acc=0.8, sats=8, hdop=2.5),
                         ("warn", ["accuracy 0.80 m > 0.5 m", "only 8 satellites (< 9)", "HDOP 2.5 > 2.0"]))
        self.assertEqual(self.q(fix=3, sats=5), ("bad", ["only 5 satellites (< 6)"]))
        self.assertEqual(self.q(), ("ok", []))
        self.assertEqual(self.q(acc=None, sats=9, hdop=1.5),
                         ("warn", ["only 9 satellites (< 10)", "HDOP 1.5 > 1.2", "accuracy not reported"]))

    @unittest.skipUnless(os.path.isfile(DRONE_PREFLIGHT), "the drone's preflight_core.py is not checked out beside the boat")
    def test_it_agrees_with_the_drones_verdict_everywhere(self):
        spec = importlib.util.spec_from_file_location("drone_preflight_core", DRONE_PREFLIGHT)
        drone = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(drone)
        checked = 0
        for fix in (0, 1, 2, 3, 4, 5, 6):
            for sats in (None, 0, 3, 5, 6, 7, 8, 9, 10, 11, 24):
                for hdop in (None, 0.5, 1.2, 1.21, 1.3, 2.0, 2.01, 3.0, 3.01, 9.0):
                    for acc in (None, 0.02, 0.5, 0.51, 1.5, 1.51, 5.0):
                        inp = {"gps": {"fix_type": fix, "satellites": 255 if sats is None else sats,
                                       "hdop": hdop, "h_acc_m": acc}}
                        want = drone.gps(inp)["state"]
                        got = gl.gps_quality(fix, sats, hdop, acc)[0]
                        self.assertEqual(got, want, "fix=%s sats=%s hdop=%s acc=%s" % (fix, sats, hdop, acc))
                        checked += 1
        self.assertEqual(checked, 7 * 11 * 10 * 7)


class HeadingQuality(unittest.TestCase):
    """gps.heading_quality: the boat has no compass, so GPS yaw is the heading and no yaw is no heading."""

    def test_the_accuracy_limits_are_named_first_guesses(self):
        self.assertEqual((gl.HEADING_WARN_ACC_DEG, gl.HEADING_BAD_ACC_DEG), (1.0, 3.0))

    def test_no_yaw_is_bad_in_both_forms_and_says_there_is_no_compass(self):
        for state in ("unavailable", "not_provided"):
            with self.subTest(state):
                quality, reasons = gl.heading_quality(state, None)
                self.assertEqual(quality, "bad")
                self.assertTrue(any("no compass" in r for r in reasons), reasons)
        self.assertEqual(gl.heading_quality("unavailable", 0.3)[1], ["no GPS heading — the boat has no compass"])

    def test_the_accuracy_boundaries_when_yaw_is_there(self):
        for acc, want in ((0.3, "ok"), (1.0, "ok"), (1.01, "warn"), (3.0, "warn"), (3.01, "bad")):
            with self.subTest(acc):
                self.assertEqual(gl.heading_quality("ok", acc)[0], want)
        self.assertEqual(gl.heading_quality("ok", 0.3), ("ok", []))
        self.assertEqual(gl.heading_quality("ok", 2.0), ("warn", ["heading accuracy 2.0° > 1.0°"]))
        self.assertEqual(gl.heading_quality("ok", 5.0), ("bad", ["heading accuracy 5.0° > 3.0°"]))

    def test_yaw_with_no_reported_accuracy_is_ok_and_says_so(self):
        self.assertEqual(gl.heading_quality("ok", None), ("ok", ["accuracy not reported"]))

    def test_an_out_of_range_yaw_value_is_unknown(self):
        self.assertEqual(gl.heading_quality(None, None)[0], "unknown")


class GpsVerdictsOnTheWire(unittest.TestCase):
    """The verdicts as /mav serves them: from real frames, inside `gps`, and never stale."""

    VERDICT_KEYS = {"quality", "quality_reasons", "heading_quality", "heading_reasons"}

    def test_a_healthy_rtk_fix_with_yaw_is_good_and_good(self):
        g = gps_snapshot()
        self.assertEqual((g["quality"], g["quality_reasons"], g["heading_quality"], g["heading_reasons"]),
                         ("ok", [], "ok", []))

    def test_the_two_verdicts_are_independent(self):
        g = gps_snapshot(yaw=65535)                    # a fine fix whose dual-antenna yaw has dropped out
        self.assertEqual((g["quality"], g["heading_quality"]), ("ok", "bad"))
        g = gps_snapshot(fix_type=2)                   # no 3D fix, though yaw is still reported
        self.assertEqual((g["quality"], g["heading_quality"]), ("bad", "ok"))
        g = gps_snapshot(hdg_acc=0)
        self.assertEqual((g["heading_quality"], g["heading_reasons"]), ("ok", ["accuracy not reported"]))

    def test_an_unreported_accuracy_does_not_turn_a_fix_bad(self):
        g = gps_snapshot(fix_type=3, h_acc=0, v_acc=0, eph=65535, satellites_visible=255)
        self.assertEqual((g["quality"], g["quality_reasons"]), ("ok", ["accuracy not reported"]))

    def test_the_verdict_keys_are_inside_gps_and_the_rest_of_the_section_is_what_it_was(self):
        g = gps_snapshot()
        self.assertEqual(set(g), {"ok", "age", "fix_type", "fix_name", "sats", "hdop", "h_acc_m", "v_acc_m",
                                  "hdg_acc_deg", "yaw_state", "yaw_deg", "lat", "lon", "alt_ellipsoid_m"}
                         | self.VERDICT_KEYS)
        snap = new_state().snapshot(1.0)
        self.assertEqual(set(snap), {"link", "hb", "pos", "gps", "rc", "batt", "ekf", "prearm", "statustext"})

    def test_a_stale_section_is_unknown_with_every_value_null_and_its_age_kept(self):
        s = new_state(stale_s=3.0)
        s.feed(gps_frame(), 100.0)
        self.assertEqual(s.snapshot(102.9)["gps"]["quality"], "ok")
        g = s.snapshot(103.5)["gps"]
        self.assertEqual((g["ok"], g["age"]), (False, 3.5))
        self.assertEqual((g["quality"], g["heading_quality"]), ("unknown", "unknown"))
        self.assertEqual((g["quality_reasons"], g["heading_reasons"]), (["GPS data is stale"],) * 2)
        values = [v for k, v in g.items() if k not in self.VERDICT_KEYS | {"ok", "age"}]
        self.assertEqual(values, [None] * len(gl.SECTIONS["gps"]))

    def test_never_heard_is_unknown_and_says_so(self):
        g = new_state().snapshot(50.0)["gps"]
        self.assertEqual((g["ok"], g["age"], g["quality"], g["heading_quality"]), (False, None, "unknown", "unknown"))
        self.assertEqual(g["quality_reasons"], ["no GPS data yet"])

    def test_other_traffic_does_not_keep_the_gps_verdict_alive(self):
        s = new_state(stale_s=3.0)
        s.feed(gps_frame(), 100.0)
        s.feed(heartbeat(), 104.0)                     # the link is up, the GPS message is not arriving
        snap = s.snapshot(104.5)
        self.assertTrue(snap["hb"]["ok"])
        self.assertEqual(snap["gps"]["quality"], "unknown")

    def test_the_verdicts_survive_json(self):
        g = json.loads(json.dumps(gps_snapshot(satellites_visible=8, h_acc=800)))
        self.assertEqual((g["quality"], g["quality_reasons"]), ("warn", ["accuracy 0.80 m > 0.5 m"]))


class GpsPosition(unittest.TestCase):
    """GPS_RAW_INT's own position and ellipsoid height, for the Telemetry tab's panel. The autopilot's
    "unknown" is 0 here too (lat / lon 0, 0; alt_ellipsoid 0), and below a fix the message may simply repeat
    the last position it had, so those are blanks as well."""

    def test_lat_lon_and_the_ellipsoid_height_are_scaled(self):
        g = gps_snapshot(lat=13_000_000, lon=1_038_000_000, alt_ellipsoid=31250)
        self.assertEqual((g["lat"], g["lon"], g["alt_ellipsoid_m"]), (1.3, 103.8, 31.25))

    def test_negative_coordinates_and_a_negative_height_are_real_values(self):
        g = gps_snapshot(lat=-338_123_456, lon=-1_000_000_001, alt_ellipsoid=-5000)
        self.assertAlmostEqual(g["lat"], -33.8123456, places=9)
        self.assertAlmostEqual(g["lon"], -100.0000001, places=9)
        self.assertEqual(g["alt_ellipsoid_m"], -5.0)

    def test_zero_zero_is_no_position_and_zero_is_no_ellipsoid_height(self):
        g = gps_snapshot(lat=0, lon=0, alt_ellipsoid=0)
        self.assertEqual((g["lat"], g["lon"], g["alt_ellipsoid_m"]), (None, None, None))
        self.assertEqual(gps_snapshot(lat=0, lon=1)["lon"], 1e-7)         # only the pair 0, 0 is "no position"

    def test_below_a_2d_fix_the_position_is_blank_whatever_the_message_repeats(self):
        for fix in (0, 1):
            g = gps_snapshot(fix_type=fix, lat=13_000_000, lon=1_038_000_000, alt_ellipsoid=31250)
            self.assertEqual((g["lat"], g["lon"], g["alt_ellipsoid_m"]), (None, None, None), fix)

    def test_a_2d_fix_has_a_position_but_only_a_3d_fix_has_a_height(self):
        g = gps_snapshot(fix_type=2, lat=13_000_000, lon=1_038_000_000, alt_ellipsoid=31250)
        self.assertEqual((g["lat"], g["alt_ellipsoid_m"]), (1.3, None))
        g = gps_snapshot(fix_type=3, lat=13_000_000, lon=1_038_000_000, alt_ellipsoid=31250)
        self.assertEqual((g["lat"], g["alt_ellipsoid_m"]), (1.3, 31.25))

    def test_the_position_section_is_unchanged_by_the_shared_helper(self):
        s = new_state()
        s.feed(wire("GLOBAL_POSITION_INT", lat=13_000_000, lon=1_038_000_000), 10.0)
        pos = s.snapshot(10.0)["pos"]
        self.assertEqual((pos["lat"], pos["lon"]), (1.3, 103.8))


class FeedStoresSections(unittest.TestCase):
    """One message may fill more than one section, through MavState._store (not by squeezing one into another)."""

    def test_a_handler_may_return_nothing_one_section_or_a_list_of_them(self):
        s = new_state()
        s._store(None, 5.0)
        s._store({"section": "hb", "mode": "AUTO", "armed": False, "type": 11}, 5.0)
        s._store([{"section": "batt", "voltage": 1.5, "current": None, "remaining": None},
                  {"section": "rc", "estop_pwm": 1500, "estop_engaged": False, "rssi": None}], 6.0)
        snap = s.snapshot(6.0)
        self.assertEqual((snap["hb"]["mode"], snap["hb"]["age"]), ("AUTO", 1.0))
        self.assertEqual((snap["batt"]["voltage"], snap["batt"]["age"]), (1.5, 0.0))
        self.assertEqual((snap["rc"]["estop_pwm"], snap["rc"]["age"]), (1500, 0.0))
        self.assertFalse(snap["gps"]["ok"])

    def test_the_values_handed_in_are_not_consumed(self):
        values = {"section": "hb", "mode": "AUTO", "armed": False, "type": 11}
        new_state()._store(values, 5.0)
        self.assertEqual(values["section"], "hb")

    def test_one_sys_status_fills_the_battery_and_the_pre_arm_state_with_the_same_age(self):
        s = new_state()
        s.feed(sys_frame(), 10.0)
        snap = s.snapshot(11.5)
        self.assertEqual((snap["batt"]["voltage"], snap["batt"]["age"]), (12.6, 1.5))
        self.assertEqual((snap["prearm"]["state"], snap["prearm"]["age"]), ("pass", 1.5))
        s.feed(heartbeat(), 13.0)                              # other traffic keeps neither alive
        snap = s.snapshot(14.0)
        self.assertEqual((snap["batt"]["ok"], snap["prearm"]["ok"]), (False, False))


class LinkLag(unittest.TestCase):
    """link.lag_ms: the drone's gcs_radio.py lag (autopilot boot clock against this laptop's), plus the reboot fix."""

    @staticmethod
    def frames(state, n=12, start=100.0, boot0=50_000, step=0.25, late=0.0, first=0):
        """`n` GLOBAL_POSITION_INT frames `step` s apart, each stamped with the autopilot's boot clock (ms) and
        arriving `late` s after that clock says it was sampled. Returns the last arrival time."""
        for i in range(first, first + n):
            state.feed(wire("GLOBAL_POSITION_INT", time_boot_ms=boot0 + int(i * step * 1000)),
                       start + i * step + late)
        return start + (first + n - 1) * step + late

    def test_it_is_blank_until_the_floor_has_been_measured(self):
        for n in range(1, gl.LAG_MIN_SAMPLES + 1):
            s = new_state()
            end = self.frames(s, n=n)
            self.assertEqual(s.snapshot(end)["link"]["lag_ms"], 0 if n == gl.LAG_MIN_SAMPLES else None, n)

    def test_a_steady_link_reads_zero_whatever_the_clock_offset(self):
        for start in (100.0, 123456.789):                     # the two clocks have nothing to do with each other
            s = new_state()
            end = self.frames(s, n=30, start=start, boot0=7_000_000)
            self.assertEqual(s.snapshot(end)["link"]["lag_ms"], 0)

    def test_a_late_frame_shows_its_delay_over_the_best_the_link_has_done(self):
        s = new_state()
        self.frames(s, n=12)
        s.feed(wire("GLOBAL_POSITION_INT", time_boot_ms=50_000 + 12 * 250), 100.0 + 12 * 0.25 + 0.4)
        self.assertEqual(s.snapshot(103.4)["link"]["lag_ms"], 400)

    def test_the_floor_follows_a_link_that_gets_better(self):
        s = new_state()
        self.frames(s, n=12, late=0.5)                         # 0.5 s of delay from the start: that is the floor
        end = self.frames(s, n=12, first=12, late=0.1)         # then it improves by 0.4 s
        self.assertEqual(s.snapshot(end)["link"]["lag_ms"], 0)
        s.feed(wire("GLOBAL_POSITION_INT", time_boot_ms=50_000 + 24 * 250), 100.0 + 24 * 0.25 + 0.3)
        self.assertEqual(s.snapshot(106.3)["link"]["lag_ms"], 200)          # 0.3 s now against a 0.1 s floor

    def test_it_is_blank_when_the_stamps_stop_even_though_the_link_is_up(self):
        s = new_state(stale_s=3.0)
        end = self.frames(s, n=12)
        s.feed(heartbeat(), end + 2.0)                         # HEARTBEAT has no boot clock
        link = s.snapshot(end + 2.5)["link"]
        self.assertEqual((link["ok"], link["lag_ms"]), (True, 0))
        link = s.snapshot(end + 4.0)["link"]                   # the last stamp is now 4 s old, the heartbeat 2 s
        self.assertEqual((link["ok"], link["lag_ms"]), (True, None))

    def test_it_is_blank_on_a_silent_link(self):
        s = new_state()
        end = self.frames(s, n=12)
        link = s.snapshot(end + 10.0)["link"]
        self.assertEqual((link["ok"], link["fps"], link["lag_ms"]), (False, None, None))

    def test_only_the_autopilot_sets_the_clock(self):
        s = new_state()
        self.frames(s, n=12)
        for who in (dict(compid=191), dict(sysid=1)):          # a companion computer, and another vehicle
            s.feed(wire("GLOBAL_POSITION_INT", time_boot_ms=1, **who), 103.0)
        s.feed(wire("GLOBAL_POSITION_INT", time_boot_ms=50_000 + 12 * 250), 103.0)
        self.assertEqual(s.snapshot(103.0)["link"]["lag_ms"], 0)

    def test_a_zero_stamp_is_not_a_stamp(self):
        s = new_state()
        for i in range(30):
            s.feed(wire("GLOBAL_POSITION_INT", lat=13_000_000, lon=1_038_000_000), 100.0 + i * 0.25)   # time_boot_ms 0
        self.assertIsNone(s.snapshot(108.0)["link"]["lag_ms"])

    def test_the_stamp_comes_from_any_message_the_autopilot_sends_not_just_positions(self):
        s = new_state()
        for i in range(gl.LAG_MIN_SAMPLES):
            s.feed(wire("RC_CHANNELS", time_boot_ms=9_000 + i * 100, chancount=16, chan7_raw=1999), 50.0 + i * 0.1)
        self.assertEqual(s.snapshot(50.9)["link"]["lag_ms"], 0)

    def test_a_reboot_starts_the_floor_again_where_the_drones_never_would(self):
        def run():
            s = new_state()
            end = self.frames(s, n=40, boot0=3_600_000)        # an hour of uptime, then the autopilot reboots ...
            # ... its boot clock restarts near zero while this laptop's goes on (the offset jumps up by an hour)
            for i in range(12):
                s.feed(wire("GLOBAL_POSITION_INT", time_boot_ms=500 + i * 250), end + 20.0 + i * 0.25)
            return s.snapshot(end + 20.0 + 11 * 0.25)["link"]["lag_ms"]
        self.assertEqual(run(), 0)
        with mock.patch.object(gl, "LAG_BACKWARDS_MS", 10 ** 12):    # the drone's behaviour: a floor that only goes down
            self.assertGreater(run(), 3_000_000)                     # an hour of "lag", for the rest of the session

    def test_a_reordered_frame_is_a_delay_not_a_reboot(self):
        s = new_state()
        end = self.frames(s, n=12)
        s.feed(wire("GLOBAL_POSITION_INT", time_boot_ms=50_000 + 11 * 250 - 300), end)     # 300 ms older, arriving now
        self.assertEqual(s.snapshot(end)["link"]["lag_ms"], 300)
        s.feed(wire("GLOBAL_POSITION_INT", time_boot_ms=50_000 + 12 * 250), end + 0.25)
        self.assertEqual(s.snapshot(end + 0.25)["link"]["lag_ms"], 0)           # and the floor was never reset

    def test_the_warning_threshold_is_served_as_configuration_even_before_anything_is_heard(self):
        self.assertEqual(new_state().snapshot(1.0)["link"]["lag_warn_ms"], gl.LAG_WARN_MS)
        self.assertEqual(gl.LAG_WARN_MS, 1500)                  # the drone page's: gcs_page.py:1447


class EkfSection(unittest.TestCase):
    """The /mav `ekf` section and its verdict."""

    VARIANCES = ("velocity_variance", "pos_horiz_variance", "pos_vert_variance", "compass_variance", "terrain_alt_variance")
    FLAGS = ("attitude", "horiz_pos_abs", "const_pos_mode", "gps_glitch", "uninitialized")

    def test_a_healthy_filter_is_good_with_every_value_decoded(self):
        e = ekf_snapshot()
        self.assertEqual((e["ok"], e["quality"], e["quality_reasons"]), (True, "ok", []))
        self.assertEqual(e["flags"], 63)
        self.assertEqual([e[k] for k in self.FLAGS], [True, True, False, False, False])
        self.assertEqual([e[k] for k in self.VARIANCES], [0.05, 0.12, 0.03, 0.02, 0.0])
        self.assertEqual((e["warn_variance"], e["bad_variance"]), (0.5, 0.8))
        self.assertEqual(e["summary"], "max variance 0.12 (horizontal position)")

    def test_the_flag_bits_are_the_dialects_and_the_glitch_bit_is_the_one_rover_sends(self):
        bits = {f.name: f.bit for f in gl.EKF_KEY_FLAGS}
        self.assertEqual(bits, {"attitude": 1, "horiz_pos_abs": 16, "const_pos_mode": 128,
                                "gps_glitch": 32768, "uninitialized": 1024})
        self.assertEqual(bits["gps_glitch"], 1 << 15)             # AP_NavEKF3_Outputs.cpp, Rover-4.6.3: `flags |= (1<<15)`

    def test_every_variance_is_good_below_half_degraded_up_to_point_eight_and_poor_above(self):
        cases = ((0.0, "ok"), (0.49, "ok"), (0.5, "warn"), (0.65, "warn"), (0.8, "warn"), (0.801, "bad"),
                 (1.0, "bad"), (7.5, "bad"))
        for key in self.VARIANCES:
            for value, want in cases:
                with self.subTest(field=key, value=value):
                    self.assertEqual(ekf_snapshot(**{key: value})["quality"], want)

    def test_float32_point_eight_is_the_boundary_not_over_it(self):
        e = ekf_snapshot(velocity_variance=0.8)
        self.assertEqual(e["velocity_variance"], 0.8)             # the wire says 0.800000011920929
        self.assertEqual(e["quality"], "warn")

    def test_the_reasons_name_the_variance_and_the_worst_come_first(self):
        e = ekf_snapshot(velocity_variance=0.6, compass_variance=0.9, flags=63 | 32768)
        self.assertEqual(e["quality"], "bad")
        self.assertEqual(e["quality_reasons"], ["compass (GPS yaw) variance 0.90 > 0.8",
                                                "the EKF believes the GPS is glitching",
                                                "velocity variance 0.60 >= 0.5"])

    def test_each_key_flag_the_wrong_way_round(self):
        cases = ((63 & ~1, "bad", "no attitude estimate"),
                 (63 & ~16, "bad", "no absolute horizontal position"),
                 (63 | 128, "bad", "constant-position mode: the EKF has no position"),
                 (63 | 1024, "bad", "the EKF has never been healthy"),
                 (63 | 32768, "warn", "the EKF believes the GPS is glitching"))
        for flags, quality, reason in cases:
            with self.subTest(flags=flags):
                e = ekf_snapshot(flags=flags)
                self.assertEqual((e["quality"], e["quality_reasons"]), (quality, [reason]))

    def test_a_filter_with_nothing_good_says_so_twice(self):
        e = ekf_snapshot(flags=0)
        self.assertEqual((e["quality"], e["quality_reasons"]),
                         ("bad", ["no attitude estimate", "no absolute horizontal position"]))

    def test_a_variance_that_is_not_a_number_is_blank_and_never_counted(self):
        for value in (float("nan"), float("inf"), -0.5):
            with self.subTest(value=value):
                e = ekf_snapshot(compass_variance=value)
                self.assertIsNone(e["compass_variance"])
                self.assertEqual(e["quality"], "ok")
        e = ekf_snapshot(**{k: float("nan") for k in self.VARIANCES})
        self.assertEqual((e["quality"], e["quality_reasons"], e["summary"]),
                         ("ok", ["variances not reported"], "variances not reported"))

    def test_the_detail_lists_every_variance_and_flag_and_what_is_missing(self):
        e = ekf_snapshot(terrain_alt_variance=float("nan"))
        self.assertEqual(e["detail"], [
            "variances: velocity 0.05, horizontal position 0.12, vertical position 0.03, compass (GPS yaw) 0.02, "
            "terrain not reported",
            "flags: attitude yes, horizontal position (abs) yes, constant-position mode no, GPS glitching no, "
            "never initialised no"])

    def test_a_stale_section_is_unknown_with_every_value_null_and_its_age_kept(self):
        e = ekf_snapshot(feed_at=10.0, snap_at=20.0)
        self.assertEqual((e["ok"], e["age"], e["quality"], e["quality_reasons"]), (False, 10.0, "unknown", ["EKF data is stale"]))
        self.assertEqual([e[k] for k in gl.SECTIONS["ekf"]], [None] * len(gl.SECTIONS["ekf"]))
        self.assertEqual((e["summary"], e["detail"]), (None, []))
        self.assertEqual((e["warn_variance"], e["bad_variance"]), (0.5, 0.8))     # configuration, not data

    def test_never_heard_is_unknown_and_says_so(self):
        e = new_state().snapshot(50.0)["ekf"]
        self.assertEqual((e["ok"], e["age"], e["quality"], e["quality_reasons"]), (False, None, "unknown", ["no EKF data yet"]))

    def test_other_traffic_does_not_keep_it_alive_and_other_components_do_not_set_it(self):
        s = new_state(stale_s=3.0)
        s.feed(ekf_frame(), 100.0)
        s.feed(heartbeat(), 104.0)
        self.assertFalse(s.snapshot(104.5)["ekf"]["ok"])
        t = new_state()
        t.feed(wire("EKF_STATUS_REPORT", compid=191, flags=63), 10.0)
        self.assertFalse(t.snapshot(10.0)["ekf"]["ok"])

    def test_it_survives_json(self):
        e = json.loads(json.dumps(ekf_snapshot(velocity_variance=0.9)))
        self.assertEqual((e["quality"], e["quality_reasons"]), ("bad", ["velocity variance 0.90 > 0.8"]))


class PrearmSection(unittest.TestCase):
    """The /mav `prearm` section: SYS_STATUS's MAV_SYS_STATUS_PREARM_CHECK present / enabled / health."""

    def test_the_whole_truth_table(self):
        table = {(False, False, False): "not_reported", (False, False, True): "not_reported",
                 (False, True, False): "not_reported", (False, True, True): "not_reported",
                 (True, False, False): "disabled", (True, False, True): "disabled",
                 (True, True, False): "fail", (True, True, True): "pass"}
        for (present, enabled, health), want in table.items():
            with self.subTest(present=present, enabled=enabled, health=health):
                p = prearm_snapshot(present=present, enabled=enabled, health=health)
                self.assertEqual((p["present"], p["enabled"], p["health"], p["state"]), (present, enabled, health, want))

    def test_each_state_has_its_verdict_and_reason(self):
        cases = (({}, "pass", "ok", []),
                 (dict(health=False), "fail", "bad", "PreArm"),
                 (dict(enabled=False, health=False), "disabled", "warn", "ARMING_CHECK"),
                 (dict(present=False, enabled=False, health=False), "not_reported", "unknown", "does not report"))
        for args, state, quality, reason in cases:
            with self.subTest(state=state):
                p = prearm_snapshot(**args)
                self.assertEqual((p["state"], p["quality"]), (state, quality))
                if reason:
                    self.assertIn(reason, " ".join(p["quality_reasons"]))
                else:
                    self.assertEqual(p["quality_reasons"], [])

    def test_only_the_prearm_bit_counts(self):
        everything_else = 0x0FFFFFFF & ~PREARM_BIT
        p = prearm_snapshot(onboard_control_sensors_present=everything_else, onboard_control_sensors_enabled=everything_else,
                            onboard_control_sensors_health=everything_else)
        self.assertEqual(p["state"], "not_reported")

    def test_the_verdict_table_is_not_shared_between_snapshots(self):
        prearm_snapshot(health=False)["quality_reasons"].append("scribble")
        self.assertEqual(len(prearm_snapshot(health=False)["quality_reasons"]), 1)

    def test_a_stale_section_is_unknown_and_blank(self):
        s = new_state()
        s.feed(sys_frame(), 10.0)
        p = s.snapshot(20.0)["prearm"]
        self.assertEqual((p["ok"], p["age"], p["state"], p["health"], p["quality"], p["quality_reasons"]),
                         (False, 10.0, None, None, "unknown", ["pre-arm data is stale"]))

    def test_never_heard_is_unknown_and_says_so(self):
        p = new_state().snapshot(5.0)["prearm"]
        self.assertEqual((p["ok"], p["age"], p["quality"], p["quality_reasons"]),
                         (False, None, "unknown", ["no pre-arm data yet"]))

    def test_the_battery_is_what_it_was_when_the_same_message_also_says_prearm(self):
        s = new_state()
        s.feed(sys_frame(voltage_battery=0), 10.0)
        self.assertIsNone(s.snapshot(10.0)["batt"]["voltage"])
        s.feed(sys_frame(voltage_battery=65535), 11.0)
        self.assertIsNone(s.snapshot(11.0)["batt"]["voltage"])
        s.feed(sys_frame(voltage_battery=12600), 12.0)
        self.assertEqual(s.snapshot(12.0)["batt"]["voltage"], 12.6)


#: Runs STRIP_JS under node against a minimal DOM. A spec file says what /mav answers (one body per poll; a body
#: {"__fail__": true} makes that fetch reject), which tab the page is on, and which hooks to fire (the page's
#: renderTel() calls window.mavTel; true = it wiped the pane first). After every poll and every hook it prints the
#: strip's tiles and plain items, its whole text, and the Telemetry panel with its cards. The DOM is only as much
#: as the strip uses; what this proves is the strip's words, classes and structure, not its pixels.
TILE_HARNESS = r"""
const fs = require('fs');
const spec = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
const has = (n, cls) => (' ' + n.className + ' ').includes(' ' + cls + ' ');
class El {
  constructor(tag){ this.tag = tag; this.className = ''; this.title = ''; this.id = ''; this.kids = []; this.text = ''; this.parentNode = null; }
  appendChild(c){ c.parentNode = this; this.kids.push(c); return c; }
  replaceChild(n, o){ n.parentNode = this; o.parentNode = null; this.kids[this.kids.indexOf(o)] = n; return o; }
  get childNodes(){ return this.kids; }
  get textContent(){ return this.text + this.kids.map(k => k.textContent).join(''); }
  set textContent(v){ this.text = String(v); this.kids = []; }
}
const root = new El('body'), strip = new El('div'), pane = new El('div');
strip.id = 'mavstrip'; pane.id = 'p-tel';
root.appendChild(strip); root.appendChild(pane);
const find = (n, id) => n.id === id ? n : n.kids.reduce((f, k) => f || find(k, id), null);
const count = (n, id) => (n.id === id ? 1 : 0) + n.kids.reduce((t, k) => t + count(k, id), 0);
global.window = {};
global.tab = spec.tab;
global.document = {getElementById: id => find(root, id), createElement: tag => new El(tag)};
let polls = 0, tick = null;
global.fetch = () => {
  const body = spec.bodies[Math.min(polls++, spec.bodies.length - 1)];
  return body.__fail__ ? Promise.reject(new Error('down'))
                       : Promise.resolve({ok: true, json: () => Promise.resolve(body)});
};
global.setInterval = f => { tick = f; return 0; };
eval(fs.readFileSync(process.argv[3], 'utf8'));

function panel(){
  const box = find(pane, 'mavtel');
  if(!box) return null;
  const [h3, hint, grid] = box.kids;
  return {h3: h3.textContent, hint: hint.textContent,
          cards: grid.kids.map(c => ({label: c.kids[0].textContent, value: c.kids[1].textContent,
                                      cls: c.kids[1].className, title: c.title}))};
}
function snap(){
  const tiles = [], items = [];
  (function walk(n){
    if(has(n, 'mv-tile')){
      const top = n.kids[0], big = top.kids[1];
      tiles.push({cls: n.className, title: n.title, label: top.kids[0].textContent,
                  dot: big.kids[0].className, word: big.kids[1].textContent, sub: n.kids[1].textContent});
    } else if(has(n, 'mv-i')){
      const v = n.kids[n.kids.length - 1];
      items.push({label: n.kids.length > 1 ? n.kids[0].textContent : '', value: v.textContent,
                  cls: v.className, title: n.title});
    }
    n.kids.forEach(walk);
  })(strip);
  return {tiles: tiles, items: items, text: strip.textContent, panel: panel(), panels: count(pane, 'mavtel')};
}
const wait = ms => new Promise(r => setTimeout(r, ms));
(async () => {
  const steps = [], hooks = [];
  await wait(30); steps.push(snap());
  for(let i = 1; i < spec.bodies.length; i++){ tick(); await wait(30); steps.push(snap()); }
  for(const wipe of spec.hooks){
    if(wipe) pane.textContent = '';
    window.mavTel();
    hooks.push(snap());
  }
  console.log(JSON.stringify({steps: steps, hooks: hooks}));
})();
"""


@unittest.skipUnless(shutil.which("node"), "node is not installed")
class StripHarness(unittest.TestCase):
    """STRIP_JS under node (see TILE_HARNESS); the base of every test that reads what the strip built."""

    def run_node(self, bodies, tab=None, hooks=()):
        """{"steps": [after each poll], "hooks": [after each hook]}. Each is {"tiles", "items", "text", "panel",
        "panels"} plus "by" (the tiles by label) and "item" (the labelled items by label)."""
        spec = {"bodies": list(bodies), "tab": tab, "hooks": list(hooks)}
        with tempfile.TemporaryDirectory() as d:
            paths = {}
            for name, body in (("spec.json", json.dumps(spec)), ("harness.js", TILE_HARNESS), ("strip.js", gl.STRIP_JS)):
                paths[name] = os.path.join(d, name)
                with open(paths[name], "w", encoding="utf-8") as f:
                    f.write(body)
            r = subprocess.run(["node", paths["harness.js"], paths["spec.json"], paths["strip.js"]],
                               capture_output=True, text=True, encoding="utf-8", timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = json.loads(r.stdout)
        for step in out["steps"] + out["hooks"]:
            step["by"] = {t["label"]: t for t in step["tiles"]}
            step["item"] = {i["label"]: i for i in step["items"] if i["label"]}
        return out

    def run_strip(self, payload):
        """The strip after one /mav answer."""
        return self.run_node([payload])["steps"][0]


class GpsTiles(StripHarness):
    """The two tiles in the strip: what they say, which class they wear, what the tooltip adds."""

    def render(self, snap_at=10.0, **kw):
        """The strip after one GPS_RAW_INT (see gps_frame; `kw` overrides) read at `snap_at`."""
        s = new_state()
        s.feed(gps_frame(**kw), 10.0)
        s.feed(heartbeat(), 10.0)
        return self.run_strip(s.snapshot(snap_at))

    def test_the_strip_has_these_tiles_in_this_order(self):
        step = self.render()
        self.assertEqual([t["label"] for t in step["tiles"]], ["Mode", "GPS", "Heading", "EKF", "Pre-arm"])

    def test_a_healthy_fix_reads_good_and_good(self):
        out = self.render()
        gps, heading = out["by"]["GPS"], out["by"]["Heading"]
        self.assertEqual((gps["cls"], gps["dot"], gps["word"], gps["sub"]),
                         ("mv-tile ok", "mv-dot ok", "Good", "RTK FIXED · 24 sats · ±0.02 m"))
        self.assertEqual((heading["cls"], heading["dot"], heading["word"], heading["sub"]),
                         ("mv-tile ok", "mv-dot ok", "Good", "GPS yaw 123.4° ±0.3°"))
        for part in ("RTK FIXED", "24 satellites", "HDOP 0.60", "H acc ±0.02 m", "V acc ±0.03 m"):
            self.assertIn(part, gps["title"])
        self.assertTrue(out["text"].index("e-stop") < out["text"].index("GPS") < out["text"].index("Heading")
                        < out["text"].index("pos"), "the tiles sit between the e-stop and the position")

    def test_the_old_gps_text_is_gone(self):
        text = self.render()["text"]
        self.assertNotIn(" sat · ", text)
        self.assertNotIn("GPS yaw ok", text)

    def test_degraded_and_poor_have_their_words_classes_and_reasons(self):
        out = self.render(fix_type=3, h_acc=800, satellites_visible=8, eph=250)
        gps = out["by"]["GPS"]
        self.assertEqual((gps["cls"], gps["dot"], gps["word"]), ("mv-tile warn", "mv-dot warn", "Degraded"))
        self.assertIn("accuracy 0.80 m > 0.5 m; only 8 satellites (< 9); HDOP 2.5 > 2.0", gps["title"])
        self.assertEqual(gps["sub"], "3D FIX · 8 sats · ±0.80 m")
        poor = self.render(h_acc=2000)["by"]["GPS"]
        self.assertEqual((poor["cls"], poor["word"]), ("mv-tile bad", "Poor"))

    def test_heading_without_yaw_says_unavailable_and_no_compass(self):
        heading = self.render(yaw=65535)["by"]["Heading"]
        self.assertEqual((heading["cls"], heading["word"], heading["sub"]), ("mv-tile bad", "Poor", "UNAVAILABLE — no compass"))
        self.assertIn("no GPS heading — the boat has no compass", heading["title"])
        self.assertEqual(self.render(yaw=0)["by"]["Heading"]["sub"], "NOT PROVIDED — no compass")

    def test_heading_accuracy_is_shown_only_when_the_gps_reports_it(self):
        self.assertEqual(self.render(hdg_acc=0)["by"]["Heading"]["sub"], "GPS yaw 123.4°")
        warn = self.render(hdg_acc=200000)["by"]["Heading"]
        self.assertEqual((warn["cls"], warn["word"], warn["sub"]), ("mv-tile warn", "Degraded", "GPS yaw 123.4° ±2.0°"))

    def test_unreported_values_are_left_off_the_sub_line_not_guessed(self):
        gps = self.render(satellites_visible=255, h_acc=0, eph=65535)["by"]["GPS"]
        self.assertEqual(gps["sub"], "RTK FIXED")
        self.assertIn("HDOP not reported", gps["title"])

    def test_a_stale_section_is_no_data_with_its_age_and_no_last_value(self):
        by = self.render(snap_at=20.0)["by"]
        for tile in (by["GPS"], by["Heading"]):
            self.assertEqual((tile["cls"], tile["dot"], tile["word"], tile["sub"]),
                             ("mv-tile unknown", "mv-dot unknown", "No data", "stale: last 10 s ago"))
            self.assertNotIn("RTK", tile["sub"] + tile["title"])
            self.assertNotIn("123.4", tile["sub"] + tile["title"])

    def test_never_heard_is_no_data(self):
        by = self.run_strip(new_state().snapshot(5.0))["by"]
        self.assertEqual([(by[k]["word"], by[k]["sub"]) for k in ("GPS", "Heading")],
                         [("No data", "no GPS data"), ("No data", "no GPS yaw data")])


class ModeTile(StripHarness):
    """The mode as a tile, highlighted with the page's warn colour while ARMED."""

    def tile(self, snap_at=10.0, **hb):
        s = new_state()
        s.feed(heartbeat(**hb), 10.0)
        return self.run_strip(s.snapshot(snap_at))["by"]["Mode"]

    def test_disarmed_is_a_neutral_tile_with_the_mode_as_its_headline(self):
        t = self.tile(custom_mode=10)
        self.assertEqual((t["cls"], t["dot"], t["word"], t["sub"]),
                         ("mv-tile neutral", "mv-dot neutral", "AUTO", "disarmed"))

    def test_armed_wears_the_warn_class_whose_colour_is_the_pages_warn_variable(self):
        t = self.tile(custom_mode=10, base_mode=65 | 128)
        self.assertEqual((t["cls"], t["dot"], t["word"], t["sub"]),
                         ("mv-tile warn", "mv-dot warn", "AUTO", "ARMED"))
        self.assertRegex(gl.STRIP_CSS, r"#mavstrip \.mv-tile\.warn\{[^}]*border-left-color:var\(--warn\)")
        self.assertRegex(gl.STRIP_CSS, r"#mavstrip \.mv-tile\.warn \.mv-big\{color:var\(--warn\)\}")

    def test_the_neutral_class_has_no_colour_of_its_own(self):
        self.assertNotIn("neutral", gl.STRIP_CSS)

    def test_every_mode_the_boat_uses_is_the_headline(self):
        for custom, name in ((0, "MANUAL"), (4, "HOLD"), (10, "AUTO"), (11, "RTL"), (15, "GUIDED")):
            self.assertEqual(self.tile(custom_mode=custom)["word"], name)

    def test_a_stale_heartbeat_is_no_data_with_its_age_and_not_the_last_mode(self):
        t = self.tile(snap_at=20.0, custom_mode=10, base_mode=65 | 128)
        self.assertEqual((t["cls"], t["dot"], t["word"], t["sub"]),
                         ("mv-tile unknown", "mv-dot unknown", "No data", "stale: last 10 s ago"))
        self.assertNotIn("AUTO", t["word"] + t["sub"] + t["title"])
        self.assertNotIn("ARMED", t["sub"])

    def test_never_heard_is_no_data(self):
        t = self.run_strip(new_state().snapshot(5.0))["by"]["Mode"]
        self.assertEqual((t["word"], t["sub"]), ("No data", "no heartbeat data"))

    def test_the_old_plain_mode_and_armed_items_are_gone(self):
        step = self.run_strip(new_state().snapshot(5.0))
        self.assertNotIn("mode", step["item"])
        self.assertEqual([i for i in step["items"] if i["value"] in ("ARMED", "disarmed")], [])


class LinkItem(StripHarness):
    """The link item with its lag."""

    def link(self, mutate=None, n=12):
        s = new_state()
        end = LinkLag.frames(s, n=n)
        snap = s.snapshot(end)
        if mutate:
            mutate(snap["link"])
        return self.run_strip(snap)["item"]["link"]

    def test_the_lag_follows_the_rate_in_the_link_item(self):
        item = self.link()
        self.assertRegex(item["value"], r"^0\.\d s · \d fps · lag 0 ms$")
        self.assertEqual(item["cls"], "mv-ok")
        self.assertIn("boot clock", item["title"])                 # the tooltip says what lag is and what it is relative to

    def test_a_lag_that_is_not_measured_yet_is_a_dash_not_zero(self):
        item = self.link(n=3)
        self.assertTrue(item["value"].endswith("· lag —"), item["value"])
        self.assertEqual(item["cls"], "mv-ok")

    def test_a_lag_above_the_drones_limit_turns_the_item_to_warn(self):
        for lag, cls in ((1500, "mv-ok"), (1501, "mv-warn"), (4200, "mv-warn")):
            item = self.link(lambda L, lag=lag: L.update(lag_ms=lag))
            self.assertEqual(item["cls"], cls, lag)
            self.assertTrue(item["value"].endswith("lag %d ms" % lag))
        self.assertIn("Above 1500 ms", self.link(lambda L: L.update(lag_ms=4200))["title"])

    def test_a_dead_link_shows_no_lag(self):
        step = self.run_strip(new_state().snapshot(5.0))
        self.assertNotIn("lag", step["item"]["link"]["value"])


class EkfTile(StripHarness):
    def tile(self, snap_at=10.0, **kw):
        s = new_state()
        s.feed(ekf_frame(**kw), 10.0)
        return self.run_strip(s.snapshot(snap_at))["by"]["EKF"]

    def test_a_healthy_filter_reads_good_and_the_tooltip_explains_the_scale_and_the_compass(self):
        t = self.tile()
        self.assertEqual((t["cls"], t["dot"], t["word"], t["sub"]),
                         ("mv-tile ok", "mv-dot ok", "Good", "max variance 0.12 (horizontal position)"))
        for part in ("Good below 0.5", "to 0.8", "Mission Planner", "velocity 0.05", "compass (GPS yaw) 0.02",
                     "attitude yes", "GPS-yaw innovation", "COMPASS_USE=0"):
            self.assertIn(part, t["title"])

    def test_degraded_and_poor_say_which_variance_tripped(self):
        warn = self.tile(pos_horiz_variance=0.6)
        self.assertEqual((warn["cls"], warn["word"], warn["sub"]),
                         ("mv-tile warn", "Degraded", "max variance 0.60 (horizontal position)"))
        self.assertIn("horizontal position variance 0.60 >= 0.5", warn["title"])
        bad = self.tile(compass_variance=0.95)
        self.assertEqual((bad["cls"], bad["word"], bad["sub"]), ("mv-tile bad", "Poor", "max variance 0.95 (compass (GPS yaw))"))
        self.assertIn("compass (GPS yaw) variance 0.95 > 0.8", bad["title"])

    def test_a_flag_problem_is_poor_even_with_perfect_variances(self):
        t = self.tile(flags=63 | 1024)
        self.assertEqual((t["cls"], t["word"]), ("mv-tile bad", "Poor"))
        self.assertIn("the EKF has never been healthy", t["title"])
        self.assertIn("never initialised yes", t["title"])

    def test_a_stale_section_is_no_data_with_its_age_and_no_last_value(self):
        t = self.tile(snap_at=20.0, compass_variance=0.95)
        self.assertEqual((t["cls"], t["dot"], t["word"], t["sub"]),
                         ("mv-tile unknown", "mv-dot unknown", "No data", "stale: last 10 s ago"))
        self.assertNotIn("0.95", t["sub"] + t["title"])
        self.assertNotIn("variances:", t["title"])

    def test_never_heard_is_no_data(self):
        t = self.run_strip(new_state().snapshot(5.0))["by"]["EKF"]
        self.assertEqual((t["word"], t["sub"]), ("No data", "no EKF data"))


class PrearmTile(StripHarness):
    def tile(self, snap_at=10.0, armed=False, **kw):
        s = new_state()
        s.feed(sys_frame(**kw), 10.0)
        s.feed(heartbeat(base_mode=65 | (128 if armed else 0)), 10.0)
        return self.run_strip(s.snapshot(snap_at))["by"]["Pre-arm"]

    def test_the_four_states(self):
        cases = (({}, "mv-tile ok", "PASS", "checks pass"),
                 (dict(health=False), "mv-tile bad", "FAIL", "see the PreArm: warnings"),
                 (dict(enabled=False, health=False), "mv-tile warn", "CHECKS OFF", "ARMING_CHECK is 0"),
                 (dict(present=False, enabled=False, health=False), "mv-tile unknown", "not reported", "no pre-arm bit"))
        for args, cls, word, sub in cases:
            with self.subTest(word=word):
                t = self.tile(**args)
                self.assertEqual((t["cls"], t["word"], t["sub"]), (cls, word, sub))

    def test_an_armed_pass_says_it_was_not_checked_again(self):
        t = self.tile(armed=True)
        self.assertEqual((t["cls"], t["word"], t["sub"]), ("mv-tile ok", "PASS", "armed: not re-checked"))
        self.assertIn("While armed", t["title"])
        self.assertEqual(self.tile(armed=True, health=False)["sub"], "see the PreArm: warnings")

    def test_a_stale_section_is_no_data_with_its_age_and_not_the_last_verdict(self):
        t = self.tile(snap_at=20.0)
        self.assertEqual((t["cls"], t["word"], t["sub"]), ("mv-tile unknown", "No data", "stale: last 10 s ago"))
        self.assertNotIn("PASS", t["word"] + t["sub"])

    def test_never_heard_is_no_data(self):
        t = self.run_strip(new_state().snapshot(5.0))["by"]["Pre-arm"]
        self.assertEqual((t["word"], t["sub"]), ("No data", "no pre-arm data"))


class BatteryItem(StripHarness):
    """The battery stays an item, now labelled unverified."""

    def batt(self, snap_at=10.0, **kw):
        s = new_state()
        s.feed(sys_frame(**kw), 10.0)
        return self.run_strip(s.snapshot(snap_at))["item"]["batt"]

    def test_a_voltage_is_labelled_unverified_and_the_tooltip_says_why(self):
        b = self.batt()
        self.assertEqual(b["value"], "12.60 V (unverified)")
        for part in ("UNVERIFIED", "BATT_VOLT_MULT", "3S or 4S", "crusader-battery"):
            self.assertIn(part, b["title"])

    def test_zero_and_65535_are_no_sensor_never_a_voltage(self):
        for raw in (0, 65535):
            with self.subTest(raw=raw):
                b = self.batt(voltage_battery=raw)
                self.assertEqual((b["value"], b["cls"]), ("no sensor", "mv-dim"))
                self.assertNotIn(" V", b["value"])
                self.assertNotIn("unverified", b["value"])

    def test_a_stale_battery_is_a_dash(self):
        b = self.batt(snap_at=20.0)
        self.assertEqual((b["value"], b["cls"]), ("—", "mv-dim"))
        self.assertIn("last received 10.0 s ago", b["title"])


class TelemetryPanel(StripHarness):
    """The GPS panel under the Telemetry tab (window.mavTel)."""

    CARDS = ["Fix", "Satellites", "HDOP", "Horizontal accuracy", "Vertical accuracy", "GPS yaw", "Yaw accuracy",
             "Latitude", "Longitude", "Altitude (ellipsoid)"]

    @staticmethod
    def mav(snap_at=10.0, **kw):
        """A /mav body after one GPS_RAW_INT (healthy RTK fix near 1.3 N 103.8 E, ellipsoid height 31.25 m)."""
        s = new_state()
        s.feed(gps_frame(**dict({"lat": 13_000_000, "lon": 1_038_000_000, "alt_ellipsoid": 31250}, **kw)), 10.0)
        s.feed(heartbeat(), 10.0)
        return s.snapshot(snap_at)

    @staticmethod
    def cards(panel):
        return {c["label"]: c for c in panel["cards"]}

    def test_a_fresh_fix_fills_every_card(self):
        panel = self.run_node([self.mav()], tab="tel")["steps"][0]["panel"]
        self.assertEqual([c["label"] for c in panel["cards"]], self.CARDS)
        want = {"Fix": "RTK FIXED", "Satellites": "24", "HDOP": "0.60", "Horizontal accuracy": "0.02 m",
                "Vertical accuracy": "0.03 m", "GPS yaw": "123.4°", "Yaw accuracy": "0.3°",
                "Latitude": "1.3000000°", "Longitude": "103.8000000°", "Altitude (ellipsoid)": "31.25 m"}
        cards = self.cards(panel)
        for label, value in want.items():
            self.assertEqual((cards[label]["value"], cards[label]["cls"]), (value, "v"), label)
        self.assertIn("GPS", panel["h3"])
        self.assertEqual(panel["hint"], "Read by this laptop, not the Jetson. last received 0.0 s ago.")

    def test_coordinates_carry_all_seven_decimals(self):
        panel = self.run_node([self.mav(lat=-338_123_456, lon=1_038_765_432)], tab="tel")["steps"][0]["panel"]
        cards = self.cards(panel)
        self.assertEqual((cards["Latitude"]["value"], cards["Longitude"]["value"]), ("-33.8123456°", "103.8765432°"))

    def test_what_the_autopilot_does_not_report_is_a_plain_dash_with_a_title_not_a_red_one(self):
        panel = self.run_node([self.mav(h_acc=0, v_acc=0, hdg_acc=0, alt_ellipsoid=0, satellites_visible=255,
                                        eph=65535)], tab="tel")["steps"][0]["panel"]
        cards = self.cards(panel)
        for label in ("Horizontal accuracy", "Vertical accuracy", "Yaw accuracy", "Altitude (ellipsoid)",
                      "Satellites", "HDOP"):
            self.assertEqual((cards[label]["value"], cards[label]["cls"]), ("—", "v"), label)
            self.assertEqual(cards[label]["title"], label + " is not reported by the autopilot")
        self.assertEqual(cards["Fix"]["value"], "RTK FIXED")               # what was reported is still there

    def test_no_position_is_a_dash(self):
        panel = self.run_node([self.mav(lat=0, lon=0, fix_type=1)], tab="tel")["steps"][0]["panel"]
        cards = self.cards(panel)
        self.assertEqual([cards[k]["value"] for k in ("Latitude", "Longitude", "Altitude (ellipsoid)")], ["—"] * 3)

    def test_gps_yaw_that_is_not_a_number_says_why_and_is_red(self):
        for yaw, word in ((65535, "UNAVAILABLE"), (0, "NOT PROVIDED"), (50000, "out of range")):
            with self.subTest(yaw=yaw):
                card = self.cards(self.run_node([self.mav(yaw=yaw)], tab="tel")["steps"][0]["panel"])["GPS yaw"]
                self.assertEqual((card["value"], card["cls"]), (word, "v stale"))

    def test_a_stale_section_is_a_red_dash_in_every_card_and_says_how_old(self):
        step = self.run_node([self.mav(snap_at=20.0)], tab="tel")["steps"][0]
        for card in step["panel"]["cards"]:
            self.assertEqual((card["value"], card["cls"]), ("—", "v stale"), card["label"])
        self.assertEqual(step["panel"]["hint"], "Read by this laptop, not the Jetson. stale: last 10 s ago.")
        self.assertNotIn("1.3", " ".join(c["value"] for c in step["panel"]["cards"]))

    def test_a_failed_poll_blanks_the_panel_and_the_next_answer_brings_it_back(self):
        out = self.run_node([self.mav(), {"__fail__": True}, self.mav(satellites_visible=12)], tab="tel")
        fresh, failed, back = (s["panel"] for s in out["steps"])
        self.assertEqual(self.cards(fresh)["Satellites"]["value"], "24")
        self.assertEqual([c["value"] for c in failed["cards"]], ["—"] * len(self.CARDS))     # no remembered value
        self.assertEqual({c["cls"] for c in failed["cards"]}, {"v stale"})
        self.assertIn("No answer from the laptop server", failed["hint"])
        self.assertEqual(self.cards(back)["Satellites"]["value"], "12")

    def test_an_error_body_from_the_laptop_server_is_a_failure_too(self):
        out = self.run_node([self.mav(), {"error": "mav status failed: boom"}], tab="tel")
        self.assertIn("No answer", out["steps"][1]["panel"]["hint"])
        self.assertEqual(self.cards(out["steps"][1]["panel"])["Fix"]["value"], "—")

    def test_the_strip_keeps_the_panel_current_by_itself_without_ever_stacking_a_second(self):
        out = self.run_node([self.mav(), self.mav(satellites_visible=11), self.mav(satellites_visible=10)], tab="tel")
        self.assertEqual([s["panels"] for s in out["steps"]], [1, 1, 1])
        self.assertEqual([self.cards(s["panel"])["Satellites"]["value"] for s in out["steps"]], ["24", "11", "10"])

    def test_it_is_only_built_while_the_telemetry_tab_is_showing(self):
        for tab in ("nodes", "map", None):
            with self.subTest(tab=tab):
                step = self.run_node([self.mav()], tab=tab)["steps"][0]
                self.assertEqual((step["panel"], step["panels"]), (None, 0))

    def test_the_pages_hook_rebuilds_it_after_the_pane_is_wiped_and_replaces_it_in_place_otherwise(self):
        out = self.run_node([self.mav()], tab="tel", hooks=[True, False, True, False])
        self.assertEqual([h["panels"] for h in out["hooks"]], [1, 1, 1, 1])
        self.assertEqual(self.cards(out["hooks"][2]["panel"])["Satellites"]["value"], "24")

    def test_the_hook_works_before_any_answer_and_shows_blanks(self):
        out = self.run_node([{"__fail__": True}], tab="nodes", hooks=[True])
        panel = out["hooks"][0]["panel"]
        self.assertEqual({c["value"] for c in panel["cards"]}, {"—"})
        self.assertIn("No answer", panel["hint"])

    def test_it_is_built_from_the_pages_own_classes_so_day_mode_recolours_it(self):
        for selector in (".grid{", ".card{", ".card .k{", ".card .v{", ".hint{", ".stale{", "h3{"):
            self.assertIn(selector, PAGE)
        panel_js = gl.STRIP_JS[gl.STRIP_JS.index("function telCard"):gl.STRIP_JS.index("function poll")]
        self.assertNotRegex(panel_js, r"style|#[0-9a-fA-F]{3,6}\b|rgb\(")          # no colour of its own
        self.assertNotIn("innerHTML", panel_js)


class Position(unittest.TestCase):
    def pos(self, **kw):
        s = new_state()
        s.feed(wire("GLOBAL_POSITION_INT", **kw), 10.0)
        return s.snapshot(10.0)["pos"]

    def test_values(self):
        p = self.pos(lat=13_000_000, lon=1_038_000_000, hdg=9050, vx=300, vy=400)
        self.assertEqual((p["lat"], p["lon"], p["heading"], p["groundspeed"]), (1.3, 103.8, 90.5, 5.0))

    def test_no_position_and_unknown_heading_are_blank(self):
        p = self.pos(lat=0, lon=0, hdg=65535)
        self.assertTrue(p["ok"])
        self.assertEqual((p["lat"], p["lon"], p["heading"]), (None, None, None))


class StatusText(unittest.TestCase):
    def feed(self, s, now, sev, text, **kw):
        s.feed(wire("STATUSTEXT", severity=sev, text=text, **kw), now)

    def test_only_warnings_or_worse_and_only_the_last_three_newest_first(self):
        s = new_state()
        self.feed(s, 1.0, 6, "info is not shown")
        for i, sev in enumerate((4, 3, 4, 2, 4)):
            self.feed(s, 10.0 + i, sev, "msg %d" % i)
        out = s.snapshot(20.0)["statustext"]
        self.assertEqual([t["text"] for t in out], ["msg 4", "msg 3", "msg 2"])
        self.assertEqual([t["age"] for t in out], [6.0, 7.0, 8.0])
        self.assertEqual([t["severity"] for t in out], [4, 2, 4])
        self.assertEqual([t["severity_name"] for t in out], ["WARNING", "CRITICAL", "WARNING"])

    def test_chunks_of_one_long_message_are_joined(self):
        s = new_state()
        self.feed(s, 10.0, 4, "PreArm: this is the first half, ", id=7, chunk_seq=0)
        self.feed(s, 10.1, 4, "and this is the second", id=7, chunk_seq=1)
        self.feed(s, 11.0, 4, "separate", id=0)
        self.assertEqual([t["text"] for t in s.snapshot(12.0)["statustext"]],
                         ["separate", "PreArm: this is the first half, and this is the second"])

    def test_text_from_another_component_or_system_is_ignored(self):
        s = new_state()
        s.feed(wire("STATUSTEXT", compid=191, severity=3, text="from a companion"), 10.0)
        s.feed(wire("STATUSTEXT", sysid=1, severity=3, text="from another boat"), 10.0)
        self.assertEqual(s.snapshot(11.0)["statustext"], [])


class Staleness(unittest.TestCase):
    def test_a_value_older_than_the_limit_is_null_with_its_age_kept(self):
        s = new_state(stale_s=3.0)
        s.feed(heartbeat(base_mode=193, custom_mode=10), 100.0)
        s.feed(wire("SYS_STATUS", voltage_battery=12600, current_battery=-1, battery_remaining=-1), 100.0)
        fresh = s.snapshot(102.9)
        self.assertEqual((fresh["hb"]["ok"], fresh["hb"]["mode"], fresh["batt"]["voltage"]), (True, "AUTO", 12.6))
        stale = s.snapshot(103.5)
        self.assertEqual((stale["hb"]["ok"], stale["hb"]["age"]), (False, 3.5))
        self.assertEqual((stale["hb"]["mode"], stale["hb"]["armed"], stale["hb"]["type"]), (None, None, None))
        self.assertEqual((stale["batt"]["ok"], stale["batt"]["voltage"]), (False, None))

    def test_never_heard_is_null_with_no_age(self):
        snap = new_state().snapshot(50.0)
        for name in ("hb", "pos", "gps", "rc", "batt"):
            self.assertFalse(snap[name]["ok"], name)
            self.assertIsNone(snap[name]["age"], name)
        self.assertFalse(snap["link"]["ok"])
        self.assertEqual(snap["statustext"], [])

    def test_the_estop_switch_is_configuration_so_it_is_there_even_when_nothing_was_heard(self):
        rc = new_state().snapshot(50.0)["rc"]
        self.assertEqual((rc["estop_channel"], rc["estop_threshold"], rc["estop_engaged"]), (7, 1200, None))


class Link(unittest.TestCase):
    def test_frames_per_second_over_the_window(self):
        s = new_state()
        for i in range(50):                                        # 10 frames/s for 5 s
            s.feed(heartbeat(), 100.0 + i * 0.1)
        link = s.snapshot(104.95)["link"]
        self.assertTrue(link["ok"])
        self.assertAlmostEqual(link["fps"], 10.0, delta=1.0)
        self.assertAlmostEqual(link["age"], 0.05, places=2)
        self.assertEqual((link["source"], link["sysid"]), ("test", 2))

    def test_a_silent_link_has_an_age_and_no_rate(self):
        s = new_state()
        s.feed(heartbeat(), 100.0)
        link = s.snapshot(110.0)["link"]
        self.assertEqual((link["ok"], link["age"], link["fps"]), (False, 10.0, None))

    def test_other_systems_do_not_count_but_are_named(self):
        s = new_state()
        s.feed(heartbeat(sysid=1, type=2), 100.0)
        link = s.snapshot(100.5)["link"]
        self.assertEqual((link["ok"], link["age"], link["other_sysids"]), (False, None, [1]))
        s.feed(heartbeat(sysid=2), 100.6)
        self.assertTrue(s.snapshot(100.7)["link"]["ok"])

    def test_every_component_of_the_system_keeps_the_link_alive_but_not_the_values(self):
        s = new_state()
        s.feed(heartbeat(compid=191, type=18, autopilot=8, base_mode=0), 100.0)
        snap = s.snapshot(100.5)
        self.assertTrue(snap["link"]["ok"])
        self.assertFalse(snap["hb"]["ok"])

    def test_a_reader_error_is_named_while_the_link_is_down(self):
        s = new_state()
        s.note_error(OSError("socket closed"))
        self.assertEqual(s.snapshot(1.0)["link"]["error"], "OSError: socket closed")
        s.feed(heartbeat(), 2.0)
        self.assertIsNone(s.snapshot(2.1)["link"]["error"])


# ---------------------------------------------------------------- never transmit

class ListenOnlyRule(unittest.TestCase):
    def test_the_reader_has_no_way_to_send(self):
        self.assertEqual([n for n in dir(gl.ListenOnly) if not n.startswith("_")], ["close", "recv_match"])

    def test_the_source_never_touches_mav_or_a_send_call(self):
        tree = ast.parse(read_file(os.path.join(SCRIPTS, "gcs_laptop.py")))
        banned = [n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)
                  and (n.attr in ("mav", "sendto", "sendall", "send") or re.fullmatch(r"\w+_send", n.attr))]
        self.assertEqual(banned, [])

    def test_nothing_is_ever_written_to_the_socket_while_frames_are_read(self):
        port = free_port()
        source = gl.ListenOnly("udpin:%s:%d" % (LOCAL, port))
        state = new_state()
        stop = threading.Event()
        reader = threading.Thread(target=gl.read_loop, args=(source, state, stop), daemon=True)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        written = []

        def spy(real, name):
            def wrapper(sock, *a, **kw):
                if sock is not sender:
                    written.append(name)
                return real(sock, *a, **kw)
            return wrapper

        patches = [mock.patch.object(socket.socket, n, spy(getattr(socket.socket, n), n))
                   for n in ("sendto", "send", "sendall")]
        for p in patches:
            p.start()
        try:
            reader.start()
            for frame in (encode("HEARTBEAT", type=11, autopilot=3, base_mode=193, custom_mode=10),
                          encode("HEARTBEAT", sysid=255, compid=190, type=6, autopilot=8),
                          encode("SYS_STATUS", voltage_battery=0, current_battery=-1, battery_remaining=-1),
                          encode("STATUSTEXT", severity=2, text="Crash: yes"),
                          encode("EKF_STATUS_REPORT", flags=63, pos_horiz_variance=0.1),
                          encode("GLOBAL_POSITION_INT", time_boot_ms=5000),
                          encode("RC_CHANNELS", chancount=16, chan7_raw=1900),
                          encode("PARAM_VALUE", param_id=b"X", param_value=1.0, param_count=1)):
                sender.sendto(frame, (LOCAL, port))
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and not state.snapshot(time.monotonic())["rc"]["ok"]:
                time.sleep(0.02)
            time.sleep(0.3)                          # long enough for any reply to have been attempted
        finally:
            stop.set()
            reader.join(3)
            for p in patches:
                p.stop()
            source.close()
            sender.close()
        snap = state.snapshot(time.monotonic())
        self.assertEqual((snap["hb"]["mode"], snap["hb"]["armed"], snap["rc"]["estop_engaged"]), ("AUTO", True, False))
        self.assertEqual(snap["statustext"][0]["text"], "Crash: yes")
        self.assertEqual((snap["ekf"]["ok"], snap["ekf"]["quality"]), (True, "ok"))        # the new messages were read ...
        self.assertTrue(snap["prearm"]["ok"])
        self.assertEqual(written, [], "the reader wrote to a socket")                      # ... and still nothing was sent


# ---------------------------------------------------------------- the Jetson

class Passthrough(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.jetson = FakeJetson()
        cls.link = explicit(cls.jetson.port)

    @classmethod
    def tearDownClass(cls):
        cls.jetson.stop()

    def test_state_is_the_jetsons_dict_unchanged(self):
        self.assertEqual(self.link.state(set()), FakeJetson.SNAPSHOT)
        self.assertEqual(self.jetson.layers[-1], set())

    def test_the_layers_survive_the_trip(self):
        self.link.state({"prox", "clusters"})
        self.assertEqual(self.jetson.layers[-1], {"clusters", "prox"})

    def test_a_hostile_layer_name_cannot_inject_a_request(self):
        self.link.state({"a b\r\nX: y&z"})
        self.assertEqual(self.jetson.layers[-1], {"a b\r\nX: y&z"})

    def test_an_action_goes_to_the_jetson_with_its_payload_and_its_answer_comes_back(self):
        out = self.link.action("/node/start", {"name": "lidar_view"})
        self.assertEqual(out, {"ok": True, "message": "did /node/start"})
        self.assertEqual(self.jetson.actions[-1], ("/node/start", {"name": "lidar_view"}))

    def test_only_absolute_paths_are_forwarded(self):
        before = len(self.jetson.actions)
        for path in ("node/start", "http://elsewhere/node/start", ""):
            with self.subTest(path=path):
                out = self.link.action(path, {})
                self.assertFalse(out["ok"])
                self.assertIn("nothing was sent", out["message"])
        self.assertEqual(len(self.jetson.actions), before)

    def test_a_download_streams_whole(self):
        class Sink:
            def __init__(self):
                self.data = bytearray()

            def write(self, b):
                self.data += b
        sink = Sink()
        self.assertTrue(self.link.download("sess 1", sink))
        self.assertEqual(bytes(sink.data), b"".join((b"sess 1|" * 20000 + bytes([i])) for i in range(5)))
        self.assertGreater(len(sink.data), 3 * gl.CHUNK)

    def test_health_follows_the_last_real_call(self):
        clock = Clock()
        link = gl.JetsonLink("%s:%d" % (LOCAL, self.jetson.port), clock=clock)
        self.assertEqual(link.status(3.0)["jetson_ok"], None)             # no call yet: unknown
        link.state(set())
        ok = link.status(3.0)
        self.assertEqual((ok["jetson_host"], ok["jetson_ok"], ok["jetson_age"]), (LOCAL, True, 0.0))
        clock.t += 10
        old = link.status(3.0)
        self.assertEqual((old["jetson_ok"], old["jetson_age"]), (None, 10.0))   # a success that old is unknown


class JetsonDown(unittest.TestCase):
    def test_a_jetson_that_goes_away_mid_session_is_an_error_never_a_remembered_snapshot(self):
        jetson = FakeJetson()
        link = explicit(jetson.port)
        self.assertEqual(link.state(set()), FakeJetson.SNAPSHOT)
        jetson.stop()
        with self.assertRaises(gl.JetsonError) as cm:
            link.state(set())
        self.assertIn("Jetson %s:%d unreachable" % (LOCAL, jetson.port), str(cm.exception))
        self.assertFalse(cm.exception.sent)
        self.assertIs(link.status(3.0)["jetson_ok"], False)

    def test_an_action_says_unreachable_and_that_nothing_was_sent(self):
        out = dead_link().action("/node/stop", {"name": "x"})
        self.assertFalse(out["ok"])
        self.assertIn("Jetson %s:%d unreachable" % DEAD, out["message"])
        self.assertTrue(out["message"].endswith("— nothing was sent"))

    def test_a_reply_lost_after_sending_says_the_action_may_have_happened(self):
        raw = RawJetson(swallow=True)
        self.addCleanup(raw.stop)
        out = explicit(raw.port).action("/power", {"verb": "shutdown"})
        self.assertFalse(out["ok"])
        self.assertIn("may have reached the Jetson", out["message"])
        self.assertNotIn("nothing was sent", out["message"])
        self.assertEqual(raw.hits, 1)

    def test_a_reply_that_is_not_json_or_not_200_is_a_failure_that_may_have_happened(self):
        raw = RawJetson()
        self.addCleanup(raw.stop)
        for status, body, words in ((200, b"hello", "not JSON"), (200, b"[1]", "not an object"),
                                    (500, b"{}", "HTTP 500")):
            with self.subTest(status=status, body=body):
                raw.status, raw.body = status, body
                out = explicit(raw.port).action("/params/set", {})
                self.assertFalse(out["ok"])
                self.assertIn(words, out["message"])
                self.assertIn("may have reached", out["message"])

    def test_a_download_from_a_dead_or_refusing_jetson_returns_false_and_writes_nothing(self):
        class Sink:
            def write(self, b):
                raise AssertionError("wrote %d bytes" % len(b))
        self.assertFalse(dead_link().download("x", Sink()))
        raw = RawJetson(404, b"no")
        self.addCleanup(raw.stop)
        self.assertFalse(explicit(raw.port).download("x", Sink()))

    def test_health_goes_false_with_the_reason(self):
        link = dead_link()
        with self.assertRaises(gl.JetsonError):
            link.state(set())
        st = link.status(3.0)
        self.assertIs(st["jetson_ok"], False)
        self.assertTrue(st["jetson_error"])
        self.assertIsNotNone(st["jetson_age"])


class Probing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.a, cls.b = FakeJetson(), FakeJetson()

    @classmethod
    def tearDownClass(cls):
        cls.a.stop()
        cls.b.stop()

    def setUp(self):
        self.clock = Clock()

    def link(self, *candidates, spec="auto"):
        return gl.JetsonLink(spec, candidates=candidates, clock=self.clock)

    def test_it_takes_the_first_candidate_that_answers_and_sticks_with_it(self):
        link = self.link(DEAD, (LOCAL, self.a.port), (LOCAL, self.b.port))
        asked_b = len(self.b.layers)
        self.assertEqual(link.probe(), (LOCAL, self.a.port))
        self.assertEqual(link.target, (LOCAL, self.a.port))
        link.state(set())
        self.assertEqual(len(self.b.layers), asked_b)                # the later candidate was never asked

    def test_nothing_answering_leaves_no_target_and_says_what_was_tried(self):
        link = self.link(DEAD)
        self.assertIsNone(link.probe())
        with self.assertRaises(gl.JetsonError) as cm:
            link.state(set())
        self.assertIn("%s:%d" % DEAD, str(cm.exception))
        self.assertIs(link.status(3.0)["jetson_ok"], False)

    def test_after_a_failure_it_reprobes_and_moves_to_the_one_that_answers(self):
        flaky = RawJetson(200, b"{}")                                # a ground station, until it is not
        self.addCleanup(flaky.stop)
        link = self.link((LOCAL, flaky.port), (LOCAL, self.b.port))
        self.assertEqual(link.probe(), (LOCAL, flaky.port))
        flaky.status = 500
        self.clock.t += 6                                           # past the 5 s re-probe spacing
        with self.assertRaises(gl.JetsonError):
            link.state(set())
        link.probe_thread.join(10)
        self.assertEqual(link.target, (LOCAL, self.b.port))
        self.assertEqual(link.state(set()), FakeJetson.SNAPSHOT)
        self.assertEqual(link.status(3.0)["jetson_host"], LOCAL)

    def test_reprobing_is_rate_limited(self):
        link = self.link(DEAD)
        with mock.patch.object(link, "probe", wraps=link.probe) as probe:
            link.probe()                                            # startup round
            for _ in range(4):                                      # a burst of failures within 5 s
                with self.assertRaises(gl.JetsonError):
                    link.state(set())
            self.assertEqual(probe.call_count, 1)
            self.clock.t += 5.1
            with self.assertRaises(gl.JetsonError):
                link.state(set())
            link.probe_thread.join(10)
            self.assertEqual(probe.call_count, 2)

    def test_an_explicit_host_is_never_probed_away_from(self):
        link = self.link((LOCAL, self.a.port), spec="%s:%d" % DEAD)
        self.assertFalse(link.auto)
        self.clock.t += 60
        with self.assertRaises(gl.JetsonError):
            link.state(set())
        self.assertIsNone(link.probe_thread)
        self.assertEqual(link.target, DEAD)

    def test_targets_parse(self):
        self.assertEqual(gl.parse_target("192.168.8.109"), ("192.168.8.109", 8090))
        self.assertEqual(gl.parse_target("crusader.local:9000"), ("crusader.local", 9000))
        self.assertEqual(gl.parse_target("http://10.0.0.5:8090/"), ("10.0.0.5", 8090))
        for bad in ("", ":8090", "host:port", "host:0", "host:70000"):
            with self.subTest(bad), self.assertRaises(ValueError):
                gl.parse_target(bad)


# ---------------------------------------------------------------- the served laptop page

class Served(unittest.TestCase):
    """The real LaptopServer in front of a fake Jetson, over real HTTP."""

    @classmethod
    def setUpClass(cls):
        cls.jetson = FakeJetson()
        cls.link = explicit(cls.jetson.port)
        cls.state = new_state(stale_s=3600)          # the class's tests outlast a 3 s limit
        cls.state.feed(heartbeat(base_mode=193, custom_mode=10), time.monotonic())
        cls.state.feed(wire("RC_CHANNELS", chancount=16, chan7_raw=1999), time.monotonic())
        cls.server = gl.LaptopServer(gl.build_page(200), cls.link,
                                     lambda: gl.mav_status(cls.link, cls.state, 3.0))
        cls.server.start(0, LOCAL)
        cls.port = cls.server._server.server_address[1]
        cls.server.allowed_hosts = gl.allowed_hosts_for(LOCAL, cls.port)
        cls.base = "http://%s:%d" % (LOCAL, cls.port)

    @classmethod
    def tearDownClass(cls):
        cls.server.stop()
        cls.jetson.stop()

    def test_the_page_is_the_patched_one(self):
        with urllib.request.urlopen(self.base + "/", timeout=5) as r:
            self.assertIn("charset=utf-8", r.headers["Content-Type"])
            html = r.read().decode("utf-8")
        self.assertIn('<div id="mavstrip">', html)
        self.assertIn("window.VIEW_HOST", html)
        self.assertIn('data-t="task1"', html)

    def test_state_passes_straight_through_with_its_layers(self):
        self.assertEqual(get_json(self.base + "/state"), FakeJetson.SNAPSHOT)
        get_json(self.base + "/state?layers=clusters,prox")
        self.assertEqual(self.jetson.layers[-1], {"clusters", "prox"})

    def test_a_post_reaches_the_jetson(self):
        self.assertEqual(post_json(self.base + "/trail/clear", {"a": 1}), {"ok": True, "message": "did /trail/clear"})
        self.assertEqual(self.jetson.actions[-1], ("/trail/clear", {"a": 1}))

    def test_the_download_link_works_through_it(self):
        with urllib.request.urlopen(self.base + "/record/download?name=s1", timeout=10) as r:
            self.assertEqual(r.headers["Content-Type"], "application/gzip")
            self.assertEqual(len(r.read()), 5 * (3 * 20000 + 1))

    def test_mav_has_the_documented_shape(self):
        mav = get_json(self.base + "/mav")
        self.assertEqual(set(mav), {"jetson_host", "jetson_ok", "jetson_age", "jetson_error", "link", "hb",
                                    "pos", "gps", "rc", "batt", "ekf", "prearm", "statustext"})
        self.assertEqual(mav["jetson_host"], LOCAL)
        self.assertEqual((mav["hb"]["mode"], mav["hb"]["armed"]), ("AUTO", True))
        self.assertEqual((mav["rc"]["estop_pwm"], mav["rc"]["estop_engaged"]), (1999, False))
        self.assertIsNone(mav["batt"]["voltage"])

    def test_a_request_from_another_site_is_refused_and_nothing_is_forwarded(self):
        before = len(self.jetson.actions)
        for headers in ({"Origin": "http://evil.example"}, {"Origin": "null"}):
            with self.subTest(headers), self.assertRaises(urllib.error.HTTPError) as cm:
                post_json(self.base + "/power", {"verb": "shutdown"}, headers)
            self.assertEqual(cm.exception.code, 403)
        conn = http.client.HTTPConnection(LOCAL, self.port, timeout=5)           # DNS rebinding
        conn.putrequest("POST", "/power", skip_host=True)
        conn.putheader("Host", "evil.example:%d" % self.port)
        conn.putheader("Content-Length", "2")
        conn.endheaders(b"{}")
        self.assertEqual(conn.getresponse().status, 403)
        conn.close()
        self.assertEqual(len(self.jetson.actions), before)

    def test_the_pages_own_origin_and_plain_clients_pass(self):
        self.assertTrue(post_json(self.base + "/trail/clear", {}, {"Origin": self.base})["ok"])
        self.assertTrue(post_json(self.base + "/trail/clear", {})["ok"])

    def test_jetson_down_the_state_has_no_snapshot_and_mav_still_answers(self):
        link = dead_link()
        server = gl.LaptopServer(gl.build_page(200), link, lambda: gl.mav_status(link, self.state, 3.0))
        server.start(0, LOCAL)
        base = "http://%s:%d" % (LOCAL, server._server.server_address[1])
        self.addCleanup(server.stop)
        state = get_json(base + "/state")
        self.assertEqual(list(state), ["error"])                    # the page shows its banner for this
        self.assertIn("unreachable", state["error"])
        out = post_json(base + "/node/stop", {"name": "x"})
        self.assertFalse(out["ok"])
        self.assertIn("nothing was sent", out["message"])
        mav = get_json(base + "/mav")
        self.assertIs(mav["jetson_ok"], False)
        self.assertEqual(mav["hb"]["mode"], "AUTO")                 # the strip keeps working


class OriginRule(unittest.TestCase):
    def test_request_allowed(self):
        allowed = gl.allowed_hosts_for("127.0.0.1", 8150)
        self.assertEqual(allowed, {"localhost:8150", "127.0.0.1:8150", "[::1]:8150"})
        cases = (("localhost:8150", None, True), ("127.0.0.1:8150", "http://127.0.0.1:8150", True),
                 ("localhost:8150", "http://localhost:8150", True), ("LOCALHOST:8150", None, True),
                 ("evil.example:8150", None, False), ("localhost:9999", None, False), (None, None, False),
                 ("localhost:8150", "http://evil.example", False), ("localhost:8150", "null", False),
                 ("localhost:8150", "http://127.0.0.1:8150", False))
        for host, origin, want in cases:
            with self.subTest(host=host, origin=origin):
                self.assertIs(gl.request_allowed(host, origin, allowed), want)

    def test_bound_beyond_loopback_the_host_check_is_off_but_the_origin_check_stays(self):
        self.assertIsNone(gl.allowed_hosts_for("0.0.0.0", 8150))
        self.assertTrue(gl.request_allowed("192.168.1.5:8150", "http://192.168.1.5:8150", None))
        self.assertFalse(gl.request_allowed("192.168.1.5:8150", "http://evil.example", None))

    def test_port_80_may_arrive_without_a_port(self):
        self.assertIn("localhost", gl.allowed_hosts_for("127.0.0.1", 80))
        self.assertNotIn("localhost", gl.allowed_hosts_for("127.0.0.1", 8150))


class Announcements(unittest.TestCase):
    class FakeState:
        sysid, source = 2, "udpin:test"

        def __init__(self):
            self.ok = False

        def snapshot(self, now):
            return {"link": {"ok": self.ok}}

    class FakeLink:
        def __init__(self):
            self.ok = None

        def status(self, stale_s):
            return {"jetson_ok": self.ok}

        def candidates_text(self):
            return "a:1, b:2"

    def run_for(self, state, link, script):
        said, stop = [], threading.Event()
        t = threading.Thread(target=gl.announce_changes, args=(state, link, 3.0, stop, lambda m, **k: said.append(m)),
                             kwargs={"every": 0.01}, daemon=True)
        t.start()
        for change in script:
            time.sleep(0.15)
            change()
        time.sleep(0.15)
        stop.set()
        t.join(3)
        return said

    def test_it_speaks_on_a_change_only_and_stays_silent_when_unknown(self):
        state, link = self.FakeState(), self.FakeLink()
        said = self.run_for(state, link, [lambda: setattr(link, "ok", False),
                                          lambda: setattr(state, "ok", True),
                                          lambda: setattr(link, "ok", True),
                                          lambda: setattr(link, "ok", None)])
        self.assertEqual(len(said), 4, said)
        self.assertIn("no MAVLink from system 2 on udpin:test", said[0])
        self.assertIn("Jetson API is not answering (a:1, b:2)", said[1])
        self.assertIn("MAVLink is arriving", said[2])
        self.assertIn("Jetson API answers", said[3])


# ---------------------------------------------------------------- the launcher and the whole program

class CommandLine(unittest.TestCase):
    def parse(self, *argv):
        return gl.build_parser().parse_args(list(argv))

    def test_defaults(self):
        a = self.parse()
        self.assertEqual((a.port, a.bind, a.mav_source, a.sysid, a.jetson, a.poll_ms, a.stale_s),
                         (8150, "127.0.0.1", "udpin:127.0.0.1:14554", 2, "auto", None, 3.0))

    def test_a_bad_value_is_a_usage_error_not_a_traceback(self):
        for argv in (["--jetson", "host:port"], ["--jetson", "host:70000"], ["--port", "70000"], ["--port", "x"],
                     ["--sysid", "0"], ["--sysid", "256"], ["--poll-ms", "0"]):
            with self.subTest(argv), contextlib.redirect_stderr(io.StringIO()) as err:
                with self.assertRaises(SystemExit) as cm:
                    self.parse(*argv)
                self.assertEqual(cm.exception.code, 2)
                self.assertIn("error:", err.getvalue())                     # argparse said why

    def test_the_jetson_may_be_auto_or_a_host(self):
        self.assertEqual(self.parse("--jetson", "AUTO").jetson, "AUTO")
        self.assertEqual(self.parse("--jetson", "192.168.8.109").jetson, "192.168.8.109")
        self.assertEqual(self.parse("--jetson", "http://10.0.0.5:8090/").jetson, "http://10.0.0.5:8090/")


class Launcher(unittest.TestCase):
    PATHS = (os.path.join(SCRIPTS, "BOAT_GUI.cmd"), os.path.abspath(os.path.join(REPO, "..", "..", "BOAT_GUI.cmd")))

    def test_cmd_files_are_crlf_only(self):
        for path in self.PATHS:
            if not os.path.isfile(path):
                continue
            with self.subTest(path):
                raw = read_file(path, "rb")
                self.assertEqual(raw.count(b"\n"), raw.count(b"\r\n"), "bare LF breaks cmd.exe label jumps")

    def test_it_probes_with_curl_and_does_nothing_this_pcs_antivirus_deletes_files_for(self):
        code = [ln for ln in read_file(self.PATHS[0], encoding="ascii").splitlines()
                if not ln.lstrip().lower().startswith("rem ")]
        text = "\n".join(code).lower()
        self.assertIn("curl.exe", text)
        for forbidden in ("invoke-webrequest", "start-process", "powershell"):
            self.assertNotIn(forbidden, text)

    def test_the_options_it_mentions_are_the_programs_real_ones(self):
        text = read_file(self.PATHS[0], encoding="ascii")
        known = {opt for action in gl.build_parser()._actions for opt in action.option_strings}
        mentioned = set(re.findall(r"(?<![\w-])--[a-z][a-z-]*", text))
        self.assertIn("--port", mentioned)
        self.assertEqual(mentioned - known, set())
        example = re.search(r"BOAT_GUI\.cmd( --\S.*)", text).group(1).split()
        self.assertTrue(gl.build_parser().parse_args(example))                  # the documented example parses

    def test_the_root_launcher_just_calls_the_one_in_tools_scripts(self):
        root = self.PATHS[1]
        if os.path.isfile(root):
            self.assertEqual(read_file(root).strip(),
                             r'@call "%~dp0Boat\rx26_asv\tools\scripts\BOAT_GUI.cmd" %*')
            self.assertTrue(os.path.isfile(self.PATHS[0]))


class WholeProgram(unittest.TestCase):
    """main() in a child process: starts, serves, reads UDP MAVLink, stops cleanly on Ctrl+C."""

    CHILD = ("import _thread, sys, threading\n"
             "sys.path.insert(0, %r)\n"
             "import gcs_laptop\n"
             "def wait_for_stdin():\n"
             "    sys.stdin.readline()\n"
             "    _thread.interrupt_main()\n"
             "threading.Thread(target=wait_for_stdin, daemon=True).start()\n"
             "sys.exit(gcs_laptop.main(%r))\n")

    def test_starts_serves_listens_and_stops(self):
        jetson = FakeJetson()
        self.addCleanup(jetson.stop)
        page_port, mav_port = free_port(), free_port()
        argv = ["--port", str(page_port), "--mav", "udpin:%s:%d" % (LOCAL, mav_port),
                "--jetson", "%s:%d" % (LOCAL, jetson.port)]
        proc = subprocess.Popen([sys.executable, "-u", "-c", self.CHILD % (SCRIPTS, argv)],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.addCleanup(lambda: proc.poll() is None and proc.kill())
        base = "http://%s:%d" % (LOCAL, page_port)
        deadline = time.monotonic() + 20
        mav = None
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(sender.close)
        while time.monotonic() < deadline:
            sender.sendto(encode("HEARTBEAT", type=11, autopilot=3, base_mode=193, custom_mode=15),
                          (LOCAL, mav_port))
            try:
                mav = get_json(base + "/mav")
                if mav["hb"]["ok"]:
                    break
            except OSError:
                pass
            time.sleep(0.2)
        self.assertIsNotNone(mav, "the program never came up")
        self.assertEqual((mav["hb"]["mode"], mav["hb"]["armed"], mav["jetson_ok"] in (True, None)), ("GUIDED", True, True))
        self.assertEqual(get_json(base + "/state"), FakeJetson.SNAPSHOT)
        proc.stdin.write("\n")
        proc.stdin.flush()
        out = proc.communicate(timeout=20)[0]
        self.assertEqual(proc.returncode, 0, out)
        for expect in ("http://localhost:%d/" % page_port, "LISTEN-ONLY", "RC ch7", "explicit", "stopped"):
            self.assertIn(expect, out)

    def test_a_port_already_in_use_is_refused_with_a_message(self):
        with socket.socket() as busy:
            busy.bind((LOCAL, 0))
            busy.listen(1)
            port = busy.getsockname()[1]
            r = subprocess.run([sys.executable, os.path.join(SCRIPTS, "gcs_laptop.py"), "--port", str(port),
                                "--mav", "udpin:%s:%d" % (LOCAL, free_port())],
                               capture_output=True, text=True, timeout=30)
        self.assertEqual(r.returncode, 2)
        self.assertIn("already in use", r.stderr)


if __name__ == "__main__":
    unittest.main()
