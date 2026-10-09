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
        self.assertNotRegex(gl.STRIP_CSS, r"#[0-9a-fA-F]{3,8}\b|rgb\(|hsl\(")
        for name in set(re.findall(r"var\((--[\w-]+)\)", gl.STRIP_CSS)):
            self.assertIn(name + ":", PAGE, "the strip uses %s, which gcs_page.py does not define" % name)

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
        self.assertEqual(written, [], "the reader wrote to a socket")


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
                                    "pos", "gps", "rc", "batt", "statustext"})
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
