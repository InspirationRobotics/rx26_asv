#!/usr/bin/env python3
"""gcs_laptop -- Crusader's ground station page, served from THIS laptop.

    python tools/scripts/gcs_laptop.py            # or double-click BOAT_GUI.cmd
    # then open http://localhost:8150

WHY THIS EXISTS. The page the boat serves (`ground_station` on the Jetson, :8090)
is only as reachable as the Jetson is: browse to it and a Jetson that is slow,
rebooting or on the other subnet takes the whole window with it. This program
serves the SAME page from the laptop, so the window stays up and says plainly
which half has gone quiet:

    the Jetson's :8090   -> everything the page already shows: nodes, map,
                            targets, logs, tuning, recording, power. /state is
                            fetched from it and every button is forwarded to it.
    MAVLink on this laptop -> a strip across the top of the page: link, mode,
                            armed, the RC e-stop, GPS, battery, autopilot
                            warnings. The Jetson's page reads NO MAVLink at all.
    the Jetson's viewers -> camera :8080, LiDAR :8081 and the Task 1 panel :8095
                            stay on the Jetson. The page frames them at the
                            Jetson's address (window.VIEW_HOST), so no video
                            ever passes through this program.

It serves the REAL page (`crusader_groundstation.gcs_page`) through the REAL
server (`gcs_server.GcsServer`), imported rather than copied, so what an
operator learns here is true of the boat's own page. gcs_page.py is patched in
memory, never on disk, and the patcher refuses to run if the page no longer
looks the way the patches expect -- see patch_page.

THIS PROGRAM NEVER TRANSMITS MAVLINK. Not a heartbeat, not a request, not a
SET_MESSAGE_INTERVAL. The autopilot's port-0 stream rates feed the boat's own
telemetry_bridge, and changing them from a laptop breaks the boat. It is
enforced structurally: the reader is handed ListenOnly, which exposes
recv_match and close and nothing else, and test_gcs_laptop.py spies on the
socket layer to prove that nothing is ever written. Do not add a send.

WHERE THE MAVLINK COMES FROM. The boat's MAVProxy unicasts the autopilot stream
to this laptop's UDP 14550. On the lab laptop the OCS fleet router owns 14550 and
forwards the boat's frames, listen-only, to 127.0.0.1:14554 -- the default here.
With no router running, `--mav udpin:0.0.0.0:14550` reads the stream directly (and
STEALS the port from QGroundControl: a udpin bind takes the datagrams, the other
program sees silence rather than an error).

BLANKS OVER GUESSES. A value older than --stale-s is null with its age kept; the
strip shows a dash. Battery voltage 0 or 65535 is "no sensor", never "0 V"
(BATT_MONITOR is 0 on this boat). A GPS yaw of 65535 is "configured but
unavailable", 0 is "not provided". The e-stop keeps the vehicle ARMED, so it is
invisible in HEARTBEAT and is read from RC_CHANNELS: SB (ch7) below 1200 us is
e-stop engaged OR RC lost (a lost RC reads 0), and the two cannot be told apart.
"""
import argparse
import collections
import contextlib
import http.client
import json
import math
import os
import socket
import sys
import threading
import time
import urllib.parse
from typing import NamedTuple

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
_PKG_ROOT = os.path.join(REPO, "crusader_groundstation")
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

os.environ.setdefault("MAVLINK20", "1")     # before pymavlink: the boat sends v2
import yaml                                                         # noqa: E402
from pymavlink import mavutil                                       # noqa: E402

from crusader_groundstation import gcs_page                         # noqa: E402
from crusader_groundstation.gcs_server import MAX_BODY, GcsServer   # noqa: E402

PARAMS_YAML = os.path.join(REPO, "crusader_bringup", "config", "crusader_params.yaml")

# ---- the Jetson -----------------------------------------------------------
JETSON_PORT = 8090
#: Where the Jetson has been seen: its wired address (lab cable, WiFi "MESAFSD")
#: and its address on the Rocket/Bullet network. Tried in this order.
JETSON_CANDIDATES = (("192.168.100.109", JETSON_PORT), ("192.168.8.109", JETSON_PORT))
PROBE_TIMEOUT_S = 1.5       # one candidate, at startup or when re-probing
REPROBE_S = 5.0             # between probe rounds while the Jetson is not answering
CONNECT_TIMEOUT_S = 3.0     # until the TCP connection is made (nothing sent yet)
STATE_TIMEOUT_S = 2.0       # GET /state, connect and each read
ACTION_TIMEOUT_S = 15.0     # POST: the reply to an action may take a while
CHUNK = 64 * 1024
#: What a call to the Jetson can raise once it is under way.
HTTP_ERRORS = (OSError, http.client.HTTPException)

# ---- MAVLink --------------------------------------------------------------
AUTOPILOT_COMPID = 1
UINT16_MAX = 65535
UINT8_MAX = 255
MAX_SEVERITY = 4            # MAV_SEVERITY_WARNING; lower is worse
KEEP_STATUSTEXT = 3
FPS_WINDOW_S = 5.0
RECV_TIMEOUT_S = 0.5
DEFAULT_MAV = "udpin:127.0.0.1:14554"
DEFAULT_POLL_MS = 200       # what ground_station.poll_period_s is on the boat
LOOPBACK_BINDS = ("127.0.0.1", "localhost", "::1")


class ConfigError(RuntimeError):
    """crusader_params.yaml does not say what this program needs."""


class PagePatchError(RuntimeError):
    """gcs_page.py changed upstream and a patch no longer matches."""


class JetsonError(Exception):
    """A call to the Jetson failed. `sent` is True when the request may have
    reached it before the failure, which decides what a POST caller is told."""

    def __init__(self, message, sent=False):
        super().__init__(message)
        self.sent = sent


# ===========================================================================
# crusader_params.yaml
# ===========================================================================

class Params(NamedTuple):
    estop_channel: int
    estop_threshold: int
    poll_ms: int


def _ros_block(doc, node, path, required=True):
    """doc[node]['ros__parameters'] as a dict; {} when absent and not required."""
    entry = doc.get(node)
    block = entry.get("ros__parameters") if isinstance(entry, dict) else None
    if isinstance(block, dict):
        return block
    if required or entry is not None:
        raise ConfigError("%s has no `%s: ros__parameters:` block" % (path, node))
    return {}


def _int_param(block, key, node, path, lo, hi):
    value = block.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or not lo <= value <= hi:
        raise ConfigError("%s: %s.%s must be an integer in %d..%d, found %r"
                          % (path, node, key, lo, hi, value))
    return value


def load_params(path=PARAMS_YAML):
    """The e-stop channel and threshold, and the page's poll period.

    estop_* come from telemetry_bridge's block; pixhawk_led_status_node carries
    its own copy that the YAML says must equal it, and a disagreement is an
    error here, not a choice -- the strip would otherwise call a state RUN that
    the boat's own LED calls e-stopped. A missing key is also an error: the e-stop
    is the one thing this strip must never guess.
    """
    try:
        with open(path, encoding="utf-8") as f:
            doc = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        raise ConfigError("cannot read %s: %s" % (path, e)) from e
    if not isinstance(doc, dict):
        raise ConfigError("%s is not a ROS parameter file" % path)
    bridge = _ros_block(doc, "telemetry_bridge", path)
    channel = _int_param(bridge, "estop_channel", "telemetry_bridge", path, 1, 18)
    threshold = _int_param(bridge, "estop_threshold", "telemetry_bridge", path, 800, 2200)
    led = _ros_block(doc, "pixhawk_led_status_node", path, required=False)
    for key, value in (("estop_channel", channel), ("estop_threshold", threshold)):
        if led and led.get(key) != value:
            raise ConfigError("%s: telemetry_bridge.%s is %r but pixhawk_led_status_node.%s is %r;"
                              " they must be the same switch" % (path, key, value, key, led.get(key)))
    # The poll period is optional: a YAML without it gets the boat's own default.
    period = _ros_block(doc, "ground_station", path, required=False).get("poll_period_s")
    if period is None:
        poll_ms = DEFAULT_POLL_MS
    elif isinstance(period, (int, float)) and not isinstance(period, bool) and period > 0:
        poll_ms = max(1, int(round(period * 1000)))
    else:
        raise ConfigError("%s: ground_station.poll_period_s must be a positive number, found %r"
                          % (path, period))
    return Params(channel, threshold, poll_ms)


# ===========================================================================
# The page, patched
# ===========================================================================

#: Small on purpose -- one or two lines on a laptop -- and made only of the
#: page's own CSS variables, so day mode (html[data-theme="day"]) recolours it
#: with everything else. A hardcoded colour here is a strip that stays dark in
#: sunlight.
STRIP_CSS = r"""
 /* mavstrip: the laptop's own MAVLink readout (gcs_laptop.py). */
 #mavstrip{flex:none;padding:3px 10px;background:var(--panel);
   border-bottom:1px solid var(--line);font-size:12px;line-height:1.5}
 #mavstrip .mv-row{display:flex;flex-wrap:wrap;align-items:baseline;gap:0 10px}
 #mavstrip .mv-t{color:var(--dim);font-weight:600}
 #mavstrip .mv-k{color:var(--dim);margin-right:5px}
 #mavstrip .mv-strong{color:var(--strong)}
 #mavstrip .mv-ok{color:var(--ok)}
 #mavstrip .mv-warn{color:var(--warn)}
 #mavstrip .mv-bad{color:var(--bad);font-weight:600}
 #mavstrip .mv-dim{color:var(--dim)}
"""

#: Polls GET /mav (served here, not by the Jetson) with one request in flight
#: at a time -- the page's own poll() explains why. Built with textContent only:
#: STATUSTEXT is text from another machine. It also hands the page the Jetson's
#: address, which the viewer frames use in place of location.hostname.
STRIP_JS = r"""
(function(){
"use strict";
var strip = document.getElementById('mavstrip');
if(!strip) return;
var DASH = '—', busy = false, first = true;

function node(tag, cls, text){
  var n = document.createElement(tag);
  if(cls) n.className = cls;
  if(text !== undefined && text !== null) n.textContent = text;
  return n;
}
function num(v, digits, unit){
  return (v === null || v === undefined) ? DASH : Number(v).toFixed(digits) + (unit || '');
}
function item(row, label, value, cls, tip){
  var s = node('span', 'mv-i');
  if(tip) s.title = tip;
  if(label) s.appendChild(node('span', 'mv-k', label));
  s.appendChild(node('span', cls || '', value));
  row.appendChild(s);
}
function ageTip(sec){
  return (sec.age === null || sec.age === undefined) ? 'never received'
                                                      : 'last received ' + sec.age.toFixed(1) + ' s ago';
}

function showLink(row, L){
  if(L.ok){
    item(row, 'link', num(L.age, 1, ' s') + ' · ' + num(L.fps, 0, ' fps'), 'mv-ok',
         'frames from system ' + L.sysid + ' on ' + L.source);
    return;
  }
  var why = (L.age === null || L.age === undefined) ? 'no frames yet' : 'silent ' + num(L.age, 0, ' s');
  if(L.other_sysids && L.other_sysids.length) why += ' (heard system ' + L.other_sysids.join(', ') + ' instead)';
  item(row, 'link', why, 'mv-bad', L.error || ('nothing from system ' + L.sysid + ' on ' + L.source));
}
/* One value of a section: its text and colour only while the section is fresh, a dim
   dash otherwise -- never the last value. `value` is computed either way, so it must
   tolerate a section with nothing in it (hence == null, which is also true of undefined). */
function fresh(row, label, sec, value, cls, tip){
  item(row, label, sec.ok ? value : DASH, sec.ok ? cls : 'mv-dim', tip || ageTip(sec));
}
function showMode(row, H){
  fresh(row, 'mode', H, H.mode, 'mv-strong');
  fresh(row, '', H, H.armed ? 'ARMED' : 'disarmed', H.armed ? 'mv-warn' : 'mv-dim');
}
function showEstop(row, R){
  var tip = 'RC channel ' + R.estop_channel + ' (SB switch); below ' + R.estop_threshold +
            ' us is e-stop engaged OR RC lost. ' + ageTip(R);
  var pwm = (R.estop_pwm == null) ? '' : ' ' + R.estop_pwm + ' µs';
  if(R.ok && R.estop_engaged === true) item(row, 'e-stop', 'ENGAGED / RC LOST' + pwm, 'mv-bad', tip);
  else if(R.ok && R.estop_engaged === false) item(row, 'e-stop', 'RUN' + pwm, 'mv-ok', tip);
  else item(row, 'e-stop', DASH, 'mv-dim', tip);
}
function showGps(row, G){
  var Y = {ok: ['ok ' + num(G.yaw_deg, 0, '°'), 'mv-ok'],
           unavailable: ['UNAVAILABLE', 'mv-bad'],
           not_provided: ['not provided', 'mv-warn']}[G.yaw_state] || [DASH, 'mv-dim'];
  fresh(row, 'GPS', G, G.fix_name + ' · ' + (G.sats == null ? DASH : G.sats) + ' sat · ' +
        num(G.h_acc_m, 2, ' m'), G.fix_type >= 3 ? 'mv-ok' : 'mv-bad');
  fresh(row, 'GPS yaw', G, Y[0], Y[1],
        'the heading source: the compass is disabled on this boat. ' + ageTip(G));
}
function showPos(row, P){
  fresh(row, 'pos', P, P.lat == null ? 'no position' : P.lat.toFixed(7) + ', ' + P.lon.toFixed(7), '');
  fresh(row, '', P, num(P.groundspeed, 1, ' m/s') + ' · hdg ' + num(P.heading, 0, '°'), '');
}
function showBatt(row, B){
  fresh(row, 'batt', B, B.voltage == null ? 'no sensor' : B.voltage.toFixed(2) + ' V',
        B.voltage == null ? 'mv-dim' : '',
        'as the autopilot reports it; 0 V and 65535 mean no sensor. See the crusader-battery skill before trusting a number. ' + ageTip(B));
}
function showJetson(row, j){
  var ok = j.jetson_ok;
  item(row, 'Jetson API', (j.jetson_host || 'no Jetson') + ' ' + (ok === true ? 'ok' : ok === false ? 'UNREACHABLE' : DASH),
       ok === true ? 'mv-ok' : ok === false ? 'mv-bad' : 'mv-dim', j.jetson_error || '');
}
function showTexts(row, list){
  list.forEach(function(t){
    item(row, '', '[' + (t.severity_name || t.severity) + '] ' + t.text + ' (' + num(t.age, 0, ' s') + ' ago)',
         t.severity <= 3 ? 'mv-bad' : 'mv-warn');
  });
}

function show(j, problem){
  var top = node('div', 'mv-row'), texts = node('div', 'mv-row');
  top.appendChild(node('span', 'mv-t', 'AUTOPILOT VIA WIFI (laptop MAVLink)'));
  if(!j){
    var waiting = first && !problem;
    item(top, '', problem || (waiting ? 'waiting for the first answer' : 'NO ANSWER FROM THE LAPTOP SERVER (gcs_laptop.py)'),
         waiting ? 'mv-dim' : 'mv-bad');
  } else {
    showLink(top, j.link || {});
    showMode(top, j.hb || {});
    showEstop(top, j.rc || {});
    showGps(top, j.gps || {});
    showPos(top, j.pos || {});
    showBatt(top, j.batt || {});
    showJetson(top, j);
    showTexts(texts, j.statustext || []);
  }
  strip.textContent = '';
  strip.appendChild(top);
  if(texts.childNodes.length) strip.appendChild(texts);
}

function poll(){
  if(busy) return;
  busy = true;
  fetch('/mav', {cache: 'no-store'}).then(function(r){
    if(!r.ok) throw new Error('HTTP ' + r.status);
    return r.json();
  }).then(function(j){
    busy = false; first = false;
    if(j.error){ show(null, j.error); return; }
    if(typeof j.jetson_host === 'string' && j.jetson_host) window.VIEW_HOST = j.jetson_host;
    show(j);
  }).catch(function(){
    busy = false; first = false;
    show(null);
  });
}
show(null);
poll();
setInterval(poll, 500);
})();
"""

#: What the page's red banner says when its /state fetch fails. In this copy that has two
#: causes the page cannot tell apart: the Jetson is not answering (LaptopServer then serves
#: an {"error": ...} body, which the page cannot render and treats as a failed fetch), or
#: this program has stopped. The strip, which is separate, says which. No double quotes in
#: it: it goes into the page as a JavaScript string.
BANNER_TEXT = ("NO CONNECTION TO THE BOAT'S JETSON (ground station API) - or to this laptop copy "
               "of the page. The strip above says which.")

#: (what it is, text in gcs_page.py, replacement, how many times it must occur).
#: The count is the point. gcs_page.py is shared with the Jetson and changes
#: upstream; a pattern that stops matching, or starts matching somewhere new
#: (a third viewer frame using location.hostname would frame the laptop), must be
#: a loud failure here, not a page that quietly points at the wrong machine.
PAGE_PATCHES = (
    ("viewer and Task 1 frame host", "location.hostname",
     "(window.VIEW_HOST||location.hostname)", 2),
    ("banner text", "'NO CONNECTION TO THE BOAT'", '"' + BANNER_TEXT + '"', 1),
    # Before <main>, not after the banner: the banner is absolutely positioned
    # INSIDE main, and a strip there would overlay the map's scale bar and the
    # Task 1 frame. Here it is a flex row of #app and main simply gets shorter.
    ("strip markup", "<main>", '<div id="mavstrip"></div>\n <main>', 1),
    ("strip style", "</style>", STRIP_CSS + "</style>", 1),
    ("strip script", "</script></body></html>",
     "</script>\n<script>" + STRIP_JS + "</script></body></html>", 1),
)


def patch_page(html):
    """gcs_page's HTML with the laptop changes applied, or PagePatchError.

    Every pattern is counted on the UNTOUCHED page first, and all the mismatches
    are reported together, so one upstream edit does not cost one run per patch.
    """
    problems = ["%s: %r occurs %d time(s), expected %d" % (what, old, html.count(old), want)
                for what, old, _new, want in PAGE_PATCHES if html.count(old) != want]
    if problems:
        raise PagePatchError("gcs_page.py no longer matches what gcs_laptop.py patches. Update "
                             "PAGE_PATCHES (and re-check the page in a browser):\n  "
                             + "\n  ".join(problems))
    for _what, old, new, _want in PAGE_PATCHES:
        html = html.replace(old, new)
    return html


def build_page(poll_ms):
    return patch_page(gcs_page.render(poll_ms).decode("utf-8")).encode("utf-8")


# ===========================================================================
# MAVLink: listen-only
# ===========================================================================

def _enum_name(enum, value, prefix):
    """'RTK_FIXED' for GPS_FIX_TYPE 6: the name comes from the dialect, not from
    a table kept here that could drift from it."""
    entry = mavutil.mavlink.enums[enum].get(value)
    return entry.name[len(prefix):].replace("_", " ") if entry else None


class ListenOnly:
    """The only door the reader has to the autopilot link: receive, and nothing
    else. `.mav`, `.write` and every *_send of the underlying connection are
    unreachable from here on purpose -- see the module docstring."""

    def __init__(self, connection):
        self._conn = mavutil.mavlink_connection(connection, robust_parsing=True,
                                                dialect="ardupilotmega")

    def recv_match(self, timeout):
        return self._conn.recv_match(blocking=True, timeout=timeout)

    def close(self):
        self._conn.close()


#: section -> the value names it carries. A stale or never-heard section has all
#: of them null; `ok` and `age` ride beside them.
SECTIONS = {
    "hb": ("mode", "armed", "type"),
    "pos": ("lat", "lon", "heading", "groundspeed"),
    "gps": ("fix_type", "fix_name", "sats", "h_acc_m", "yaw_state", "yaw_deg"),
    "rc": ("estop_pwm", "estop_engaged", "rssi"),
    "batt": ("voltage", "current", "remaining"),
}


class MavState:
    """What the autopilot last said, with the age of each thing.

    `feed` takes a decoded message and the time it arrived (time.monotonic());
    `snapshot` takes "now" and returns the /mav body. Both are pure apart from
    the lock, so the tests drive them with invented clocks and encoded messages.

    Only the AUTOPILOT counts for values: system `sysid`, component 1. The link
    block alone counts every component of that system, so a silent autopilot
    behind a chatty companion shows a live link and a dead heartbeat, which is
    the true picture. A HEARTBEAT of type GCS is never an autopilot.
    """

    def __init__(self, sysid, estop_channel, estop_threshold, stale_s=3.0, source=""):
        self.sysid, self.stale_s, self.source = sysid, stale_s, source
        self.estop_channel, self.estop_threshold = estop_channel, estop_threshold
        self._lock = threading.Lock()
        self._rec = {}                       # section -> {"t": monotonic, **values}
        self._texts = collections.deque(maxlen=KEEP_STATUSTEXT)
        self._frames = collections.deque()   # arrival times of frames from sysid
        self._first = self._last = None
        self._others = {}                    # other sysid -> last time heard
        self._error = None
        self._handlers = {"HEARTBEAT": self._on_heartbeat, "GLOBAL_POSITION_INT": self._on_position,
                          "GPS_RAW_INT": self._on_gps, "RC_CHANNELS": self._on_rc,
                          "SYS_STATUS": self._on_sys_status}

    # ------------------------------------------------------------- feeding

    def feed(self, msg, now):
        typ = msg.get_type()
        if typ == "BAD_DATA":
            return
        src = msg.get_srcSystem()
        with self._lock:
            if src != self.sysid:
                self._others[src] = now
                return
            self._error = None
            if self._first is None:
                self._first = now
            self._last = now
            self._frames.append(now)
            while self._frames and now - self._frames[0] > FPS_WINDOW_S:
                self._frames.popleft()
            if msg.get_srcComponent() != AUTOPILOT_COMPID:
                return
            if typ == "STATUSTEXT":
                self._on_statustext(msg, now)
                return
            handler = self._handlers.get(typ)
            values = handler(msg) if handler else None
            if values is not None:
                self._rec[values.pop("section")] = dict(values, t=now)

    def note_error(self, error):
        """The reader hit an exception; the strip says why the link is quiet."""
        with self._lock:
            self._error = "%s: %s" % (type(error).__name__, error)

    def _on_heartbeat(self, msg):
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            return None
        return {"section": "hb", "mode": mavutil.mode_string_v10(msg), "type": msg.type,
                "armed": bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}

    def _on_position(self, msg):
        placed = not (msg.lat == 0 and msg.lon == 0)        # 0,0 is "no position"
        return {"section": "pos", "lat": msg.lat / 1e7 if placed else None,
                "lon": msg.lon / 1e7 if placed else None,
                "heading": None if msg.hdg == UINT16_MAX else msg.hdg / 100.0,
                "groundspeed": math.hypot(msg.vx, msg.vy) / 100.0}

    def _on_gps(self, msg):
        # yaw is cdeg: 0 not provided, 65535 configured but unavailable (the
        # boat's heading comes from dual-antenna GPS yaw), else 1..36000.
        if msg.yaw == UINT16_MAX:
            state, deg = "unavailable", None
        elif msg.yaw == 0:
            state, deg = "not_provided", None
        elif msg.yaw <= 36000:
            state, deg = "ok", (msg.yaw / 100.0) % 360.0
        else:
            state, deg = None, None
        return {"section": "gps", "fix_type": msg.fix_type,
                "fix_name": _enum_name("GPS_FIX_TYPE", msg.fix_type, "GPS_FIX_TYPE_"),
                "sats": None if msg.satellites_visible == UINT8_MAX else msg.satellites_visible,
                "h_acc_m": msg.h_acc / 1000.0 if msg.h_acc else None,
                "yaw_state": state, "yaw_deg": deg}

    def _on_rc(self, msg):
        raw = getattr(msg, "chan%d_raw" % self.estop_channel)
        pwm = None if raw == UINT16_MAX else raw
        if msg.chancount == 0:                   # no RC channels at all: RC lost
            engaged = True
        elif pwm is None:                        # channel not reported: unknown
            engaged = None
        else:
            engaged = pwm < self.estop_threshold     # a lost RC reads 0, below it
        return {"section": "rc", "estop_pwm": pwm, "estop_engaged": engaged,
                "rssi": None if msg.rssi == UINT8_MAX else msg.rssi}

    def _on_sys_status(self, msg):
        return {"section": "batt",
                "voltage": (None if msg.voltage_battery in (0, UINT16_MAX)
                            else msg.voltage_battery / 1000.0),
                "current": None if msg.current_battery < 0 else msg.current_battery / 100.0,
                "remaining": None if msg.battery_remaining < 0 else msg.battery_remaining}

    def _on_statustext(self, msg, now):
        if msg.severity > MAX_SEVERITY:
            return
        text = msg.text.decode("utf-8", "replace") if isinstance(msg.text, bytes) else str(msg.text)
        text = text.split("\x00", 1)[0]          # chunks keep their own spaces; the snapshot trims
        # A long message arrives as chunks that share a non-zero id; join them.
        last = self._texts[-1] if self._texts else None
        if msg.id and last and last["id"] == msg.id:
            last["text"] += text
        else:
            self._texts.append({"t": now, "severity": msg.severity, "text": text, "id": msg.id})

    # ------------------------------------------------------------ snapshot

    def snapshot(self, now):
        with self._lock:
            out = {"link": self._link(now)}
            for name in SECTIONS:
                out[name] = self._section(name, now)
            out["rc"]["estop_channel"] = self.estop_channel          # configuration,
            out["rc"]["estop_threshold"] = self.estop_threshold      # never stale
            out["statustext"] = [
                {"age": round(now - t["t"], 1), "severity": t["severity"],
                 "severity_name": _enum_name("MAV_SEVERITY", t["severity"], "MAV_SEVERITY_"),
                 "text": t["text"].strip()} for t in reversed(self._texts)]
            return out

    def _section(self, name, now):
        rec = self._rec.get(name)
        age = None if rec is None else now - rec["t"]
        ok = age is not None and age <= self.stale_s
        out = {"ok": ok, "age": None if age is None else round(age, 2)}
        out.update({field: (rec[field] if ok else None) for field in SECTIONS[name]})
        return out

    def _link(self, now):
        age = None if self._last is None else now - self._last
        ok = age is not None and age <= self.stale_s
        while self._frames and now - self._frames[0] > FPS_WINDOW_S:
            self._frames.popleft()
        span = min(FPS_WINDOW_S, max(now - self._first, 1.0)) if self._first is not None else None
        return {"source": self.source, "sysid": self.sysid, "ok": ok,
                "age": None if age is None else round(age, 2),
                "fps": round(len(self._frames) / span, 1) if ok else None,
                "other_sysids": sorted(s for s, t in self._others.items() if now - t <= self.stale_s),
                "error": None if ok else self._error}


def read_loop(source, state, stop, clock=time.monotonic):
    """Feed `state` from `source` until `stop` is set. Receives only."""
    while not stop.is_set():
        try:
            msg = source.recv_match(RECV_TIMEOUT_S)
        except Exception as e:                                       # noqa: BLE001
            state.note_error(e)
            stop.wait(0.5)
            continue
        if msg is not None:
            state.feed(msg, clock())


# ===========================================================================
# The Jetson
# ===========================================================================

def parse_target(spec, default_port=JETSON_PORT):
    """'host', 'host:port' or 'http://host:port/' -> (host, port)."""
    s = spec.strip()
    if s.lower().startswith("http://"):
        s = s[len("http://"):]
    s = s.rstrip("/")
    host, port = s, default_port
    if ":" in s:
        host, _, tail = s.rpartition(":")
        if not tail.isdigit() or not 1 <= int(tail) <= 65535:
            raise ValueError("bad port in %r" % spec)
        port = int(tail)
    if not host:
        raise ValueError("no host in %r" % spec)
    return host, port


def _hostport(target):
    return "%s:%d" % target


class JetsonLink:
    """Every call to the boat's ground_station, and which Jetson that is.

    `spec` is "auto" (probe `candidates`, stick with the one that answers; once
    it fails, re-probe all of them at most every REPROBE_S) or an explicit
    host / host:port, which is used as given and never probed away from.

    Health is the outcome of the last REAL call, not a separate poll, so the strip
    reports exactly what the page's own requests are experiencing. A success
    older than the staleness limit is "unknown" rather than "ok".
    """

    def __init__(self, spec="auto", candidates=JETSON_CANDIDATES, clock=time.monotonic):
        self.auto = spec.strip().lower() == "auto"
        self._candidates = tuple(candidates) if self.auto else (parse_target(spec),)
        self._chosen = None if self.auto else self._candidates[0]
        self._clock = clock
        self._lock = threading.Lock()
        self._outcome = None                 # (when, ok, error text)
        self._last_probe = None
        self._probing = False
        self.probe_thread = None             # the last background probe; tests join it

    # ----------------------------------------------------------- selection

    @property
    def target(self):
        with self._lock:
            return self._chosen

    def candidates_text(self):
        return ", ".join(_hostport(c) for c in self._candidates)

    def probe(self):
        """GET /state on each candidate in order; adopt the first that answers.
        Returns the chosen (host, port), or None if none ever has. A Jetson that
        stops answering stays 'chosen' (its address is still the best guess for
        the viewers); only a better answer moves it."""
        found = next((c for c in self._candidates if self._answers(*c)), None)
        with self._lock:
            self._last_probe = self._clock()
            if found:
                self._chosen = found
        self._note(found is not None, None if found else "no Jetson answered (tried %s)"
                   % self.candidates_text())
        return self.target

    @staticmethod
    def _answers(host, port):
        conn = http.client.HTTPConnection(host, port, timeout=PROBE_TIMEOUT_S)
        try:
            conn.request("GET", "/state")
            resp = conn.getresponse()
            resp.read()
            return resp.status == 200
        except HTTP_ERRORS:
            return False
        finally:
            conn.close()

    def _kick_probe(self):
        """Re-probe in the background, if auto, none is running and one is due.
        In the background so the request that just failed fails fast and the
        page's banner is not held back by a second round of timeouts."""
        if not self.auto:
            return
        with self._lock:
            due = self._last_probe is None or self._clock() - self._last_probe >= REPROBE_S
            if self._probing or not due:
                return
            self._probing = True
        self.probe_thread = threading.Thread(target=self._probe_then_release, daemon=True)
        self.probe_thread.start()

    def _probe_then_release(self):
        try:
            self.probe()
        finally:
            with self._lock:
                self._probing = False

    # -------------------------------------------------------------- health

    def _note(self, ok, error=None):
        with self._lock:
            self._outcome = (self._clock(), ok, error)

    def _error(self, message, sent):
        """Record a failed call and build the JetsonError to raise for it. A
        failure may also mean the Jetson moved to the other address."""
        self._note(False, message)
        self._kick_probe()
        return JetsonError(message, sent)

    def status(self, stale_s):
        """The /mav keys for the Jetson: host (no port: the page adds the viewer
        ports itself), ok (True / False / None = no recent call), age, error."""
        with self._lock:
            target, outcome = self._chosen, self._outcome
        ok = age = error = None
        if outcome is not None:
            when, ok, error = outcome
            age = round(self._clock() - when, 2)
            if ok and age > stale_s:
                ok = None
        return {"jetson_host": target and target[0], "jetson_ok": ok,
                "jetson_age": age, "jetson_error": error}

    # ------------------------------------------------------------ requests

    def _open(self, method, path, body, connect_timeout, read_timeout):
        """Send one request; return (connection, response head). JetsonError.sent
        says whether the request could have reached the Jetson."""
        target = self.target
        if target is None:
            raise self._error("no Jetson answered (tried %s)" % self.candidates_text(), False)
        who = _hostport(target)
        conn = http.client.HTTPConnection(*target, timeout=connect_timeout)
        try:
            conn.connect()
        except OSError as e:
            conn.close()
            raise self._error("Jetson %s unreachable: %s" % (who, e), False) from e
        try:
            conn.sock.settimeout(read_timeout)
            headers = {"Connection": "close", "Accept": "application/json"}
            if body is not None:
                headers["Content-Type"] = "application/json"
            conn.request(method, path, body=body, headers=headers)
            return conn, conn.getresponse()
        except HTTP_ERRORS as e:
            conn.close()
            raise self._error("Jetson %s gave no reply to %s: %s" % (who, path, e), True) from e

    def _read_json(self, conn, resp, path):
        """The body of `resp` as a dict, or JetsonError. Closes `conn`."""
        who = _hostport(self.target)
        try:
            data = resp.read()
        except HTTP_ERRORS as e:
            raise self._error("Jetson %s dropped the reply to %s: %s" % (who, path, e), True) from e
        finally:
            conn.close()
        if resp.status != 200:
            problem = "answered HTTP %d to %s" % (resp.status, path)
        else:
            try:
                parsed = json.loads(data)
            except ValueError:
                problem = "answered %s with something that is not JSON" % path
            else:
                if isinstance(parsed, dict):
                    self._note(True)
                    return parsed
                problem = "answered %s with JSON that is not an object" % path
        raise self._error("Jetson %s %s" % (who, problem), True)

    def state(self, layers):
        """GET /state with the same optional layers, unchanged, or JetsonError."""
        path = "/state"
        if layers:
            path += "?" + urllib.parse.urlencode({"layers": ",".join(sorted(layers))}, safe=",")
        conn, resp = self._open("GET", path, None, STATE_TIMEOUT_S, STATE_TIMEOUT_S)
        return self._read_json(conn, resp, path)

    def action(self, path, payload):
        """Forward one POST. Never raises: the page needs a dict to show, and it
        must say whether the action might have happened."""
        if not path.startswith("/"):
            return {"ok": False, "message": "refused: %r is not an action path — nothing was sent" % path}
        try:
            conn, resp = self._open("POST", path, json.dumps(payload).encode("utf-8"),
                                    CONNECT_TIMEOUT_S, ACTION_TIMEOUT_S)
            return self._read_json(conn, resp, path)
        except JetsonError as e:
            tail = ("the request may have reached the Jetson; check what it did before repeating"
                    if e.sent else "nothing was sent")
            return {"ok": False, "message": "%s — %s" % (e, tail)}

    def download(self, name, fileobj):
        """Stream /record/download to `fileobj`. False if the Jetson failed
        (a write error to the browser is not the Jetson's and propagates)."""
        path = "/record/download?" + urllib.parse.urlencode({"name": name})
        try:
            conn, resp = self._open("GET", path, None, CONNECT_TIMEOUT_S, ACTION_TIMEOUT_S)
        except JetsonError:
            return False
        who = _hostport(self.target)
        try:
            if resp.status != 200:
                self._error("Jetson %s answered HTTP %d to %s" % (who, resp.status, path), True)
                return False
            while True:
                try:
                    chunk = resp.read(CHUNK)
                except HTTP_ERRORS as e:
                    self._error("Jetson %s dropped the download: %s" % (who, e), True)
                    return False
                if not chunk:
                    self._note(True)
                    return True
                fileobj.write(chunk)
        finally:
            conn.close()


# ===========================================================================
# The server
# ===========================================================================

def allowed_hosts_for(bind, port):
    """The Host headers this server answers to, or None when bound beyond
    loopback (the operator chose that; the Origin check still applies).

    A page on another site can make the browser POST to localhost -- and these
    POSTs stop nodes and power the Jetson off. A DNS-rebinding page arrives with
    a foreign Host header, a cross-site form or fetch with a foreign Origin."""
    if bind not in LOOPBACK_BINDS:
        return None
    names = {"localhost", "127.0.0.1", "[::1]"}
    return {n + ":%d" % port for n in names} | (names if port == 80 else set())


def request_allowed(host, origin, allowed_hosts):
    """True when the request names this server and, if it carries an Origin, comes
    from a page this server served. Requests without an Origin (curl, the browser's
    address bar) pass the Origin test; they still need the right Host."""
    host = (host or "").strip().lower()
    if allowed_hosts is not None and host not in allowed_hosts:
        return False
    return origin is None or origin.strip().lower() == "http://" + host


class LaptopServer(GcsServer):
    """GcsServer plus GET /mav and the same-origin guard. GcsServer.start() builds
    its HTTP server from self.handler(), so overriding that is all it takes."""

    def __init__(self, page_bytes, link, mav_fn, allowed_hosts=None):
        super().__init__(page_bytes, link.state, link.action, link.download)
        self.mav_fn = mav_fn
        self.allowed_hosts = allowed_hosts

    def handler(self):
        outer, base = self, super().handler()

        class Handler(base):
            def _admitted(self):
                if request_allowed(self.headers.get("Host"), self.headers.get("Origin"),
                                   outer.allowed_hosts):
                    return True
                # Read the body we are refusing: closing on unread data can reset
                # the connection, and the browser would report a network error
                # instead of the 403.
                with contextlib.suppress(ValueError, OSError):
                    self.rfile.read(min(int(self.headers.get("Content-Length") or 0), MAX_BODY))
                self.send_error(403, "this request did not come from the page served here")
                return False

            def do_GET(self):
                if not self._admitted():
                    return
                if self.path == "/mav" or self.path.startswith("/mav?"):
                    try:
                        body = json.dumps(outer.mav_fn()).encode("utf-8")
                    except Exception as e:                           # noqa: BLE001
                        body = json.dumps({"error": "mav status failed: %s" % e}).encode("utf-8")
                    return self._send(body, "application/json", nocache=True)
                return super().do_GET()

            def do_POST(self):
                if self._admitted():
                    super().do_POST()

        return Handler


# ===========================================================================
# main
# ===========================================================================

def port_in_use(host, port):
    """Is something already listening? Needed because Windows lets a second
    server bind a port the first still holds (SO_REUSEADDR), without an error."""
    s = socket.socket()
    s.settimeout(0.5)
    try:
        return s.connect_ex(("127.0.0.1" if host in ("0.0.0.0", "") else host, port)) == 0
    finally:
        s.close()


def announce_changes(state, link, stale_s, stop, say=print, every=1.0):
    """Say, once per change, when MAVLink or the Jetson goes quiet or returns.
    A window that stops updating must say why; silence reads as 'broken'.
    An unknown Jetson (no recent call) says nothing, and neither does a first
    observation that is already fine."""
    last = {}
    while not stop.wait(every):
        checks = (
            ("mav", state.snapshot(time.monotonic())["link"]["ok"], "MAVLink is arriving.",
             "*** no MAVLink from system %d on %s -- is the OCS fleet router (fleet_link) running, "
             "and QGroundControl closed if you read 14550 directly?" % (state.sysid, state.source)),
            ("jetson", link.status(stale_s)["jetson_ok"], "Jetson API answers.",
             "*** the Jetson API is not answering (%s) -- the page shows its banner; the MAVLink "
             "strip keeps working." % link.candidates_text()))
        for key, ok, up, down in checks:
            if ok is None or last.get(key) == ok:
                continue
            if key in last or not ok:
                say("  " + (up if ok else down), flush=True)
            last[key] = ok


def mav_status(link, state, stale_s, now=None):
    """The GET /mav body: the Jetson's health, then everything the autopilot said."""
    out = link.status(stale_s)
    out.update(state.snapshot(time.monotonic() if now is None else now))
    return out


def int_in(lo, hi):
    """argparse type: an integer lo..hi, so a bad value is a usage error and not a traceback
    from deep inside the socket layer."""
    def parse(text):
        try:
            value = int(text)
        except ValueError:
            value = None
        if value is None or not lo <= value <= hi:
            raise argparse.ArgumentTypeError("%r is not an integer in %d..%d" % (text, lo, hi))
        return value
    return parse


def jetson_spec(text):
    """argparse type: 'auto', or a host / host:port that parse_target accepts."""
    if text.strip().lower() != "auto":
        try:
            parse_target(text)
        except ValueError as e:
            raise argparse.ArgumentTypeError(str(e)) from e
    return text


def build_parser():
    ap = argparse.ArgumentParser(
        description="Crusader's ground station page, served from this laptop. "
                    "Listen-only MAVLink for the strip; everything else is the Jetson's.")
    ap.add_argument("--port", type=int_in(1, 65535), default=8150, help="the laptop page (default 8150)")
    ap.add_argument("--bind", default="127.0.0.1",
                    help="address to serve on (default 127.0.0.1; the page can stop nodes "
                         "and power the Jetson off, so widen it deliberately)")
    ap.add_argument("--mav", dest="mav_source", default=DEFAULT_MAV,
                    help="where the boat's MAVLink comes from: any pymavlink connection string "
                         "(default %s, from the OCS fleet router; udpin:0.0.0.0:14550 when no "
                         "router runs). Listen-only, always." % DEFAULT_MAV)
    ap.add_argument("--sysid", type=int_in(1, 255), default=2, help="the autopilot's system id (default 2)")
    ap.add_argument("--jetson", type=jetson_spec, default="auto",
                    help="'auto' (default): probe %s and stick with the one that answers; or an "
                         "explicit host or host:port, which disables probing"
                         % " then ".join(_hostport(c) for c in JETSON_CANDIDATES))
    ap.add_argument("--poll-ms", type=int_in(1, 60000), default=None,
                    help="how often the page asks /state (default: ground_station.poll_period_s "
                         "from crusader_params.yaml, else %d)" % DEFAULT_POLL_MS)
    ap.add_argument("--stale-s", type=float, default=3.0,
                    help="a MAVLink value older than this is blank (default 3.0)")
    return ap


def startup_block(args, link, params, poll_ms):
    target = link.target
    if link.auto and target is None:
        jetson = "NOT reachable yet (tried %s); re-probing every %d s" % (link.candidates_text(), REPROBE_S)
    elif link.auto:
        jetson = "%s (auto: probed %s)" % (_hostport(target), link.candidates_text())
    else:
        jetson = "%s (explicit; not probed away from)" % _hostport(target)
    host = "localhost" if args.bind in LOOPBACK_BINDS else args.bind
    lines = ["Boat ground station (laptop)",
             "  page      http://%s:%d/" % (host, args.port),
             "  Jetson    " + jetson,
             "  MAVLink   %s, system %d, LISTEN-ONLY (never transmits)" % (args.mav_source, args.sysid),
             "  e-stop    RC ch%d, engaged or RC lost below %d us (crusader_params.yaml)"
             % (params.estop_channel, params.estop_threshold),
             "  page poll %d ms; MAVLink values blank after %.1f s" % (poll_ms, args.stale_s)]
    if args.bind not in LOOPBACK_BINDS:
        lines.append("  *** bound to %s: anyone who can reach this port can press the boat's "
                     "buttons, including power-off" % args.bind)
    return "\n".join(lines)


def fail(message):
    """Say on stderr why the program cannot start; the exit status for that."""
    print("gcs_laptop: " + message, file=sys.stderr)
    return 2


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        params = load_params()
        poll_ms = args.poll_ms if args.poll_ms is not None else params.poll_ms
        page = build_page(poll_ms)
    except (ConfigError, PagePatchError) as e:
        return fail(str(e))
    if port_in_use(args.bind, args.port):
        return fail("port %d is already in use -- is the Boat GUI already running? "
                    "(--port picks another)" % args.port)
    state = MavState(args.sysid, params.estop_channel, params.estop_threshold, args.stale_s, args.mav_source)
    try:
        source = ListenOnly(args.mav_source)
    except Exception as e:                                           # noqa: BLE001
        return fail("cannot open MAVLink source %s: %s\n  (another program holding that UDP port? "
                    "14550 is QGroundControl's; the router forwards on 14554)" % (args.mav_source, e))
    link = JetsonLink(args.jetson)
    link.probe()
    server = LaptopServer(page, link, lambda: mav_status(link, state, args.stale_s),
                          allowed_hosts_for(args.bind, args.port))
    stop = threading.Event()
    try:
        server.start(args.port, args.bind)
    except OSError as e:
        source.close()
        return fail("cannot serve on %s:%d: %s" % (args.bind, args.port, e))
    try:
        threading.Thread(target=read_loop, args=(source, state, stop), daemon=True).start()
        print(startup_block(args, link, params, poll_ms))
        print("  Ctrl+C to stop.", flush=True)
        announce_changes(state, link, args.stale_s, stop)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.stop()
        source.close()
    print("Boat ground station stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
