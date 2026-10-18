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
    MAVLink on this laptop -> a strip across the top of the page: link (with its
                            lag), the RC e-stop, five drone-style tiles (Mode,
                            armed highlighted; GPS fix quality; Heading = GPS
                            yaw; EKF health; Pre-arm), battery, autopilot
                            warnings, and a GPS panel under the Telemetry tab.
                            The Jetson's page reads NO MAVLink at all. The team
                            logos and the word "Crusader" sit first in the page's
                            top bar (team_logos.py, from the drone GUI).
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
unavailable", 0 is "not provided". A verdict (/mav gps.quality, gps.heading_quality,
ekf.quality, prearm.quality) is "unknown" whenever its section is stale, never the last
grade; satellites 255, eph 0 / 65535, a zero h_acc / v_acc / hdg_acc / alt_ellipsoid, a
0,0 position and a non-finite EKF variance are blank. The link's lag (/mav link.lag_ms) is
blank until it has been measured and whenever it has not been refreshed within --stale-s.
The e-stop keeps the vehicle ARMED, so it is
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
from html import escape
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
from crusader_groundstation.gcs_server import GcsServer, body_limit  # noqa: E402
from crusader_groundstation.mission_api import LIB_PATHS, MissionLib   # noqa: E402

PARAMS_YAML = os.path.join(REPO, "crusader_bringup", "config", "crusader_params.yaml")
# The Mission tab's library endpoints are answered HERE, from this checkout, so a mission saved
# from the laptop lands in the repo (and in git); start / stop / status still go to the Jetson.
MISSION_BT_DIR = os.path.join(REPO, "crusader_bt", "behavior_trees")
MISSION_DIR = os.path.join(REPO, "crusader_bt", "missions")

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

# ---- GPS verdicts ---------------------------------------------------------
# Mirrors the drone GUI's GPS tile (Drone/rx26_uav/uav_groundstation/uav_groundstation/
# preflight_core.py), so a "Degraded" means the same thing on both vehicles. Same values, same
# strict `>` / `<` comparisons; the one thing left out is its Septentrio-only `gnss` source,
# which the boat's UM982 does not have. Line numbers are preflight_core.py's.
GPS_MIN_FIX = 3                             # :113  `fix < 3` is "no 3D fix", always bad
GPS_MIN_SATS = 10                           # :33   no accuracy figure: fewer is a warning
GPS_MAX_HDOP = 1.2                          # :34   no accuracy figure: higher is a warning
GPS_GOOD_ACC_M = 0.5                        # :41   accuracy reported: worse than this is a warning
GPS_POOR_ACC_M = 1.5                        # :42   accuracy reported: worse than this is bad
GPS_WARN_SATS, GPS_POOR_SATS = 9, 6         # :49   accuracy reported: fewer is warn / bad
RTK_FIXED = mavutil.mavlink.GPS_FIX_TYPE_RTK_FIXED     # :50 (6). Exempt from GPS_WARN_SATS only
GPS_WARN_HDOP, GPS_POOR_HDOP = 2.0, 3.0     # :51   accuracy reported: higher is warn / bad
# The boat's heading is dual-antenna GPS yaw (COMPASS_USE=0), so the drone has no equivalent: these
# two are a FIRST GUESS for a ~1 m baseline, not measured. Nobody has seen the UM982's yaw accuracy
# on the water yet (the boat has never been driven); revisit once /mav shows real hdg_acc_deg.
HEADING_WARN_ACC_DEG = 1.0
HEADING_BAD_ACC_DEG = 3.0

# ---- EKF verdict ----------------------------------------------------------
# Mission Planner's EKF light (ExtLibs/Controls/HUD.cs on master, read 2026-10-10: `if (ekfstatus > 0.8)` red,
# `else if (ekfstatus > 0.5)` yellow, else green), where ekfstatus is the largest of the five EKF_STATUS_REPORT
# variances and a missing EKF_ATTITUDE or an EKF_UNINITIALIZED forces it to 1.0 (ExtLibs/ArduPilot/CurrentState.cs).
# ArduPilot itself defines only the 0.8: Rover's FS_EKF_THRESH defaults to 0.8 ("0.6 Strict, 0.8 Default, 1.0
# Relaxed", Rover/Parameters.cpp at Rover-4.6.3; the failsafe fires when two of compass, position and velocity
# exceed it for 1 s); the 0.5 is Mission Planner's own. One edge differs: MP calls exactly 0.5 green, this calls
# it degraded (`>=`). QGroundControl turned up no variance thresholds (its estimator fact group reads
# ESTIMATOR_STATUS, a different message; not read further).
EKF_WARN_VARIANCE = 0.5
EKF_BAD_VARIANCE = 0.8


class KeyFlag(NamedTuple):
    """One EKF_STATUS_FLAGS bit the EKF tile judges. /mav ekf carries it as `name`: True when the bit is SET."""
    name: str
    bit: int                # the dialect's, not a table kept here
    label: str              # how the tooltip names it
    good_when_set: bool
    severity: str           # what it costs when it is the wrong way round: "bad" or "warn"
    problem: str            # the reason it gives then


#: The five flags the tile reports. EKF_GPS_GLITCHING is bit 15: Rover-4.6.3 sends `1<<15`.
EKF_KEY_FLAGS = (
    KeyFlag("attitude", mavutil.mavlink.EKF_ATTITUDE, "attitude", True, "bad", "no attitude estimate"),
    KeyFlag("horiz_pos_abs", mavutil.mavlink.EKF_POS_HORIZ_ABS, "horizontal position (abs)", True, "bad",
            "no absolute horizontal position"),
    KeyFlag("const_pos_mode", mavutil.mavlink.EKF_CONST_POS_MODE, "constant-position mode", False, "bad",
            "constant-position mode: the EKF has no position"),
    KeyFlag("gps_glitch", mavutil.mavlink.EKF_GPS_GLITCHING, "GPS glitching", False, "warn",
            "the EKF believes the GPS is glitching"),
    KeyFlag("uninitialized", mavutil.mavlink.EKF_UNINITIALIZED, "never initialised", False, "bad",
            "the EKF has never been healthy"))
#: (field in /mav ekf, how a reason names it). compass_variance is the largest of three axes of
#: sqrt(max(magTestRatio, yawTestRatio)) (AP_NavEKF3_Outputs.cpp, Rover-4.6.3). With the compass disabled
#: (COMPASS_USE=0, EK3_SRC1_YAW=2) that is taken to be the GPS-yaw innovation: not yet seen on the boat.
EKF_VARIANCES = (("velocity_variance", "velocity"), ("pos_horiz_variance", "horizontal position"),
                 ("pos_vert_variance", "vertical position"), ("compass_variance", "compass (GPS yaw)"),
                 ("terrain_alt_variance", "terrain"))

# ---- link lag -------------------------------------------------------------
LAG_WARN_MS = 1500          # the drone page's link tile turns warn above this (gcs_page.py:1447)
LAG_MIN_SAMPLES = 10        # the first sample IS the floor, so its lag is 0 by construction: not a measurement
LAG_BACKWARDS_MS = 2000     # a boot clock this far behind its high-water mark is a reboot, not a reordering

# ---- pre-arm --------------------------------------------------------------
PREARM_BIT = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
#: state -> (verdict, reasons). Rover-4.6.3 (libraries/GCS_MAVLink/GCS.cpp) sets `present` always, `enabled`
#: when ARMING_CHECK is non-zero, and `health` when enabled AND (the vehicle is armed OR the pre-arm checks
#: pass): so while ARMED the bit says "pass" without having been re-checked, and with checks switched off
#: `health` is clear, which is "disabled", not a failure.
PREARM_VERDICTS = {
    "pass": ("ok", []),
    "fail": ("bad", ["the autopilot's pre-arm checks are failing: see its PreArm: messages"]),
    "disabled": ("warn", ["the autopilot's pre-arm checks are switched off (ARMING_CHECK is 0)"]),
    "not_reported": ("unknown", ["the autopilot does not report pre-arm health"]),
}
GPS_FIX_2D = mavutil.mavlink.GPS_FIX_TYPE_2D_FIX    # below this the message's lat/lon may be the last position


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

#: Small on purpose -- a line or two on a laptop -- and made only of the page's
#: own CSS variables, so day mode (html[data-theme="day"]) recolours it with
#: everything else. A hardcoded colour here is a strip that stays dark in
#: sunlight. The ONE literal is the logo badge's white: the logos are dark line
#: art that needs a white ground in both themes, and gcs_page.py (which must not
#: be edited) has no --logo-bg to borrow.
STRIP_CSS = r"""
 /* team logos: first thing in the page's top bar (the drone page's .logos, same look). */
 .logos{flex:none;display:flex;align-items:center;gap:6px;margin-right:8px;background:#fff;
   border-radius:6px;padding:2px 6px;border:1px solid var(--line)}
 .logos img{height:26px;width:auto;display:block}
 /* the vehicle's name beside the logos: the drone page's .brand (17 px, bold), in the page's --strong. */
 #bar .brand{flex:none;margin-right:12px;font-size:17px;font-weight:700;letter-spacing:.2px;
   color:var(--strong);white-space:nowrap}
 /* mavstrip:the laptop's own MAVLink readout (gcs_laptop.py). */
 #mavstrip{flex:none;padding:3px 10px;background:var(--panel);
   border-bottom:1px solid var(--line);font-size:12px;line-height:1.5}
 #mavstrip .mv-row{display:flex;flex-wrap:wrap;align-items:center;gap:0 10px}
 #mavstrip .mv-t{color:var(--dim);font-weight:600}
 #mavstrip .mv-k{color:var(--dim);margin-right:5px}
 #mavstrip .mv-strong{color:var(--strong)}
 #mavstrip .mv-ok{color:var(--ok)}
 #mavstrip .mv-warn{color:var(--warn)}
 #mavstrip .mv-bad{color:var(--bad);font-weight:600}
 #mavstrip .mv-dim{color:var(--dim)}
 /* The drone page's GPS tile, shrunk to sit in a row: a 4 px coloured edge is the verdict, the
    dot and the word repeat it (colour is never the only signal), warn/bad also tint the tile.
    The tint is color-mix over the page's own --card; a browser without color-mix keeps the
    plain --card line above it. */
 #mavstrip .mv-tile{display:inline-flex;flex-direction:column;padding:0 9px 1px 8px;
   background:var(--card);border:1px solid var(--line);border-left:4px solid var(--line);
   border-radius:6px;line-height:1.3}
 #mavstrip .mv-tile.ok{border-left-color:var(--ok)}
 #mavstrip .mv-tile.warn{border-left-color:var(--warn);
   background:color-mix(in srgb,var(--warn) 16%,var(--card))}
 #mavstrip .mv-tile.bad{border-left-color:var(--bad);
   background:color-mix(in srgb,var(--bad) 16%,var(--card))}
 #mavstrip .mv-tl{display:flex;align-items:center;gap:6px;white-space:nowrap}
 #mavstrip .mv-tk{font-size:10.5px;font-weight:700;letter-spacing:.8px;text-transform:uppercase;
   color:var(--dim)}
 #mavstrip .mv-big{display:inline-flex;align-items:center;gap:5px;font-size:14px;font-weight:700;
   color:var(--dim)}
 #mavstrip .mv-tile.ok .mv-big{color:var(--ok)}
 #mavstrip .mv-tile.warn .mv-big{color:var(--warn)}
 #mavstrip .mv-tile.bad .mv-big{color:var(--bad)}
 #mavstrip .mv-dot{width:11px;height:11px;border-radius:50%;flex:none;display:inline-block;
   background:var(--dot)}
 #mavstrip .mv-dot.ok{background:var(--ok)}
 #mavstrip .mv-dot.warn{background:var(--warn)}
 #mavstrip .mv-dot.bad{background:var(--bad)}
 #mavstrip .mv-sub{font-size:11px;color:var(--dim);white-space:nowrap}
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
/* The last /mav body, for the Telemetry panel. null until a poll has answered, and again after any
   poll that fails: a panel never shows the last good body as if it were current. */
var lastMav = null;

function node(tag, cls, text){
  var n = document.createElement(tag);
  if(cls) n.className = cls;
  if(text !== undefined && text !== null) n.textContent = text;
  return n;
}
/* null for a blank, so a caller can tell "not reported" from a number; num() is the same with a dash. */
function val(v, digits, unit){
  return (v === null || v === undefined) ? null : Number(v).toFixed(digits) + (unit || '');
}
function num(v, digits, unit){
  var s = val(v, digits, unit);
  return s === null ? DASH : s;
}
function item(row, label, value, cls, tip){
  var s = node('span', 'mv-i');
  if(tip) s.title = tip;
  if(label) s.appendChild(node('span', 'mv-k', label));
  s.appendChild(node('span', cls || '', value));
  row.appendChild(s);
}
/* Never heard at all (as against heard, but not lately): the section's age is null. */
function unheard(sec){ return sec.age === null || sec.age === undefined; }
function ageTip(sec){
  return unheard(sec) ? 'never received' : 'last received ' + sec.age.toFixed(1) + ' s ago';
}

var LAG_TIP = 'lag: how far behind the autopilot this data arrives. The autopilot stamps each message with its ' +
  'own boot clock; this is (this laptop clock minus that stamp) above the smallest value seen since this page ' +
  'started or the autopilot last rebooted, so it is relative to the best the link has done, and it includes any ' +
  'slow clock drift. Blank until measured, and whenever it has not been refreshed lately.';
function showLink(row, L){
  if(L.ok){
    var slow = L.lag_ms != null && L.lag_warn_ms != null && L.lag_ms > L.lag_warn_ms;
    item(row, 'link', num(L.age, 1, ' s') + ' · ' + num(L.fps, 0, ' fps') + ' · lag ' + num(L.lag_ms, 0, ' ms'),
         slow ? 'mv-warn' : 'mv-ok',
         'frames from system ' + L.sysid + ' on ' + L.source + '. ' + LAG_TIP +
         (slow ? ' Above ' + L.lag_warn_ms + ' ms.' : ''));
    return;
  }
  var why = unheard(L) ? 'no frames yet' : 'silent ' + num(L.age, 0, ' s');
  if(L.other_sysids && L.other_sysids.length) why += ' (heard system ' + L.other_sysids.join(', ') + ' instead)';
  item(row, 'link', why, 'mv-bad', L.error || ('nothing from system ' + L.sysid + ' on ' + L.source));
}
/* One value of a section: its text and colour only while the section is fresh, a dim
   dash otherwise -- never the last value. `value` is computed either way, so it must
   tolerate a section with nothing in it (hence == null, which is also true of undefined). */
function fresh(row, label, sec, value, cls, tip){
  item(row, label, sec.ok ? value : DASH, sec.ok ? cls : 'mv-dim', tip || ageTip(sec));
}
function showEstop(row, R){
  var tip = 'RC channel ' + R.estop_channel + ' (SB switch); below ' + R.estop_threshold +
            ' us is e-stop engaged OR RC lost. ' + ageTip(R);
  var pwm = (R.estop_pwm == null) ? '' : ' ' + R.estop_pwm + ' µs';
  if(R.ok && R.estop_engaged === true) item(row, 'e-stop', 'ENGAGED / RC LOST' + pwm, 'mv-bad', tip);
  else if(R.ok && R.estop_engaged === false) item(row, 'e-stop', 'RUN' + pwm, 'mv-ok', tip);
  else item(row, 'e-stop', DASH, 'mv-dim', tip);
}
var QUALITY = {ok: 'Good', warn: 'Degraded', bad: 'Poor', unknown: 'No data', neutral: ''};
/* One drone-style tile: label, a dot and the verdict word, a sub line, a tooltip. `verdict` is the
   server's (gps.quality, ekf.quality ...) or 'neutral' for a state that is neither good nor bad; anything
   else reads as no data. `word` replaces the verdict word where the headline is a state of its own (the
   mode, PASS / FAIL). Text nodes only. */
function tile(row, label, verdict, sub, tip, word){
  var q = QUALITY.hasOwnProperty(verdict) ? verdict : 'unknown';
  var t = node('span', 'mv-tile ' + q), top = node('span', 'mv-tl'), big = node('span', 'mv-big');
  t.title = tip;
  big.appendChild(node('span', 'mv-dot ' + q));
  big.appendChild(node('span', '', word == null ? QUALITY[q] : word));
  top.appendChild(node('span', 'mv-tk', label));
  top.appendChild(big);
  t.appendChild(top);
  t.appendChild(node('span', 'mv-sub', sub));
  row.appendChild(t);
}
function reasonTip(list){
  return (list && list.length) ? '\n' + list.join('; ') : '';
}
/* A section that is not fresh has no values and no verdict: say so and how old, never the last one. */
function noData(sec, what){
  return unheard(sec) ? 'no ' + what + ' data' : 'stale: last ' + sec.age.toFixed(0) + ' s ago';
}
/* "H acc ±0.02 m", or "H acc not reported": a blank is said to be blank, never filled in. */
function reading(label, v, digits, unit, sign){
  return label + (v == null ? ' not reported' : ' ' + (sign || '') + v.toFixed(digits) + (unit || ''));
}
/* A tile whose section may be stale. Not fresh: "No data", the section's age, and the tooltip's `intro`; never
   the last values. Fresh: build() returns {sub, tip, word}, with tip and word optional (the tip then defaults
   to `intro` and the age). `verdict` and `reasons` are the server's; the reasons always end the tooltip. */
function vtile(row, label, sec, what, intro, verdict, reasons, build){
  var b = sec.ok ? build() : {sub: noData(sec, what)};
  tile(row, label, sec.ok ? verdict : 'unknown', b.sub, (b.tip || intro + ' ' + ageTip(sec)) + reasonTip(reasons), b.word);
}
function showGps(row, G){
  vtile(row, 'GPS', G, 'GPS', 'GPS fix quality.', G.quality, G.quality_reasons, function(){
    var name = G.fix_name || 'fix ' + G.fix_type, parts = [name];
    if(G.sats != null) parts.push(G.sats + ' sats');
    if(G.h_acc_m != null) parts.push('±' + G.h_acc_m.toFixed(2) + ' m');
    return {sub: parts.join(' · '),
            tip: [name, G.sats == null ? 'satellites not reported' : G.sats + ' satellites',
                  reading('HDOP', G.hdop, 2), reading('H acc', G.h_acc_m, 2, ' m', '±'),
                  reading('V acc', G.v_acc_m, 2, ' m', '±')].join(' · ')};
  });
}
/* The two yaw states that are not a number, and how they read; null for any other (out of range). */
var YAW_WORD = {unavailable: 'UNAVAILABLE', not_provided: 'NOT PROVIDED'};
function yawWord(state){ return YAW_WORD.hasOwnProperty(state) ? YAW_WORD[state] : null; }
/* The heading is dual-antenna GPS yaw: the compass is disabled on this boat. */
function showHeading(row, G){
  vtile(row, 'Heading', G, 'GPS yaw',
        'The heading source: GPS yaw from the dual antennas; the compass is disabled on this boat.',
        G.heading_quality, G.heading_reasons, function(){
    var word = yawWord(G.yaw_state);
    if(G.yaw_state === 'ok')
      return {sub: 'GPS yaw ' + G.yaw_deg.toFixed(1) + '°' + (G.hdg_acc_deg == null ? '' : ' ±' + G.hdg_acc_deg.toFixed(1) + '°')};
    return {sub: word ? word + ' — no compass' : 'yaw out of range'};
  });
}
function showMode(row, H){
  vtile(row, 'Mode', H, 'heartbeat', 'Autopilot flight mode and armed state (HEARTBEAT).',
        H.armed ? 'warn' : 'neutral', [], function(){
    return {sub: H.armed ? 'ARMED' : 'disarmed', word: H.mode == null ? DASH : H.mode};
  });
}
/* EKF health. The server decides (ekf.quality, from the variances and the key flags) and writes the words
   (ekf.summary, ekf.detail); this only lays them out. */
function showEkf(row, E){
  vtile(row, 'EKF', E, 'EKF', 'EKF health (EKF_STATUS_REPORT).', E.quality, E.quality_reasons, function(){
    return {sub: E.summary,
            tip: ['EKF health. Good below ' + E.warn_variance + ', Degraded from there to ' + E.bad_variance +
                  ', Poor above it, ' +
                  'on the largest variance (Mission Planner\'s EKF light); a missing attitude, absolute position ' +
                  'or an uninitialised EKF is Poor. ' + ageTip(E)].concat(E.detail, [
                  'On this boat the compass variance is the GPS-yaw innovation: the compass is disabled (COMPASS_USE=0).'
                  ]).join('\n')};
  });
}
var PREARM = {pass: ['PASS', 'checks pass'], fail: ['FAIL', 'see the PreArm: warnings'],
              disabled: ['CHECKS OFF', 'ARMING_CHECK is 0'], not_reported: ['not reported', 'no pre-arm bit']};
/* Pre-arm: the autopilot's own verdict (SYS_STATUS health bit). While armed it reports "pass" without
   checking again, so an armed PASS says so. */
function showPrearm(row, P, H){
  vtile(row, 'Pre-arm', P, 'pre-arm', 'Pre-arm checks (SYS_STATUS, MAV_SYS_STATUS_PREARM_CHECK).',
        P.quality, P.quality_reasons, function(){
    var armed = P.state === 'pass' && H.ok && H.armed, words = PREARM[P.state] || PREARM.not_reported;
    return {sub: armed ? 'armed: not re-checked' : words[1], word: words[0],
            tip: 'Pre-arm checks: the autopilot\'s own verdict. While armed it reports PASS without checking again. ' + ageTip(P)};
  });
}
function showPos(row, P){
  fresh(row, 'pos', P, P.lat == null ? 'no position' : P.lat.toFixed(7) + ', ' + P.lon.toFixed(7), '');
  fresh(row, '', P, num(P.groundspeed, 1, ' m/s') + ' · hdg ' + num(P.heading, 0, '°'), '');
}
function showBatt(row, B){
  var none = B.voltage == null || B.voltage <= 0;
  fresh(row, 'batt', B, none ? 'no sensor' : B.voltage.toFixed(2) + ' V (unverified)', none ? 'mv-dim' : '',
        'UNVERIFIED: BATT_VOLT_MULT is disputed (the two candidate multipliers differ by about 45%) and the cell ' +
        'count, 3S or 4S, is unsettled. This is the voltage as the autopilot reports it; 0 V and 65535 mean no ' +
        'sensor. See the crusader-battery skill before trusting a number. ' + ageTip(B));
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
    var hb = j.hb || {}, gps = j.gps || {};
    showLink(top, j.link || {});
    showMode(top, hb);
    showEstop(top, j.rc || {});
    showGps(top, gps);
    showHeading(top, gps);
    showEkf(top, j.ekf || {});
    showPrearm(top, j.prearm || {}, hb);
    showPos(top, j.pos || {});
    showBatt(top, j.batt || {});
    showJetson(top, j);
    showTexts(texts, j.statustext || []);
  }
  strip.textContent = '';
  strip.appendChild(top);
  if(texts.childNodes.length) strip.appendChild(texts);
}

/* ---- the GPS panel on the Telemetry tab. The page's renderTel() rewrites the whole pane on every Jetson
   poll and then calls window.mavTel (the hook gcs_laptop.py patches in), so the panel is rebuilt each time;
   this script also refreshes it itself after each /mav answer, which keeps it alive while the Jetson is
   down and the page is no longer rendering. Same card / grid classes as the pane, so day mode recolours it. */
/* One card: a value only while the section is fresh (else a red dash, never the last value), a plain dash with
   a tooltip where the autopilot did not report it (text == null), red where `bad`. */
function telCard(grid, G, c){
  var label = c[0], text = c[1], card = node('div', 'card'), blank = !G.ok || text == null;
  if(G.ok && text == null) card.title = label + ' is not reported by the autopilot';
  card.appendChild(node('div', 'k', label));
  card.appendChild(node('div', !G.ok || c[2] ? 'v stale' : 'v', blank ? DASH : text));
  grid.appendChild(card);
}
function buildTel(){
  var box = node('div'), grid = node('div', 'grid');
  var G = (lastMav && lastMav.gps) || {ok: false, age: null};
  box.id = 'mavtel';
  box.appendChild(node('h3', '', 'GPS (autopilot, via the laptop MAVLink)'));
  box.appendChild(node('div', 'hint', 'Read by this laptop, not the Jetson. ' +
    (lastMav === null ? 'No answer from the laptop server (gcs_laptop.py).' :
     G.ok ? ageTip(G) + '.' : noData(G, 'GPS') + '.')));
  [['Fix', G.fix_type == null ? null : (G.fix_name || 'fix ' + G.fix_type), G.quality === 'bad'],
   ['Satellites', val(G.sats, 0)],
   ['HDOP', val(G.hdop, 2)],
   ['Horizontal accuracy', val(G.h_acc_m, 2, ' m')],
   ['Vertical accuracy', val(G.v_acc_m, 2, ' m')],
   ['GPS yaw', G.yaw_state === 'ok' ? G.yaw_deg.toFixed(1) + '°' : yawWord(G.yaw_state) || 'out of range',
    G.yaw_state !== 'ok'],
   ['Yaw accuracy', val(G.hdg_acc_deg, 1, '°')],
   ['Latitude', val(G.lat, 7, '°')],
   ['Longitude', val(G.lon, 7, '°')],
   ['Altitude (ellipsoid)', val(G.alt_ellipsoid_m, 2, ' m')]
  ].forEach(function(c){ telCard(grid, G, c); });
  box.appendChild(grid);
  return box;
}
function putTel(){
  var pane = document.getElementById('p-tel'), old = document.getElementById('mavtel');
  if(!pane) return;
  if(old && old.parentNode === pane) pane.replaceChild(buildTel(), old); else pane.appendChild(buildTel());
}
window.mavTel = putTel;
function refreshTel(){ if(typeof tab !== 'undefined' && tab === 'tel') putTel(); }

function poll(){
  if(busy) return;
  busy = true;
  fetch('/mav', {cache: 'no-store'}).then(function(r){
    if(!r.ok) throw new Error('HTTP ' + r.status);
    return r.json();
  }).then(function(j){
    busy = false; first = false;
    if(j.error){ lastMav = null; show(null, j.error); refreshTel(); return; }
    if(typeof j.jetson_host === 'string' && j.jetson_host) window.VIEW_HOST = j.jetson_host;
    lastMav = j;
    show(j);
    refreshTel();
  }).catch(function(){
    busy = false; first = false;
    lastMav = null;
    show(null);
    refreshTel();
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

def logos_html():
    """The team logos as the page's top-bar badge (CSS in STRIP_CSS), or "" if team_logos.py -- the
    drone GUI's generated file, copied next to this script -- will not import: a page without its
    logos still works. Names go through escape(): one of them has an ampersand."""
    try:
        import team_logos
    except ImportError:
        return ""
    return '<div class="logos">%s</div>' % "".join(
        '<img src="%s" alt="%s" title="%s">' % (uri, escape(name), escape(name))
        for name, uri in team_logos.LOGOS)


#: The vehicle's name, beside the logos: the drone page's `<div class="brand">Ekko</div>` (gcs_page.py:360).
BRAND_HTML = '<div class="brand" title="rx26_asv ground station">Crusader</div>'


def bar_head_html():
    """What goes first in the page's top bar: the team logos, then the vehicle's name. The name stays
    when the logos cannot be imported."""
    return logos_html() + BRAND_HTML


BAR_HEAD_HTML = bar_head_html()
BAR_OPEN = '<div id="bar">'


def logos_patch(markup):
    """The PAGE_PATCHES entry that makes `markup` the first thing in the page's top bar. With
    no markup it replaces the anchor with itself, so it still counts it."""
    return ("top-bar head (team logos, vehicle name)", BAR_OPEN,
            BAR_OPEN + ("\n  " + markup if markup else ""), 1)


#: The page's render() draws the Telemetry pane with `if(tab==='tel') renderTel();` -- once, so the count
#: below is 1. The replacement calls the strip's window.mavTel after it, inside a try so a fault in the GPS
#: panel can never stop render() before it reaches the viewers and the banner. THE ORDER IN PAGE_PATCHES
#: MATTERS: this runs before the strip script goes in, and STRIP_JS must not contain the anchor text, or
#: the replacement would be applied to the script too.
TEL_HOOK = ("if(tab==='tel') renderTel();",
            "if(tab==='tel'){ renderTel(); try{ if(window.mavTel) window.mavTel(); }"
            " catch(e){ console.error('mavTel failed', e); } }")


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
    # First in the top bar, so the logos and the name sit left of the tabs. The anchor is the bar's own
    # opening tag, which occurs once (the CSS says `#bar{`, never `<div id="bar">`).
    logos_patch(BAR_HEAD_HTML),
    ("telemetry GPS panel hook",) + TEL_HOOK + (1,),
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


def _known(raw, unknown, scale=None):
    """`raw` (divided by `scale`, if given), or None when it is one of the field's "unknown"
    values -- blanks over guesses. The sentinels are the dialect's: GPS_RAW_INT.satellites_visible
    and .eph say UINT8_MAX / UINT16_MAX when unknown. h_acc, v_acc and hdg_acc (the extension
    fields) document no sentinel, and 0 is what a sender that does not fill them leaves (MAVLink 2
    also drops trailing zero bytes, which decode as 0); a measured 0.000 m is not a thing. eph 0 is
    blanked too, although the dialect names only UINT16_MAX: an HDOP of 0.00 cannot be measured
    (the drone's gcs_radio.py blanks it as well)."""
    if raw in unknown:
        return None
    return raw if scale is None else raw / scale


def _lat_lon(msg):
    """(lat, lon) in degrees from a message carrying degE7 lat / lon, or (None, None) at 0, 0 -- the
    autopilot's "no position"."""
    if msg.lat == 0 and msg.lon == 0:
        return None, None
    return msg.lat / 1e7, msg.lon / 1e7


def _variance(raw):
    """An EKF variance to 3 places, or None when it is NaN, infinite or negative -- not a variance.
    The rounding makes float32 0.8 (which decodes as 0.800000011920929) mean 0.8, so the strict `>`
    in ekf_quality means what it says at the boundary."""
    return round(float(raw), 3) if math.isfinite(raw) and raw >= 0 else None


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


# ---------------------------------------------------------------------------
# GPS verdicts: pure functions of the decoded values, so the tests drive every boundary
# ---------------------------------------------------------------------------

def _gps_reasons(acc, sats, hdop, acc_limit, sats_limit, hdop_limit):
    """What is wrong with the fix at ONE level of the verdict: the accuracy above `acc_limit`,
    the satellites below `sats_limit`, the HDOP above `hdop_limit`, each only when it is known
    (acc / sats / hdop are None when not). Empty means the fix passes that level."""
    return [text for text, hit in (
        ("accuracy %.2f m > %.1f m" % (acc or 0, acc_limit or 0), acc is not None and acc > acc_limit),
        ("only %d satellites (< %d)" % (sats or 0, sats_limit), sats is not None and sats < sats_limit),
        ("HDOP %.1f > %.1f" % (hdop or 0, hdop_limit), hdop is not None and hdop > hdop_limit)) if hit]


def gps_quality(fix_type, sats, hdop, h_acc_m):
    """('ok' | 'warn' | 'bad' | 'unknown', [short reasons]) for the position fix, as the drone's
    preflight_core.gps() judges it (see the GPS_* constants for the source lines). Unknown
    inputs are None: an unknown satellite count or HDOP never vetoes, it is simply not counted.

    Without an accuracy figure only satellites and HDOP can be judged, against the strict
    GPS_MIN_SATS / GPS_MAX_HDOP. With one, that decides first and satellites / HDOP can still
    veto: GPS_POOR_* make it bad, GPS_WARN_* make it a warning (RTK FIXED is exempt from the
    satellite warning only -- a good fix on few satellites is normal for the corrections it uses).
    """
    if fix_type is None:
        return "unknown", ["no GPS data"]
    if fix_type < GPS_MIN_FIX:
        return "bad", ["no 3D fix"]
    acc = h_acc_m if h_acc_m is not None and h_acc_m > 0 else None
    if acc is None:
        why = _gps_reasons(None, sats, hdop, None, GPS_MIN_SATS, GPS_MAX_HDOP)
        return ("warn" if why else "ok"), why + ["accuracy not reported"]
    poor = _gps_reasons(acc, sats, hdop, GPS_POOR_ACC_M, GPS_POOR_SATS, GPS_POOR_HDOP)
    if poor:
        return "bad", poor
    warn_sats = GPS_POOR_SATS if fix_type == RTK_FIXED else GPS_WARN_SATS
    why = _gps_reasons(acc, sats, hdop, GPS_GOOD_ACC_M, warn_sats, GPS_WARN_HDOP)
    return ("warn" if why else "ok"), why


def heading_quality(yaw_state, hdg_acc_deg):
    """('ok' | 'warn' | 'bad' | 'unknown', [short reasons]) for the boat's heading. There is no
    compass (COMPASS_USE=0): the heading IS the dual-antenna GPS yaw, so no yaw is no heading --
    bad, not a lesser grade. With yaw, the accuracy the GPS reports grades it against the
    HEADING_* guesses; an accuracy it does not report is ok, and says so."""
    if yaw_state == "unavailable":
        return "bad", ["no GPS heading — the boat has no compass"]
    if yaw_state == "not_provided":
        return "bad", ["GPS does not provide yaw — the boat has no compass"]
    if yaw_state != "ok":
        return "unknown", ["GPS yaw value out of range"]
    if hdg_acc_deg is None:
        return "ok", ["accuracy not reported"]
    if hdg_acc_deg > HEADING_BAD_ACC_DEG:
        return "bad", ["heading accuracy %.1f° > %.1f°" % (hdg_acc_deg, HEADING_BAD_ACC_DEG)]
    if hdg_acc_deg > HEADING_WARN_ACC_DEG:
        return "warn", ["heading accuracy %.1f° > %.1f°" % (hdg_acc_deg, HEADING_WARN_ACC_DEG)]
    return "ok", []


def no_data_reason(sec, what):
    """Why a section has no verdict: it was never heard, or not lately."""
    return "no %s data yet" % what if sec["age"] is None else "%s data is stale" % what


def quality_block(sec, what, judge):
    """{"quality", "quality_reasons"} for a /mav section (as _section built it). A section that is
    not fresh has no values, so it gets no verdict either: "unknown", never a stale grade that
    could be read as current. A fresh one is judged by `judge(sec)` -> (quality, [reasons])."""
    if not sec["ok"]:
        return {"quality": "unknown", "quality_reasons": [no_data_reason(sec, what)]}
    quality, reasons = judge(sec)
    return {"quality": quality, "quality_reasons": reasons}


def gps_verdicts(sec):
    """The four verdict keys for a /mav `gps` section: the position fix's, and the heading's."""
    fix = quality_block(sec, "GPS", lambda s: gps_quality(s["fix_type"], s["sats"], s["hdop"], s["h_acc_m"]))
    heading = quality_block(sec, "GPS", lambda s: heading_quality(s["yaw_state"], s["hdg_acc_deg"]))
    return dict(fix, heading_quality=heading["quality"], heading_reasons=heading["quality_reasons"])


# ---------------------------------------------------------------------------
# EKF and pre-arm verdicts
# ---------------------------------------------------------------------------

def ekf_quality(sec):
    """('ok' | 'warn' | 'bad', [reasons]) for a FRESH /mav `ekf` section. Poor: an uninitialised EKF, no
    attitude, no absolute horizontal position (or constant-position mode), or any variance above
    EKF_BAD_VARIANCE. Degraded: GPS glitching, or any variance from EKF_WARN_VARIANCE up to that. A
    variance the autopilot did not give (None) is not counted; if there is none at all the verdict
    rests on the flags and says so. The worst reasons come first."""
    bad, warn = [], []
    for flag in EKF_KEY_FLAGS:
        if sec[flag.name] != flag.good_when_set:
            (bad if flag.severity == "bad" else warn).append(flag.problem)
    known = _ekf_variances(sec, only_known=True)
    for label, v in known:
        if v > EKF_BAD_VARIANCE:
            bad.append("%s variance %.2f > %.1f" % (label, v, EKF_BAD_VARIANCE))
        elif v >= EKF_WARN_VARIANCE:
            warn.append("%s variance %.2f >= %.1f" % (label, v, EKF_WARN_VARIANCE))
    if bad:
        return "bad", bad + warn
    if warn:
        return "warn", warn
    return "ok", ([] if known else ["variances not reported"])


def _ekf_variances(sec, only_known=False):
    """[(label, variance)] in EKF_VARIANCES order; with only_known, without the ones that are None."""
    pairs = [(label, sec[key]) for key, label in EKF_VARIANCES]
    return [p for p in pairs if p[1] is not None] if only_known else pairs


def ekf_text(sec):
    """(summary, [detail lines]) for a FRESH `ekf` section: the largest variance for the tile's sub line;
    every variance and key flag for its tooltip."""
    known = _ekf_variances(sec, only_known=True)
    summary = "max variance %.2f (%s)" % max((v, label) for label, v in known) if known else "variances not reported"
    variances = ", ".join(("%s %.2f" % (label, v)) if v is not None else "%s not reported" % label
                          for label, v in _ekf_variances(sec))
    flags = ", ".join("%s %s" % (flag.label, "yes" if sec[flag.name] else "no") for flag in EKF_KEY_FLAGS)
    return summary, ["variances: " + variances, "flags: " + flags]


def ekf_verdicts(sec):
    """The `ekf` verdict keys: quality and its reasons, the words for the tile (summary, detail; None
    and empty while stale) and the two thresholds, which are configuration and so never stale."""
    out = quality_block(sec, "EKF", ekf_quality)
    out["summary"], out["detail"] = ekf_text(sec) if sec["ok"] else (None, [])
    out["warn_variance"], out["bad_variance"] = EKF_WARN_VARIANCE, EKF_BAD_VARIANCE
    return out


def prearm_state(present, enabled, health):
    """'pass' | 'fail' | 'disabled' | 'not_reported' from the three SYS_STATUS bits (see PREARM_VERDICTS)."""
    if not present:
        return "not_reported"
    if not enabled:
        return "disabled"
    return "pass" if health else "fail"


def prearm_quality(sec):
    quality, reasons = PREARM_VERDICTS[sec["state"]]
    return quality, list(reasons)


def prearm_verdicts(sec):
    return quality_block(sec, "pre-arm", prearm_quality)


#: section -> the value names it carries. A stale or never-heard section has all
#: of them null; `ok` and `age` ride beside them. `gps`, `ekf` and `prearm` also get the verdict keys of
#: gps_verdicts, ekf_verdicts and prearm_verdicts, which are never null: a stale section reads "unknown".
SECTIONS = {
    "hb": ("mode", "armed", "type"),
    "pos": ("lat", "lon", "heading", "groundspeed"),
    "gps": ("fix_type", "fix_name", "sats", "hdop", "h_acc_m", "v_acc_m", "hdg_acc_deg",
            "yaw_state", "yaw_deg", "lat", "lon", "alt_ellipsoid_m"),
    "rc": ("estop_pwm", "estop_engaged", "rssi"),
    "batt": ("voltage", "current", "remaining"),
    "ekf": ("flags",) + tuple(f.name for f in EKF_KEY_FLAGS) + tuple(key for key, _ in EKF_VARIANCES),
    "prearm": ("present", "enabled", "health", "state"),
}


class LagClock:
    """How far behind its own clock the autopilot's data arrives -- the drone GUI's link lag
    (Drone/rx26_uav/tools/scripts/gcs_radio.py:234-246), with one fix.

    A message carries the boot time it was SAMPLED at, so (this laptop's clock - that stamp) is the
    transport delay plus a fixed offset between the two clocks. The offset is constant, so the
    smallest value ever seen is the floor, and today's excess over it is how far behind the data is.
    It is relative to the best the link has done, and it cannot tell slow clock drift from delay.

    THE FIX: when the autopilot reboots its boot clock restarts near 0, the offset jumps up by the
    whole time the old floor had been running, and the drone's floor, which only ever goes down, would
    report that as lag for the rest of the session. A boot clock that goes backwards by more than
    LAG_BACKWARDS_MS (a reordered frame is milliseconds, a reboot is the old uptime) starts over.
    Not thread-safe by itself: MavState calls it under its own lock.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.floor = None           # smallest (local clock - boot clock), in seconds
        self.high_ms = None         # largest boot clock seen: what "backwards" is measured from
        self.samples = 0
        self.lag_s = None
        self.at = None              # local time of the last sample

    def sample(self, boot_ms, now):
        """One message's boot-time stamp (ms) and the time it arrived. 0 is not a stamp: a message that
        does not fill it in leaves 0, and a real 0 is the first millisecond of a boot."""
        if not boot_ms:
            return
        if self.high_ms is not None and boot_ms < self.high_ms - LAG_BACKWARDS_MS:
            self.reset()
        self.high_ms = boot_ms if self.high_ms is None else max(self.high_ms, boot_ms)
        offset = now - boot_ms / 1000.0
        if self.floor is None or offset < self.floor:
            self.floor = offset
        self.lag_s, self.at = offset - self.floor, now
        self.samples += 1

    def lag_ms(self, now, stale_s):
        """Whole milliseconds, or None until LAG_MIN_SAMPLES have set the floor (the first sample IS the
        floor, so its lag is 0 by construction) and whenever no stamp has arrived within `stale_s`."""
        if self.samples < LAG_MIN_SAMPLES or now - self.at > stale_s:
            return None
        return int(round(self.lag_s * 1000))


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
        self._lag = LagClock()
        #: message type -> handler. A handler returns None, the values of ONE section (a dict that names
        #: it in "section"), or a list of those: one message may fill several (SYS_STATUS: battery and pre-arm).
        self._handlers = {"HEARTBEAT": self._on_heartbeat, "GLOBAL_POSITION_INT": self._on_position,
                          "GPS_RAW_INT": self._on_gps, "RC_CHANNELS": self._on_rc,
                          "SYS_STATUS": self._on_sys_status, "EKF_STATUS_REPORT": self._on_ekf}

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
            # The AUTOPILOT's boot clock only: a companion computer's messages carry its own clock.
            self._lag.sample(getattr(msg, "time_boot_ms", None), now)
            if typ == "STATUSTEXT":
                self._on_statustext(msg, now)
                return
            handler = self._handlers.get(typ)
            self._store(handler(msg) if handler else None, now)

    def _store(self, result, now):
        """Record what a handler returned (see __init__): each section's values with their arrival time."""
        if result is None:
            return
        for values in ([result] if isinstance(result, dict) else result):
            rec = dict(values, t=now)
            self._rec[rec.pop("section")] = rec

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
        lat, lon = _lat_lon(msg)
        return {"section": "pos", "lat": lat, "lon": lon,
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
        # Position and height only from a fix that has them: below a 2D / 3D fix the autopilot may repeat
        # the last position it had, which would be a stale value wearing a measurement's clothes.
        lat, lon = _lat_lon(msg) if msg.fix_type >= GPS_FIX_2D else (None, None)
        alt = _known(msg.alt_ellipsoid, (0,), 1000.0) if msg.fix_type >= GPS_MIN_FIX else None
        return {"section": "gps", "fix_type": msg.fix_type, "lat": lat, "lon": lon, "alt_ellipsoid_m": alt,
                "fix_name": _enum_name("GPS_FIX_TYPE", msg.fix_type, "GPS_FIX_TYPE_"),
                "sats": _known(msg.satellites_visible, (UINT8_MAX,)),
                "hdop": _known(msg.eph, (0, UINT16_MAX), 100.0),
                "h_acc_m": _known(msg.h_acc, (0,), 1000.0),
                "v_acc_m": _known(msg.v_acc, (0,), 1000.0),
                "hdg_acc_deg": _known(msg.hdg_acc, (0,), 1e5),
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
        present, enabled, health = (bool(getattr(msg, "onboard_control_sensors_" + which) & PREARM_BIT)
                                    for which in ("present", "enabled", "health"))
        return [{"section": "batt",
                 "voltage": (None if msg.voltage_battery in (0, UINT16_MAX)
                             else msg.voltage_battery / 1000.0),
                 "current": None if msg.current_battery < 0 else msg.current_battery / 100.0,
                 "remaining": None if msg.battery_remaining < 0 else msg.battery_remaining},
                {"section": "prearm", "present": present, "enabled": enabled, "health": health,
                 "state": prearm_state(present, enabled, health)}]

    def _on_ekf(self, msg):
        values = {"section": "ekf", "flags": msg.flags}
        values.update((flag.name, bool(msg.flags & flag.bit)) for flag in EKF_KEY_FLAGS)
        values.update((key, _variance(getattr(msg, key))) for key, _ in EKF_VARIANCES)
        return values

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
            for name, verdicts in (("gps", gps_verdicts), ("ekf", ekf_verdicts), ("prearm", prearm_verdicts)):
                out[name].update(verdicts(out[name]))
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
                "lag_ms": self._lag.lag_ms(now, self.stale_s) if ok else None,
                "lag_warn_ms": LAG_WARN_MS,                      # configuration, never stale
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
    """GcsServer plus GET /mav, the same-origin guard and the Mission tab's library endpoints
    (answered locally by `mission`, a MissionLib; every other POST is forwarded to the Jetson).
    GcsServer.start() builds its HTTP server from self.handler(), so overriding that is all
    it takes."""

    def __init__(self, page_bytes, link, mav_fn, allowed_hosts=None, mission=None):
        super().__init__(page_bytes, link.state, self._action, link.download)
        self.link = link
        self.mav_fn = mav_fn
        self.allowed_hosts = allowed_hosts
        self.mission = mission or MissionLib(MISSION_BT_DIR, MISSION_DIR)

    def _action(self, path, payload):
        if path in LIB_PATHS:
            return self.mission.handle(path, payload)
        return self.link.action(path, payload)

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
                    self.rfile.read(min(int(self.headers.get("Content-Length") or 0),
                                        body_limit(self.path)))
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
