"""task1_panel — be Ekko for a Task 1 Disruptive run: lay out the course,
launch the sim, tell the boat what only the UAV can see, answer its checkpoints.

    PYTHONPATH=~/robotx_ws/src/rx26_asv/crusader_sim GZ_PARTITION=crusader_sim \
      python3 -u -m crusader_sim.task1_panel --port 8095 >/tmp/task1_panel.log 2>&1 &
    then, on Windows: http://localhost:8095

Runs in: WSL2 Ubuntu-22.04, on the HOST — plain python3, no ROS, not in docker.
It drives the sim from outside: gz_sim_up.sh / gz_sim_down.sh as children,
task1_goal through `docker exec crsd-sim`, gz-transport for the boat's TRUE
pose, and the RXL radio on udp 14555 to rxl_link_node.

VERIFIED 2026-09-30, all in --dry-run: the whole RXL handshake against a fake
boat speaking crusader_link/rxl_codec on udp 14655 (ACK, SEND CHANGES + ACK,
SEND CHANGES alone at a gate, auto-ACK, re-ask counting, 5 s resends, rewind
on a second START); the judge re-pairing on set_states and grading synthetic
odometry; the sensor views on synthetic gz messages over a private
GZ_PARTITION, lazy subscribe/unsubscribe included; the page in a browser.

VERIFIED 2026-09-30 against the REAL sim (three runs, TASK1_PANEL.cmd):
LAUNCH (--no-uav, all beacons unlit), ATTACH, STOP SIM, task1_goal through
docker exec, and rxl_link_node + the tree on udp 14555 — ACK, SEND CHANGES +
ACK at gate checkpoints (the boat replans: plan v2, v3, and circles the NEW
exit), 5 s resends. Sensor views on a real render: RTF ~1.0 with all three
on, same as off; the LiDAR view's port/starboard and fore/aft checked
against true buoy positions.

VERIFIED 2026-10-01, offline only (the panel started on a spare port against a
scratch GZ_PARTITION/CRUSADER_SIM_GEN, a synthetic feed datagram; no sim): the
EXIT checkpoint label (checkpoint_text), the default template, the page's JSON,
and the page's own JS in a browser — Boat's map canvas, the costmap layer, the
track colours and the track/fused pairing, driven by that synthetic feed.

TODO — what is still unverified or missing:
  * LIVE SIM CHECK of the 2026-10-01 additions: (1) the EXIT checkpoint banner
    on a real run of task1_avoid (checkpoint 3 of 2 gates reads "EXIT gate -
    confirm exit"); (2) the costmap layer against a real panel_feed costmap
    export (cell/LiDAR density and speed, the square size res_m, whether
    positions line up with the buoys); (3) track colours once the tracker
    publishes red_buoy/green_buoy/flashing_blue_buoy/steady_blue_buoy/
    unknown_buoy (with the side beacons off, unknown_buoy is the normal label),
    and the "boat says ?, UAV says red" fill, whose track<->fused pairing is by
    position (FUSE_MATCH_M in the page) because the report carries no track id;
    (4) task1_avoid itself: that Nav2 routes round all four obstacles.
  * saved layouts: save/load work; no delete.

WHY A HUMAN UAV. In the Disruptive tier every beacon in the world is unlit, so
the colours exist only in what the UAV reports (sim_uav.py:14-16). sim_uav plays
a UAV that reports the course file and acknowledges every checkpoint by itself;
this panel puts a person in that seat, so a colour can change halfway through a
run — the case the boat's replanning exists for, and one sim_uav can never
produce. The sim is started with --no-uav so the two never share the radio.

THE RADIO is tools/bench/uav_link.py, imported and never copied — its header
says why the handshake must exist once. What each control does on the air:

    sim up          UavLink(auto_confirm=False); send_plan(field)
    every 50 ms     poll()    USV_REACHED_GATE(seq) off the socket
    every 5 s       resend()  the boat aborts after 15 s without a field
                              (PassagePlanFresh max_age): two losses of margin
    START TASK 1    rewind(); send_plan(SENT field); then task1_goal. Without
                    rewind() checkpoint 1 of a second run is answered from the
                    first run's cache (uav_link.py:220-232)
    SEND CHANGES    send_plan(STAGED field). A changed field alone releases a
                    GATE checkpoint; the ENTRY checkpoint (seq 1) needs an ACK
    ACK             confirm()                the SENT field again, then the ack
    SEND + ACK      confirm(STAGED field)    field first, THEN the ack
    auto-ACK        confirm() as each new checkpoint is asked

    seq 1 = "ENTRY orbit done", seq k+1 = "gate k cleared", and the ask after
    the LAST gate is the EXIT checkpoint: with n gates in the SENT field (n =
    min(#red, #green)) seq n+1 is labelled "EXIT gate - confirm exit". The boat
    waits for it like any other before it circles the EXIT, and circles whatever
    the latest field calls EXIT, so a moved EXIT may still be sent at that ask.
    (Verified in a sim log: 3 gates, checkpoint 4 asked and awaited.) Each ask
    carries `what`: the plain sentence the page's banner shows.

Every field is the WHOLE course and buoy id = list index, so positions are
locked once the sim is up; only colours change mid-run. The panel is the
ground truth of the course, so each changed field is also handed to the
referee (Task1Judge.set_states) at the moment it goes on the air.

UAV POSITION ERROR, ALWAYS UNDER 1 M. A real UAV does not know where a buoy is to
the centimetre: its GPS, its attitude and its pixel-to-ground projection put
each buoy some distance from where the boat's depth camera maps it, and the
boat's fusePassage pairs the two within assoc_radius_m (crusader_bt
nav_math.hpp:457-500). The team's requirement is < 1 m, so the panel refuses
anything else (UAV_MAX_M = 0.99): the first R = 6 m runs (2026-09-30) put the
boat into a buoy both times — offsets over the 5 m association radius left
gates mis-paired from tracks of the WRONG buoys — which is a test of a UAV the
team does not have. Every buoy gets a FIXED random offset, uniform over a disc of
radius R (default 0.8 m), drawn when the sim comes up (LAUNCH or ATTACH) from a
seed, and every field the
panel sends carries the buoy at true + offset — the entry_*/exit_* fields too,
because they are just the ENTRY/EXIT buoys' reported positions. The course
file the world is built from and the referee both keep the TRUE positions; the
map shows the truth (solid) and what the UAV says (hollow, joined by a line).
A fixed offset is the dominant real error (a systematic georegistration bias);
the optional per-send jitter (sigma, metres per axis) adds a fresh draw to every
transmission on top, and the TOTAL offset (fixed + jitter) is clamped to
UAV_MAX_M, so no field ever carries a buoy 1 m or more from the truth. Jitter
changes the field's CONTENT, and rxl_link_node
versions the plan by content, so the boat replans (PlanChanged) on every
change: with jitter > 0 the periodic 5 s resend is therefore a send_plan of a
freshly jittered field, not uav_link's resend() — and the boat replans every 5 s.
Re-roll = a new random seed; allowed whenever no run is going.

THE BOAT'S OWN PICTURE (map layers, each toggled in the page, default on). The
page has TWO canvases sharing one pan/zoom: the top one is the truth (solid) and
what the UAV sent (hollow), with the referee's gate pairs and the boat's trail;
the "Boat's map" below it draws only what the boat believes, plus the true buoys as
faint dots to read it against. A checkbox overlays the boat's layers on the top map.
The panel has no ROS, so panel_feed.py (a node in crsd-sim, started by gz_rig_up.sh)
subscribes the boat's topics and sends compact JSON to udp 127.0.0.1:14556
(--feed-port); a thread here (panel_feed.FeedReceiver) keeps the newest packet.
    planned path   /crsd/nav/leg_status: the leg bt_runner is driving, dashed by
                   state like the ground station (green FOLLOWING, red BLOCKED,
                   yellow PLANNING/DEGRADED, grey STRAIGHT), the carrot as a
                   ring, the goal as a cross, and a NAV <state> badge
    boat's tracks  /crsd/world_targets: what target_tracker believes from the
                   boat's camera — squares, "#id label", faded by time since
                   seen. The OUTLINE is the boat's own colour vote from the label
                   (red_buoy, green_buoy, flashing_blue_buoy, steady_blue_buoy;
                   black_buoy for old data; unknown_buoy = not confidently seen,
                   the normal case with the side beacons off: a grey outline
                   with "?"). Where the fused passage gives that buoy a colour,
                   the FILL is the UAV's colour, so "boat says ?, UAV says red"
                   reads as a grey "?" outline filled red. Not the true buoys
                   (solid) and not the UAV's report (hollow circles)
    fused passage  /crsd/safe_passage_report: the tree's association of the UAV
                   field to those tracks (the UAV's colour, at the tracker's
                   position where a track matched, else the UAV's) — small
                   diamonds. Published only while a run is going
    costmap        the Nav2 local costmap, as panel_feed's "costmap" layer
                   {res_m, cells, lidar, stamp}; each position in the layer's
                   [x, y] course metres like a path point (a {x, y} object is
                   read too). cells = lethal/inscribed obstacles, translucent
                   squares of side res_m; lidar = the STVL LiDAR voxels, in a
                   second colour. Drawn UNDER the tracks and the path. A
                   panel_feed that predates the layer sends no key: the layer
                   draws nothing and says so
A layer whose topic is older than 2 s is STALE: the server returns its age and no
data, and the page draws nothing and says so. Never the last value.

HTTP (ThreadingHTTPServer, like tools/bt_view.py). Beacon states are the course
file's strings: off flash_red flash_green flash_blue(ENTRY) steady_blue(EXIT).
POSTs take a JSON object and return {"ok": bool, "error": str?}.

    GET  /                           the page (task1_panel.html beside this file)
    GET  /api/state?log=sim:N,radio:N,mission:N,judge:N&trail=GEN:N
                                     everything the page draws, incl. the judge's
                                     live verdict, the sensor views' status and
                                     `feed` (the boat's layers, with ages);
                                     logs and the trail come back from N onward
    GET  /api/sensor/<rgb|depth|lidar>.jpg
                                     the view's latest JPEG, age in X-Frame-Age;
                                     204 + X-Sensor-Status while it has none.
                                     Asking is what subscribes (panel_sensors.py)
    POST /api/layout   {buoys: [{x, y, state}]}   the Setup layout, whole
    POST /api/template {name}        a courses/*.yaml as the layout (default:
                                     task1_avoid, which a fresh panel — one with
                                     no panel.yaml yet — starts on, and the
                                     Load template list offers first)
    POST /api/clear
    POST /api/save     {name}        POST /api/load {name}
    POST /api/launch                 write panel.yaml, gz_sim_up.sh --no-uav
    POST /api/attach                 a sim is already up from panel.yaml
    POST /api/stop_sim               gz_sim_down.sh; back to Setup, layout kept
    POST /api/stage    {id, state}   stage a colour change (not sent)
    POST /api/send     POST /api/discard
    POST /api/start    POST /api/abort   (abort stops the SCRIPT, not the tree)
    POST /api/ack      POST /api/send_ack
    POST /api/auto_ack {on}
    POST /api/uav_error {radius_m?, jitter_m?, seed?}
                                     UAV position error (above); not while a run
                                     is going. With the sim up it redraws the
                                     offsets and sends the new field at once
    POST /api/reroll                 a new random seed, same rules

Test hooks: --rxl-endpoint, --feed-port and --port (keep a test clear of a real
run's 14555, 14556 and 8095); --dry-run (every child process is a harmless stub,
the mission a --dry-run-mission-s long printout); GZ_PARTITION and CRUSADER_SIM_GEN
in the environment keep a test's gz traffic and panel.yaml away from a live sim's.

LAKE MODE (--lake --datum LAT,LON). The same radio handshake against the REAL boat at a lake, with no
simulator: lake_panel.py (LakePanel, a subclass of Panel) and lake_panel.html. It runs inside the `asv`
container on the Jetson, takes the boat's pose / FCU state / hazards from panel_feed's extra layers, resends
the field only while a browser polls /api/state (dead-man), and starts lake_goal, which refuses unless the
pilot has armed the boat and chosen GUIDED. Its HTTP routes are the lake_panel.ACTIONS only: the sim's
launch / attach / stop_sim do not exist there. LAKE_MODE.md is the operator procedure.
The sim page and the lake page share panel_common.js / panel_common.css (served at /panel_common.*).
"""
import argparse
import json
import math
import os
import random
import re
import signal
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from crusader_sim import course as C
from crusader_sim.panel_feed import FEED_PORT, FeedReceiver
from crusader_sim.paths import _SRC_PKG, _SRC_REPO, courses_dir, generated_dir
from crusader_sim.sim_uav import plan_from_course
from crusader_sim.task1_judge import Task1Judge, format_verdict

ENTRY, EXIT = "flash_blue", "steady_blue"
RED, GREEN = "flash_red", "flash_green"
DEFAULT_TEMPLATE = "task1_avoid"      # courses/: the layout a fresh panel starts with, and what Load template offers first
LABEL = {RED: "RED", GREEN: "GREEN", ENTRY: "ENTRY", EXIT: "EXIT",
         "off": "BLACK"}
STATES = tuple(LABEL)
MAX_BUOYS = 10                  # handbook 3.3.2:9 "ten (10) buoys"; the message carries 10
MIN_SPACING_M = 1.0
START_CLEAR_M = 3.0
RESEND_S = 5.0
UAV_MAX_M = 0.99                # the UAV is never 1 m or more off: R, jitter and the total
UAV_LIMITS = {"radius_m": (0.0, UAV_MAX_M), "jitter_m": (0.0, UAV_MAX_M)}    # metres
UAV_DEFAULT = {"radius_m": 0.8, "jitter_m": 0.0, "seed": 1}
SEED_MAX = 2 ** 31 - 1
POLL_S = 0.05
ODOM_DEAD_S = 5.0
ASK_SILENT_S = 10.0             # the boat re-asks every 3 s; 10 s quiet = it stopped
ASK_GIVEUP_S = 120.0            # ... and gives up and drives on after 120 s
ODOM_TOPIC = "/model/crusader/odometry"
PANEL_COURSE = "panel"
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
SENSOR_RE = re.compile(r"^/api/sensor/(\w+)\.jpg$")
COMMON = {"/panel_common.js": "text/javascript; charset=utf-8",      # what the sim page and the lake page share,
          "/panel_common.css": "text/css; charset=utf-8"}            # served from beside this file
DOWNLOAD_RE = re.compile(r"^/api/course/([A-Za-z0-9_-]{1,40})\.yaml$")
TREE_RE = re.compile(r"^\[tree\]\s+(\S+)\s+([\d.]+)%\s+buoys\s+(\d+)/(\d+)\s+plan v(\d+)\s*(.*)$")
# = courses/task1_core.yaml's header; origin = tools/sitl/start_sitl.sh SITL_HOME
COURSE_HEAD = {"origin": {"lat": 1.28060, "lon": 103.85570}, "draft_m": 0.24,
               "depth_m": 5.0, "boat_start": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
               "tier": "disruptive"}
SILENT = ("field-change-released", "timed out", "boat stopped asking")
# UavLink's own lines for the two events the panel logs itself, with the
# checkpoint's name and wait (_on_ask, _close; uav_link.py:180, 216-217):
# passed through, every ask and every answer showed twice in the Radio log.
# Its "re-asked -> confirmed again" and "rewound" lines still pass.
LINK_ECHO_RE = re.compile(r"^checkpoint \d+(: the boat is asking| confirmed, field sent)")

STUB_UP = r"""
import sys, time
for i, s in enumerate(["sync", "generate", "gazebo", "transmitter", "SITL", "e-stop", "rig"], 1):
    print("\n=== %d/7 %s (dry run) ===" % (i, s), flush=True); time.sleep(0.3)
print("\n=== ready ===", flush=True)
"""
STUB_MISSION = r"""
import sys, time
T = float(sys.argv[1]); n = max(1, int(T))
print("[operator] armed (dry run)", flush=True)
for k in range(n):
    ph = ["approach", "entry_orbit", "gates", "exit_orbit"][min(3, 4 * k // n)]
    print("[tree] %-12s %5.1f%%  buoys %d/10  plan v1  " % (ph, 100.0 * k / n, min(10, k)), flush=True)
    time.sleep(T / n)
print("[result] outcome 1  dry run", flush=True)
print("         classified 10, passed correctly 0, 0 s", flush=True)
"""


# ------------------------------------------------------------------ course

def course_of(name, buoys, origin=None, boat_start=None, approach=None):
    """[{x, y, state}] -> a course dict in courses/*.yaml's schema (course.py). The sim's panel
    leaves origin/boat_start/approach alone (the SITL home, the origin); the lake panel gives the
    lake datum, where the boat was, and the approach point the operator clicked."""
    c = dict(COURSE_HEAD, name=name)
    if origin is not None:
        c["origin"] = {"lat": float(origin["lat"]), "lon": float(origin["lon"])}
    if boat_start is not None:
        c["boat_start"] = dict(boat_start)
    if approach is not None:
        c["approach"] = {"x": round(approach["x"], 2), "y": round(approach["y"], 2)}
    c["elements"] = [{"type": "robobuoy", "name": "b%d" % i, "x": round(b["x"], 2),
                      "y": round(b["y"], 2), "beacon": b["state"],
                      "side_beacon": False, "up_beacon": False}
                     for i, b in enumerate(buoys)]
    return c


def course_yaml(c):
    """The course as YAML text. Beacons are always quoted: bare `off` is a
    boolean to YAML 1.1 (task1_core.yaml quotes it for the same reason)."""
    o, bs = c["origin"], c["boat_start"]
    out = ["# written by crusader_sim.task1_panel; list index == RXL buoy id",
           "name: %s" % c["name"],
           "origin: {lat: %.7f, lon: %.7f}" % (o["lat"], o["lon"]),
           "draft_m: %s" % c["draft_m"], "depth_m: %s" % c["depth_m"],
           "boat_start: {x: %s, y: %s, yaw_deg: %s}" % (bs["x"], bs["y"], bs["yaw_deg"]),
           "tier: %s" % c["tier"]]
    if "approach" in c:
        out.append("approach: {x: %s, y: %s}" % (c["approach"]["x"], c["approach"]["y"]))
    out += ["", "elements:"]
    for e in c["elements"]:
        out.append('  - {type: robobuoy, name: %s, x: %.2f, y: %.2f, beacon: "%s", '
                   "side_beacon: false, up_beacon: false}"
                   % (e["name"], e["x"], e["y"], e["beacon"]))
    return "\n".join(out) + "\n"


def write_atomic(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    os.replace(tmp, path)


def layout_of(course):
    """A course dict -> ([{x, y, state}], note). Order kept; names dropped
    (they become b0..b9 by list order)."""
    out, notes = [], []
    for name, x, y, s, _side, _up in C.buoys(course):
        if s not in LABEL:
            notes.append("%s: unknown beacon %r -> BLACK" % (name, s))
            s = "off"
        out.append({"x": float(x), "y": float(y), "state": s})
    if len(out) > MAX_BUOYS:
        notes.append("kept the first %d of %d buoys" % (MAX_BUOYS, len(out)))
        out = out[:MAX_BUOYS]
    return out, "; ".join(notes)


def check_layout(buoys, start_clear_m=START_CLEAR_M):
    """(errors, warnings). Errors block LAUNCH; warnings do not. start_clear_m: how far a buoy
    must be from (0, 0), where the sim's boat starts; the lake passes 0 (its boat is wherever it is)."""
    errors, warnings = [], []
    states = [b["state"] for b in buoys]
    for s, what in ((ENTRY, "ENTRY (flashing blue)"), (EXIT, "EXIT (steady blue)")):
        if states.count(s) != 1:
            errors.append("need exactly 1 %s, have %d" % (what, states.count(s)))
    if len(buoys) != MAX_BUOYS:
        warnings.append("%d buoys; the handbook course has 10 (3.3.2:9)" % len(buoys))
    for i, a in enumerate(buoys):
        d0 = math.hypot(a["x"], a["y"])
        if d0 < start_clear_m:
            errors.append("b%d is %.1f m from the boat's start (min %.0f)" % (i, d0, start_clear_m))
        for j in range(i + 1, len(buoys)):
            d = math.hypot(a["x"] - buoys[j]["x"], a["y"] - buoys[j]["y"])
            if d < MIN_SPACING_M:
                errors.append("b%d and b%d are %.2f m apart (min %.1f)" % (i, j, d, MIN_SPACING_M))
    return errors, warnings


def field_problem(states):
    """Why this field cannot be sent, or None. The boat takes ENTRY/EXIT only
    from the message's entry_*/exit_* fields, so each needs exactly one buoy."""
    for s in (ENTRY, EXIT):
        if states.count(s) != 1:
            return "the field needs exactly 1 %s, has %d" % (LABEL[s], states.count(s))
    return None


def field_gates(states):
    """How many gates a field has: one per red/green pair, so min(#red, #green)."""
    return min(states.count(RED), states.count(GREEN))


def checkpoint_text(seq, gates):
    """(label, what) for the boat's seq-th ask, in a field of `gates` gates.

    The boat asks after the ENTRY orbit (seq 1) and after each gate (seq k+1 =
    gate k cleared), THE LAST GATE INCLUDED, and waits for the answer before it
    circles the EXIT (seen in a sim log: 3 gates -> checkpoint 4 asked and
    awaited). So seq == gates + 1 is the EXIT checkpoint. `label` is the table
    row; `what` is the plain sentence the banner shows. `gates` is counted in
    the SENT field, the one the boat holds when it asks."""
    last = seq == gates + 1
    if seq == 1:
        done, did = "ENTRY orbit done", "circled the ENTRY buoy"
    else:
        done, did = "gate %d cleared" % (seq - 1), "driven through gate %d" % (seq - 1)
    if last:
        label = "EXIT gate - confirm exit" if seq > 1 else done + " - confirm exit"
        nxt = "the EXIT buoy (as it stands in the field you last sent) before it circles it"
    else:
        label = "%s - confirm gate %d" % (done, seq)
        nxt = ("gate %d (its red and green buoys, as they stand in the field you last sent) "
               "before it drives it" % seq)
    return label, "The boat has %s and asks you to confirm %s." % (did, nxt)


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def uav_offsets(seed, radius_m, n):
    """n fixed (dx, dy) offsets [m], uniform over a disc of radius_m.

    The radius is R*sqrt(u), not R*u: the latter piles the points up near the
    centre (uniform in radius is not uniform in area). Buoy i's draw depends
    only on the seed and i, so moving or adding a buoy never re-rolls the rest;
    the generator is drawn from even at R = 0, so R only scales the same angles."""
    rng = random.Random(seed)
    out = []
    for _ in range(n):
        r, a = radius_m * math.sqrt(rng.random()), rng.uniform(0.0, 2.0 * math.pi)
        out.append((r * math.cos(a), r * math.sin(a)))
    return out


def clamp_offset(dx, dy, limit=UAV_MAX_M):
    """(dx, dy) scaled down to length `limit` if it is longer, direction kept."""
    n = math.hypot(dx, dy)
    return (dx, dy) if n <= limit else (dx * limit / n, dy * limit / n)


def check_uav(body):
    """(settings, error): the radius_m/jitter_m/seed present in a POST body,
    validated; any other key is an error rather than silently ignored."""
    out = {}
    for k, (lo, hi) in UAV_LIMITS.items():
        if k in body:
            if not (_num(body[k]) and lo <= body[k] <= hi):
                return None, "%s must be a number from %g to %g (UAV position error < 1 m)" % (
                    k, lo, hi)
            out[k] = float(body[k])
    if "seed" in body:
        sd = body["seed"]
        if not (isinstance(sd, int) and not isinstance(sd, bool) and 0 <= sd <= SEED_MAX):
            return None, "seed must be an integer from 0 to %d" % SEED_MAX
        out["seed"] = sd
    extra = sorted(set(body) - set(out))
    if extra:
        return None, "unknown setting: " + ", ".join(map(str, extra))
    return out, None


# ------------------------------------------------------------------ radio

def open_link(endpoint, on_log, on_ask):
    """A UavLink that also reports every request it sees.

    On the host nothing is colcon-installed, so crusader_common (geo) and
    crusader_link (rxl_codec) come from the source tree, beside tools/bench —
    the directory sim_uav.py:27-33 looks for in the container.

    The subclass only OBSERVES: _request is the hook uav_link split out of
    poll() (uav_link.py:163-171), and the panel needs each call to count the
    boat's re-asks, which the library answers but does not log.
    """
    for base in (os.environ.get("RX26_SRC"), _SRC_REPO,
                 os.path.expanduser("~/robotx_ws/src/rx26_asv")):
        if base and os.path.isfile(os.path.join(base, "tools", "bench", "uav_link.py")):
            break
    else:
        raise FileNotFoundError("tools/bench/uav_link.py not found; set RX26_SRC")
    for d in (os.path.join(base, "tools", "bench"), os.path.join(base, "crusader_link"),
              os.path.join(base, "crusader_common")):
        if d not in sys.path:
            sys.path.insert(0, d)
    import uav_link  # noqa: E402  (tools/bench, not a package)

    class PanelLink(uav_link.UavLink):
        def _request(self, seq):
            super()._request(seq)
            on_ask(seq)

    return PanelLink(endpoint, auto_confirm=False, on_log=on_log)


# ------------------------------------------------------------------ plumbing

class LogBuffer:
    """Numbered lines, bounded; the page asks for everything after line n.

    Its own leaf lock, never the panel's: UavLink calls on_log with ITS lock
    held (uav_link.py:258-262), and a Radio log that waited on the panel's lock
    there could deadlock against a panel action waiting on the link.
    Consecutive repeats collapse into one "(repeated N times)" line.
    """

    def __init__(self, maxlen=3000):
        self._lock = threading.Lock()
        self._lines = deque(maxlen=maxlen)
        self._seq = 0
        self._last, self._rep = None, 0

    def add(self, text):
        with self._lock:
            if text == self._last:
                self._rep += 1
                return
            if self._rep:
                self._push("  (previous line repeated %d more times)" % self._rep)
            self._last, self._rep = text, 0
            self._push(text)

    def _push(self, text):
        self._seq += 1
        self._lines.append((self._seq, "%s  %s" % (time.strftime("%H:%M:%S"), text)))

    def since(self, n):
        with self._lock:
            return {"seq": self._seq, "lines": [t for s, t in self._lines if s > n]}


class Proc:
    """One child in its own process group, its output line by line to a log.

    Exit is taken from wait(), NOT from EOF: gz_sim_up.sh leaves children
    running that may hold the pipe open, and EOF would then never come.
    """

    def __init__(self, name, argv, log, on_line=None, on_exit=None):
        # a name, not argv[1]: that is "exec" for docker and "-u" for a dry-run stub
        self.name, self.argv, self.log = name, argv, log
        self.on_line, self.on_exit = on_line, on_exit
        self.p = None

    def start(self):
        self.p = subprocess.Popen(self.argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  stdin=subprocess.DEVNULL, start_new_session=True,
                                  text=True, bufsize=1, errors="replace")
        pump = threading.Thread(target=self._pump, daemon=True)
        pump.start()
        threading.Thread(target=self._wait, args=(pump,), daemon=True).start()
        return self

    def _pump(self):
        for line in self.p.stdout:
            line = line.rstrip("\n")
            self.log.add(line)
            if self.on_line:
                _safe(self.on_line, line)

    def _wait(self, pump):
        code = self.p.wait()
        pump.join(2.0)               # let the last lines ([result]) land first
        self.log.add("[panel] %s exited with code %d" % (self.name, code))
        if self.on_exit:
            _safe(self.on_exit, code)

    def running(self):
        return self.p is not None and self.p.poll() is None

    def signal(self, sig):
        try:
            os.killpg(self.p.pid, sig)
        except (ProcessLookupError, PermissionError, AttributeError):
            pass


def _safe(fn, *a):
    try:
        fn(*a)
    except Exception:                     # noqa: BLE001 -- a callback must not kill its thread
        traceback.print_exc()


# ------------------------------------------------------------------ panel

class Panel:
    """All shared state, under self.lock.

    LOCK ORDER: tx_lock -> link's lock -> log locks, and tx_lock -> self.lock.
    Never call the link while holding self.lock: the link calls back into the
    panel (on_ask) and the two would wait on each other.
    """

    ACTIONS = None          # None: every act_* method is a POST route; a subclass lists its own
    PAGE = "task1_panel.html"

    def __init__(self, a):
        self.a = a
        # the frame the fields are placed in: the sim's course origin, or the lake's datum (main() sets a.origin)
        self.origin = dict(getattr(a, "origin", None) or COURSE_HEAD["origin"])
        self.lock = threading.RLock()
        self.tx_lock = threading.Lock()
        self.quit = threading.Event()
        self.logs = {k: LogBuffer() for k in ("sim", "radio", "mission", "judge")}
        self.errors = {}
        self.dir = self._panel_dir()
        self.layout, self.layout_rev, self.layout_note = [], 0, ""
        self.sim, self.sim_step, self.sim_detail, self.sim_up_t = "down", "", "", None
        self.launched = None
        self.proc_sim = self.proc_mission = None
        self.link = None
        self.sent, self.staged = [], []
        self.tx_count, self.resends, self.last_tx = 0, 0, None
        # the UAV's position error: settings, the offsets drawn from them when
        # the sim came up (one per buoy, list order), and the per-send jitter's
        # generator. The TRUE positions are self.launched / the course file.
        self.uav, self.offsets, self.jrng = dict(UAV_DEFAULT), [], random.Random()
        self.auto_ack = False
        self.checkpoints = []
        self.boat, self.odom_t, self.trail, self.trail_gen = None, None, [], 0
        self.judge, self.judge_t, self.judge_seen, self.verdict = None, 0.0, 0, None
        self.mission = self._fresh_mission()
        self.gz = None
        self._odom_on = False
        self.feed = FeedReceiver(a.feed_port)      # started in main(), like the sensors
        self.sensors = None
        self._init_gz()
        self._init_sensors()
        self._init_layout()

    def _panel_dir(self):
        """Where this panel keeps panel.yaml and its saved layouts."""
        return os.path.join(generated_dir(), "panel")

    def _init_gz(self):
        """gz-transport, for the boat's TRUE pose. The lake panel has none (the boat's pose
        comes from the feed) and skips it."""
        try:
            os.environ.setdefault("GZ_PARTITION", "crusader_sim")    # = gz_sim_up.sh:45
            from gz.transport13 import Node
            from gz.msgs10.odometry_pb2 import Odometry
            self.gz, self._Odometry = Node(), Odometry
        except Exception as e:                 # noqa: BLE001 -- the page shows it
            self.errors["gz"] = "gz-transport unavailable (%s): no boat pose" % e

    def _init_sensors(self):
        try:
            # its own import: numpy/cv2 are the only non-stdlib needs, and a host
            # without them still gets a working panel, just no pictures
            from crusader_sim.panel_sensors import SensorHub
            self.sensors = SensorHub(self.gz)
        except Exception as e:                 # noqa: BLE001 -- the page shows it
            self.errors["sensors"] = "sensor views unavailable: %s" % e

    def _init_layout(self):
        panel_yaml = os.path.join(self.dir, "panel.yaml")
        if os.path.isfile(panel_yaml):               # the last launched layout wins ...
            self._set_layout(*layout_of(C.load(panel_yaml)))
        else:                                        # ... a panel that never launched starts on the default
            self._load_file(os.path.join(courses_dir(), DEFAULT_TEMPLATE + ".yaml"),
                            "default template " + DEFAULT_TEMPLATE)

    @staticmethod
    def _fresh_mission():
        return {"running": False, "phase": "", "pct": 0.0, "buoys": "", "plan": "",
                "warning": "", "result": "", "operator": "", "exit_code": None}

    # ---------------------------------------------------------- commands
    def _script(self, name, *args):
        if self.a.dry_run:
            stub = STUB_UP if name == "gz_sim_up.sh" else 'print("dry run: nothing to stop")'
            return [sys.executable, "-u", "-c", stub] + list(args)
        return ["bash", os.path.join(_SRC_PKG, "scripts", name)] + list(args)

    def _mission_argv(self):
        if self.a.dry_run:
            return [sys.executable, "-u", "-c", STUB_MISSION, str(self.a.dry_run_mission_s)]
        return ["docker", "exec", self.a.container, "bash", "-c",
                "source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash"
                " && python3 -u -m crusader_sim.task1_goal --course %s --no-judge" % PANEL_COURSE]

    # ---------------------------------------------------------- setup
    def _set_layout(self, buoys, note=""):
        self.layout, self.layout_note = buoys, note
        self.layout_rev += 1

    def _editable(self):
        return self.sim in ("down", "failed")

    def act_layout(self, body):
        buoys = body.get("buoys")
        if not isinstance(buoys, list) or len(buoys) > MAX_BUOYS:
            return {"ok": False, "error": "buoys must be a list of at most %d" % MAX_BUOYS}
        clean = []
        for b in buoys:
            if not (isinstance(b, dict) and _num(b.get("x")) and _num(b.get("y"))
                    and abs(b["x"]) <= 500 and abs(b["y"]) <= 500 and b.get("state") in LABEL):
                return {"ok": False, "error": "bad buoy %r" % (b,)}
            clean.append({"x": float(b["x"]), "y": float(b["y"]), "state": b["state"]})
        with self.lock:
            if not self._editable():
                return {"ok": False, "error": "the layout is locked while the sim is %s" % self.sim}
            self._set_layout(clean)
            return {"ok": True, "rev": self.layout_rev}

    def act_template(self, body):
        name = body.get("name", DEFAULT_TEMPLATE)
        if not (isinstance(name, str) and NAME_RE.match(name)):
            return {"ok": False, "error": "bad template name"}
        return self._load_file(os.path.join(courses_dir(), name + ".yaml"), "template " + name)

    def act_load(self, body):
        name = body.get("name")
        if not (isinstance(name, str) and NAME_RE.match(name)):
            return {"ok": False, "error": "bad layout name"}
        return self._load_file(os.path.join(self.dir, "layouts", name + ".yaml"), "layout " + name)

    def _load_file(self, path, what):
        if not os.path.isfile(path):
            return {"ok": False, "error": "no %s" % what}
        try:
            buoys, note = layout_of(C.load(path))
        except Exception as e:                 # noqa: BLE001
            return {"ok": False, "error": "%s: %s" % (what, e)}
        with self.lock:
            if not self._editable():
                return {"ok": False, "error": "the layout is locked while the sim is %s" % self.sim}
            self._set_layout(buoys, "loaded %s%s" % (what, "; " + note if note else ""))
        return {"ok": True}

    def act_clear(self, _body):
        with self.lock:
            if not self._editable():
                return {"ok": False, "error": "the layout is locked while the sim is %s" % self.sim}
            self._set_layout([], "cleared")
        return {"ok": True}

    def act_save(self, body):
        name = body.get("name")
        if not (isinstance(name, str) and NAME_RE.match(name)):
            return {"ok": False, "error": "name: letters, digits, - and _ (max 40)"}
        with self.lock:
            text = course_yaml(course_of(name, self.layout))
        write_atomic(os.path.join(self.dir, "layouts", name + ".yaml"), text)
        return {"ok": True}

    # ---------------------------------------------------------- sim
    def act_launch(self, _body):
        with self.lock:
            if not self._editable():
                return {"ok": False, "error": "the sim is %s" % self.sim}
            errs, _ = check_layout(self.layout)
            if errs:
                return {"ok": False, "error": "; ".join(errs)}
            course = course_of(PANEL_COURSE, self.layout)
            path = os.path.join(self.dir, PANEL_COURSE + ".yaml")
            write_atomic(path, course_yaml(course))
            self.launched, self.sim, self.sim_step, self.sim_detail = course, "launching", "starting", ""
            self._reset_boat()
            argv = self._script("gz_sim_up.sh", "--course-file", path, "--no-uav")
        self.logs["sim"].add("[panel] launching: " + " ".join(argv[-4:]))
        try:
            self.proc_sim = Proc("gz_sim_up.sh", argv, self.logs["sim"], self._on_sim_line, self._on_sim_exit).start()
        except OSError as e:
            with self.lock:
                self.sim, self.sim_detail = "failed", str(e)
            return {"ok": False, "error": str(e)}
        return {"ok": True}

    def _on_sim_line(self, line):
        m = re.match(r"^=== (.+) ===$", line.strip())
        if m:
            with self.lock:
                self.sim_step = m.group(1)

    def _on_sim_exit(self, code):
        with self.lock:
            if self.sim != "launching":
                return                          # stopped while launching
            if code != 0:
                self.sim, self.sim_detail = "failed", "gz_sim_up.sh exited %d, see the Sim log" % code
                return
            self.sim, self.sim_up_t = "up", time.time()
        self._radio_up()

    def act_attach(self, _body):
        path = os.path.join(self.dir, PANEL_COURSE + ".yaml")
        if not os.path.isfile(path):
            return {"ok": False, "error": "no %s: nothing was launched from this panel" % path}
        with self.lock:
            if not self._editable():
                return {"ok": False, "error": "the sim is %s" % self.sim}
            buoys, note = layout_of(C.load(path))
            self._set_layout(buoys, "attached to " + path + ("; " + note if note else ""))
            self.launched = course_of(PANEL_COURSE, buoys)
            self.sim, self.sim_step, self.sim_up_t = "up", "attached", time.time()
            self._reset_boat()
        self._radio_up()
        return {"ok": True}

    def act_stop_sim(self, _body):
        with self.lock:
            if self.sim in ("down", "stopping"):
                return {"ok": False, "error": "the sim is %s" % self.sim}
            self.sim, self.sim_step = "stopping", "stopping"
        self._stop_mission()
        if self.proc_sim and self.proc_sim.running():
            self.proc_sim.signal(signal.SIGTERM)
        self._radio_down()

        def done(_code):
            with self.lock:
                self.sim, self.sim_step = "down", ""
        Proc("gz_sim_down.sh", self._script("gz_sim_down.sh"), self.logs["sim"], on_exit=done).start()
        return {"ok": True}

    # ---------------------------------------------------------- radio
    def _radio_up(self):
        with self.lock:
            states = [e["beacon"] for e in self.launched["elements"]]
            self.sent, self.staged = list(states), list(states)
            self.checkpoints, self.tx_count, self.resends, self.last_tx = [], 0, 0, None
            self.errors.pop("radio", None)
            note = self._draw_offsets()
        self.logs["radio"].add(note)
        try:
            link = open_link(self.a.rxl_endpoint, self._on_link_log, self._on_ask)
        except Exception as e:                  # noqa: BLE001 -- the page shows it
            with self.lock:
                self.errors["radio"] = "radio failed to open: %s" % e
            self.logs["radio"].add("radio failed to open: %s" % e)
            return
        with self.tx_lock:
            self.link = link
        self._transmit(states, "initial field")
        if self.gz is not None and not self._odom_on:
            self.gz.subscribe(self._Odometry, ODOM_TOPIC, self._on_odom)
            self._odom_on = True

    def _on_link_log(self, line):
        """UavLink's log lines -> the Radio log. Called with the LINK's lock held,
        so it touches only the log's own leaf lock (see LogBuffer)."""
        if not LINK_ECHO_RE.match(line):
            self.logs["radio"].add(line)

    def _radio_down(self):
        with self.tx_lock:
            link, self.link = self.link, None
        if link is not None:
            try:
                link.conn.close()
            except Exception:                   # noqa: BLE001
                pass
            self.logs["radio"].add("radio closed")
        if self.gz is not None and self._odom_on:
            self.gz.unsubscribe(ODOM_TOPIC)
            self._odom_on = False

    # ---------------------------------------------------------- UAV error
    def _draw_offsets(self):
        """Draw the UAV's fixed offsets for the launched buoys from the current
        settings; returns the Radio-log line saying what they are. Call with
        self.lock held."""
        u = self.uav
        self.offsets = uav_offsets(u["seed"], u["radius_m"], len(self.launched["elements"]))
        self.jrng = random.Random(u["seed"] ^ 0x5EED)
        line = "UAV position error: R %.1f m, seed %d, offsets %s (max %.2f m)" % (
            u["radius_m"], u["seed"],
            " ".join("b%d %.1f" % (i, math.hypot(*o)) for i, o in enumerate(self.offsets)),
            max((math.hypot(*o) for o in self.offsets), default=0.0))
        if u["jitter_m"] > 0:
            line += ("; JITTER %.2f m per axis on every send (total offset clamped to %.2f m), so "
                     "every resend changes the field and the boat replans every %g s"
                     % (u["jitter_m"], UAV_MAX_M, RESEND_S))
        return line

    def _set_uav(self, new):
        """Apply validated settings. With the sim up the offsets are redrawn and
        the new field goes out at once; before LAUNCH they are just remembered."""
        with self.lock:
            if self.sim not in ("down", "failed", "up"):
                return {"ok": False, "error": "the sim is %s" % self.sim}
            if self.mission["running"]:
                return {"ok": False, "error": "locked while a run is going"}
            self.uav.update(new)
            live = self.sim == "up"
            note = self._draw_offsets() if live else None
            states = list(self.sent)
        if live:
            self.logs["radio"].add(note)
            self._transmit(states, "UAV error changed")
        return {"ok": True, "seed": self.uav["seed"]}

    def act_uav_error(self, body):
        new, err = check_uav(body)
        return {"ok": False, "error": err} if err else self._set_uav(new)

    def act_reroll(self, _body):
        return self._set_uav({"seed": random.SystemRandom().randrange(1, SEED_MAX)})

    def _reported(self):
        """[(x, y)] where the UAV says each buoy is, before any per-send jitter.
        Call with self.lock held."""
        return [(e["x"] + dx, e["y"] + dy)
                for e, (dx, dy) in zip(self.launched["elements"], self.offsets)]

    def _plan(self, states):
        """The field as the UAV sends it: TRUE position + the buoy's fixed
        offset + (jitter > 0) a fresh draw per buoy, per call, the total clamped
        under 1 m. ENTRY and EXIT come out of plan_from_course as the reported
        positions of those buoys."""
        with self.lock:
            sj = self.uav["jitter_m"]
            pos = []
            for e, (dx, dy) in zip(self.launched["elements"], self.offsets):
                if sj > 0:
                    dx, dy = clamp_offset(dx + self.jrng.gauss(0.0, sj), dy + self.jrng.gauss(0.0, sj))
                pos.append((e["x"] + dx, e["y"] + dy))
        return plan_from_course(course_of(PANEL_COURSE, [
            {"x": x, "y": y, "state": s} for (x, y), s in zip(pos, states)], origin=self.origin))

    def _operator_present(self):
        """May the panel speak for the UAV right now (resends, auto-ACK)? The sim's always may;
        the lake panel's dead-man says no once the browser has stopped polling."""
        return True

    def _gates(self):
        """The gate count the checkpoint labels use: the SENT field's (one per red/green pair).
        The lake panel prefers the boat's own count from its passage report."""
        return field_gates(self.sent)

    def _resend(self, link):
        """The periodic retransmission. Call with tx_lock held. At jitter 0 it is
        uav_link's resend() — byte-identical content, so the boat does not
        replan. With jitter every retransmission must carry a NEW field, which
        resend() (it replays the cached one) cannot do."""
        with self.lock:
            jittered, states = self.uav["jitter_m"] > 0, list(self.sent)
        if not jittered:
            return link.resend()
        link.send_plan(*self._plan(states), quiet=True)
        return True

    def _transmit(self, states, why):
        """Put a field on the air (no ack). Call with neither lock held."""
        with self.tx_lock:
            if self.link is None:
                return False, "the radio is not up"
            self.link.send_plan(*self._plan(states), quiet=True)
            with self.lock:
                self._note_tx(states, why)
        return True, "sent"

    def _note_tx(self, states, why, ack_reply=None):
        """Bookkeeping after a field went out. Call with self.lock held."""
        self.tx_count += 1
        self.last_tx = time.time()
        changes = ["b%d %s->%s" % (i, LABEL[o], LABEL[n])
                   for i, (o, n) in enumerate(zip(self.sent, states)) if o != n]
        if changes:
            self.sent = list(states)
            if self.judge is not None:
                self.judge.set_states({"b%d" % i: s for i, s in enumerate(states)})
                self._judge_log()
            for rec in self._open():
                rec["changed"] = True
                # a CHANGED field alone releases a gate checkpoint (not ENTRY's)
                if rec["seq"] > 1 and ack_reply is None:
                    self._close(rec, "field-change-released")
        if why:
            self.logs["radio"].add("field sent (%s): ENTRY b%d, EXIT b%d; %s" % (
                why, states.index(ENTRY), states.index(EXIT),
                ", ".join(changes) if changes else "no change"))

    def _open(self):
        return [r for r in self.checkpoints if r["reply"] is None]

    def _close(self, rec, reply):
        rec["reply"], rec["answered"] = reply, time.time()
        self.logs["radio"].add("checkpoint %d (%s): %s after %.0f s" % (
            rec["seq"], rec["label"], reply, rec["answered"] - rec["asked"]))

    def _on_ask(self, seq):
        """Every USV_REACHED_GATE the link takes off the socket (poll thread)."""
        now = time.time()
        with self.lock:
            rec = next((r for r in self.checkpoints if r["seq"] == seq), None)
            if rec is None:
                for r in self._open():
                    if r["seq"] < seq:
                        self._close_silent(r)
                label, what = checkpoint_text(seq, self._gates())
                rec = {"seq": seq, "label": label, "what": what, "asked": now,
                       "last_ask": now, "asks": 1, "answered": None, "reply": None,
                       "changed": False, "auto_tried": False, "late_asks": 0}
                self.checkpoints.append(rec)
                self.logs["radio"].add("BOAT ASKS checkpoint %d: %s" % (seq, rec["label"]))
            elif rec["reply"] is None:
                rec["asks"] += 1
                rec["last_ask"] = now
            elif rec["reply"] in SILENT:          # we gave up on it too early
                rec.update(reply=None, answered=None, last_ask=now, auto_tried=False)
                rec["asks"] += 1
                self.logs["radio"].add("checkpoint %d asked again: reopened" % seq)
            else:
                rec["late_asks"] += 1             # uav_link re-acked it (uav_link.py:174-177)

    def _close_silent(self, rec):
        if rec["changed"] and rec["seq"] > 1:
            why = "field-change-released"
        elif rec["last_ask"] - rec["asked"] >= ASK_GIVEUP_S - ASK_SILENT_S:
            why = "timed out"
        else:
            why = "boat stopped asking"
        self._close(rec, why)

    def _answer(self, reply, staged):
        with self.lock:
            if not self._open():
                return {"ok": False, "error": "the boat is not waiting on a checkpoint"}
            states = list(self.staged if staged else self.sent)
            if staged:
                problem = field_problem(states)
                if problem or states == self.sent:
                    return {"ok": False, "error": problem or "nothing staged"}
        with self.tx_lock:
            if self.link is None:
                return {"ok": False, "error": "the radio is not up"}
            r = self.link.confirm(*self._plan(states)) if staged else self.link.confirm()
            m = re.match(r"checkpoint (\d+) confirmed$", r)
            with self.lock:
                # the field line first: it went on the air before the ack did,
                # and the Radio log must not read as if the boat left on the old one
                if not r.startswith("nothing"):
                    self._note_tx(states, "SEND CHANGES + ACK" if staged else None, ack_reply=reply)
                if m:
                    rec = next((x for x in self.checkpoints if x["seq"] == int(m.group(1))), None)
                    if rec is not None and rec["reply"] is None:
                        self._close(rec, reply)
        return {"ok": bool(m), "error": None if m else r}

    def act_ack(self, _body):
        return self._answer("ack", staged=False)

    def act_send_ack(self, _body):
        return self._answer("changes+ack", staged=True)

    def act_stage(self, body):
        i, s = body.get("id"), body.get("state")
        with self.lock:
            if self.sim != "up":
                return {"ok": False, "error": "the sim is %s" % self.sim}
            if not (isinstance(i, int) and not isinstance(i, bool) and 0 <= i < len(self.staged)
                    and s in LABEL):
                return {"ok": False, "error": "bad id/state"}
            self.staged[i] = s
        return {"ok": True}

    def act_discard(self, _body):
        with self.lock:
            self.staged = list(self.sent)
        return {"ok": True}

    def act_send(self, _body):
        with self.lock:
            states = list(self.staged)
            problem = field_problem(states) or (None if states != self.sent else "nothing staged")
        if problem:
            return {"ok": False, "error": problem}
        ok, r = self._transmit(states, "SEND CHANGES")
        return {"ok": ok, "error": None if ok else r}

    def act_auto_ack(self, body):
        if not isinstance(body.get("on"), bool):
            return {"ok": False, "error": "on must be true/false"}
        with self.lock:
            self.auto_ack = body["on"]
        self.logs["radio"].add("auto-ACK %s" % ("ON" if body["on"] else "off"))
        return {"ok": True}

    # ---------------------------------------------------------- mission
    def act_start(self, _body):
        with self.lock:
            if self.sim != "up" or self.link is None:
                return {"ok": False, "error": "the sim/radio is not up"}
            if self.proc_mission and self.proc_mission.running():
                return {"ok": False, "error": "a run is already going"}
        with self.tx_lock:
            self.link.rewind()
        with self.lock:
            self.checkpoints = []
            course = course_of(PANEL_COURSE, [
                {"x": e["x"], "y": e["y"], "state": s}
                for e, s in zip(self.launched["elements"], self.sent)])
            self.judge = Task1Judge(course, circle_radius_m=8.0, echo=False)
            self.judge_seen, self.verdict = 0, None
            self.mission = self._fresh_mission()
            self.mission["running"] = True
            self.trail, self.trail_gen = [], self.trail_gen + 1
            states = list(self.sent)
        self.logs["judge"].add("--- new run ---")
        self._transmit(states, "START")          # the field must be there before the goal
        self.proc_mission = Proc("task1_goal", self._mission_argv(), self.logs["mission"],
                                 self._on_mission_line, self._on_mission_exit).start()
        return {"ok": True}

    def _on_mission_line(self, line):
        with self.lock:
            m = TREE_RE.match(line)
            if m:
                self.mission.update(phase=m.group(1), pct=float(m.group(2)),
                                    buoys="%s/%s" % (m.group(3), m.group(4)),
                                    plan="v" + m.group(5), warning=m.group(6))
            elif line.startswith("[result]"):
                self.mission["result"] = line
            elif line.lstrip().startswith("classified") and self.mission["result"]:
                self.mission["result"] += "\n" + line
            elif line.startswith("[operator]"):
                self.mission["operator"] = line

    def _on_mission_exit(self, code):
        with self.lock:
            self.mission["running"], self.mission["exit_code"] = False, code
            if self.judge is not None:
                self.verdict = format_verdict(self.judge.verdict())
        if self.verdict:
            for line in self.verdict.splitlines():
                self.logs["judge"].add(line)

    def _stop_mission(self):
        p = self.proc_mission
        if p is None or not p.running():
            return
        p.signal(signal.SIGINT)
        if not self.a.dry_run:
            # killing the docker CLI does not stop the process in the container
            subprocess.run(["docker", "exec", self.a.container, "pkill", "-INT", "-f",
                            "crusader_sim.task1_goal"], timeout=10,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def act_abort(self, _body):
        self._stop_mission()
        return {"ok": True}

    # ---------------------------------------------------------- boat
    def _reset_boat(self):
        self.boat, self.odom_t, self.trail = None, None, []
        self.trail_gen += 1
        # the last launch's offsets must not outlive it: _radio_up draws the new
        # ones a moment AFTER sim turns "up", and state() shows no run until then
        self.offsets = []

    def _on_odom(self, msg):
        try:
            p, q = msg.pose.position, msg.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            now = time.time()
            with self.lock:
                self.boat, self.odom_t = (p.x, p.y, yaw), now
                if not self.trail or math.hypot(p.x - self.trail[-1][0], p.y - self.trail[-1][1]) >= 0.2:
                    self.trail.append((round(p.x, 2), round(p.y, 2)))
                    if len(self.trail) > 5000:
                        self.trail, self.trail_gen = self.trail[-2500:], self.trail_gen + 1
                if self.mission["running"] and self.judge is not None and now - self.judge_t >= 0.05:
                    self.judge_t = now
                    self.judge.update(p.x, p.y, yaw)
                    self._judge_log()
        except Exception:                       # noqa: BLE001
            traceback.print_exc()

    def _judge_log(self):
        """The judge's new events into the Judge log. Call with self.lock held.
        From set_states as well as update(): a re-pairing must show when the
        field goes out, not on the next pose — which in a stalled sim never comes."""
        for ev in self.judge.events[self.judge_seen:]:
            self.logs["judge"].add(ev)
        self.judge_seen = len(self.judge.events)

    # ---------------------------------------------------------- worker
    def loop(self):
        slow = 0.0
        while not self.quit.is_set():
            _safe(self._radio_tick)
            if time.time() >= slow:
                slow = time.time() + 0.5
                with self.lock:
                    for rec in self._open():
                        if time.time() - rec["last_ask"] > ASK_SILENT_S:
                            self._close_silent(rec)
            self.quit.wait(POLL_S)

    def _radio_tick(self):
        with self.tx_lock:
            link = self.link
            if link is None:
                return
            link.poll()
            with self.lock:
                due = self.last_tx is None or time.time() - self.last_tx >= RESEND_S
            if due and self._operator_present() and self._resend(link):
                with self.lock:
                    self.resends += 1
                    self.last_tx = time.time()
        with self.lock:
            rec = (next((r for r in self._open() if not r["auto_tried"]), None)
                   if self.auto_ack and self._operator_present() else None)
            if rec is not None:
                rec["auto_tried"] = True
        if rec is not None:
            self._answer("auto-ack", staged=False)

    # ---------------------------------------------------------- state
    def _logs_since(self, q):
        """The four log buffers from the page's `log=sim:N,radio:N,...` cursor onward."""
        logs = {}
        want = dict(p.split(":", 1) for p in (q.get("log", [""])[0]).split(",") if ":" in p)
        for k, buf in self.logs.items():
            try:
                logs[k] = buf.since(int(want.get(k, 0)))
            except ValueError:
                logs[k] = buf.since(0)
        return logs

    def _trail_since(self, q):
        """The boat's trail from the page's `trail=GEN:N` cursor onward. Call with self.lock held."""
        tg, tn = (q.get("trail", ["-1:0"])[0] + ":0").split(":")[:2]
        start = int(tn) if tg == str(self.trail_gen) and tn.isdigit() else 0
        return {"gen": self.trail_gen, "from": start, "pts": self.trail[start:]}

    def _field_state(self, field_live):
        """The blocks the sim page and the lake page draw the same way: `run` (the field as sent
        and staged, with the UAV's offsets), `uav` (the position-error settings) and the
        checkpoints. field_live: a field is on the air. Call with self.lock held."""
        now = time.time()
        run = None
        if (field_live and self.launched is not None
                and len(self.offsets) == len(self.launched["elements"])):
            run = {"buoys": [{"x": e["x"], "y": e["y"], "ux": u[0], "uy": u[1], "sent": s,
                              "staged": t}
                             for e, u, s, t in zip(self.launched["elements"], self._reported(),
                                                   self.sent, self.staged)],
                   "unsent": sum(1 for s, t in zip(self.sent, self.staged) if s != t),
                   "problem": field_problem(self.staged)}
        # the offsets the run is using, or — before LAUNCH — the ones this
        # seed WILL give the layout (a pure function of seed/R/index)
        offs = self.offsets if run else uav_offsets(
            self.uav["seed"], self.uav["radius_m"], len(self.layout))
        uav = dict(self.uav, offsets=[[round(dx, 3), round(dy, 3)] for dx, dy in offs],
                   locked=self.mission["running"] or self.sim in ("launching", "stopping"),
                   live=run is not None)
        ask = self._open()
        ask = ask[-1] if ask else None
        # copies: the JSON is built after this lock is released, while the
        # poll thread may still be counting re-asks into these records
        return {"run": run, "uav": uav,
                "checkpoint": None if ask is None else dict(ask, waiting_s=now - ask["asked"]),
                "checkpoints": [dict(r) for r in self.checkpoints]}

    def _link_pending(self):
        """(radio up?, the checkpoint the boat is waiting on). Read BEFORE self.lock is taken:
        the link's own lock must never be waited on under it (class docstring)."""
        link = self.link
        return link is not None, link.status()["pending"] if link is not None else None

    def _radio_state(self, up, pending):
        """The `radio` block. Call with self.lock held."""
        return {"up": up, "endpoint": self.a.rxl_endpoint,
                "sent": self.tx_count, "resends": self.resends,
                "last_tx_age": time.time() - self.last_tx if self.last_tx else None,
                "auto_ack": self.auto_ack, "pending": pending}

    def state(self, q):
        radio_up, pending = self._link_pending()
        logs = self._logs_since(q)
        now = time.time()
        feed = self.feed.view()                  # its own leaf lock, never under self.lock
        with self.lock:
            errs, warns = check_layout(self.layout)
            odom_age = now - self.odom_t if self.odom_t else None
            alive = None
            if self.sim == "up":
                ref = self.odom_t or self.sim_up_t or now
                alive = now - ref < ODOM_DEAD_S
            return {
                "dry_run": self.a.dry_run,
                "errors": dict(self.errors, **({"feed": feed["error"]} if feed["error"] else {})),
                "sim": {"state": self.sim, "step": self.sim_step, "detail": self.sim_detail,
                        "alive": alive, "odom_age": odom_age},
                "layout": {"rev": self.layout_rev, "buoys": self.layout, "note": self.layout_note,
                           "errors": errs, "warnings": warns},
                "templates": _names(courses_dir()), "default_template": DEFAULT_TEMPLATE,
                "layouts": _names(os.path.join(self.dir, "layouts")),
                "radio": self._radio_state(radio_up, pending),
                "mission": dict(self.mission), "verdict": self.verdict,
                # the referee's live view, so the page (and a test) can see a
                # colour change re-pair the gates the moment it goes on the air
                "judge": None if self.judge is None else self.judge.verdict(),
                "sensors": None if self.sensors is None else self.sensors.status(),
                "feed": feed,
                "boat": None if self.boat is None else {
                    "x": self.boat[0], "y": self.boat[1], "yaw": self.boat[2], "age": odom_age},
                "trail": self._trail_since(q),
                "logs": logs,
                **self._field_state(self.launched is not None and self.sim in ("up", "stopping")),
            }

    def download(self, name):
        """(bytes, filename) for GET /api/course/<name>.yaml, or None. The sim's panel has none."""
        return None

    def sensor_frame(self, name):
        """(jpeg or None, age_s, why-no-frame) for GET /api/sensor/<name>.jpg;
        KeyError for a view that does not exist."""
        if self.sensors is None:
            if name not in ("rgb", "depth", "lidar"):
                raise KeyError(name)
            return None, None, self.errors.get("sensors", "unavailable")
        return self.sensors.get(name)

    def shutdown(self):
        """Stop the mission script, close the radio, drop the sensor
        subscriptions. The sim is NOT stopped."""
        self.quit.set()
        _safe(self._stop_mission)
        _safe(self._radio_down)
        if self.sensors is not None:
            self.sensors.stop()
        self.feed.stop()


def _names(d):
    try:
        return sorted(f[:-5] for f in os.listdir(d) if f.endswith(".yaml"))
    except OSError:
        return []


def _header_safe(s):
    """An error text as an HTTP header value: latin-1, one line, bounded.
    http.server encodes headers as latin-1 and raises on anything else."""
    return re.sub(r"[^\x20-\x7e]", "?", str(s))[:200]


# ------------------------------------------------------------------ http

def make_handler(panel):
    here = os.path.dirname(os.path.abspath(__file__))
    page = os.path.join(here, panel.PAGE)
    # every act_* method is a POST route, unless the panel lists its own (the lake panel does:
    # it inherits the sim's launch/attach/stop acts and none of them may be reachable there)
    names = panel.ACTIONS if panel.ACTIONS is not None else [n[4:] for n in dir(panel) if n.startswith("act_")]
    actions = {"/api/" + n: getattr(panel, "act_" + n) for n in names}

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            u = urlparse(self.path)
            if u.path in ("/", "/index.html"):
                # re-read per request: a sync of the checkout shows up on reload
                with open(page, "rb") as f:
                    return self._send(f.read(), "text/html; charset=utf-8")
            if u.path in COMMON:
                with open(os.path.join(here, u.path[1:]), "rb") as f:
                    return self._send(f.read(), COMMON[u.path])
            if u.path == "/api/state":
                return self._send(json.dumps(panel.state(parse_qs(u.query))).encode(),
                                  "application/json")
            m = DOWNLOAD_RE.match(u.path)
            if m:
                got = panel.download(m.group(1))
                if got is None:
                    return self.send_error(404)
                data, name = got
                return self._send(data, "text/yaml; charset=utf-8",
                                  {"Content-Disposition": 'attachment; filename="%s"' % name})
            m = SENSOR_RE.match(u.path)
            if m:
                try:
                    jpeg, age, why = panel.sensor_frame(m.group(1))
                except KeyError:
                    return self.send_error(404)
                if jpeg is None:
                    # 204, not an old frame: the page shows `why` in its place
                    return self._send(b"", None, {"X-Sensor-Status": _header_safe(why)}, 204)
                return self._send(jpeg, "image/jpeg", {"X-Frame-Age": "%.2f" % age})
            self.send_error(404)
            return None

        def do_POST(self):
            fn = actions.get(urlparse(self.path).path)
            if fn is None:
                return self.send_error(404)
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(min(n, 65536)) or b"{}") if n else {}
                if not isinstance(body, dict):
                    raise ValueError("body must be a JSON object")
            except ValueError as e:
                return self._send(json.dumps({"ok": False, "error": "bad request: %s" % e}).encode(),
                                  "application/json")
            try:
                result = fn(body)
            except Exception as e:               # noqa: BLE001
                traceback.print_exc()
                result = {"ok": False, "error": "%s failed: %s" % (self.path, e)}
            return self._send(json.dumps(result).encode(), "application/json")

        def _send(self, body, ctype, headers=None, code=200):
            try:
                self.send_response(code)
                if ctype:
                    self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                for k, v in (headers or {}).items():
                    self.send_header(k, v)
                self.end_headers()
                self.wfile.write(body)
            except ConnectionError:
                pass                              # tab closed mid-write

        def log_message(self, *_):
            pass                                  # 4 Hz polling would drown the log

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=8095,
                    help="clear of 8090 ground station, 8085 bt_view, 8080/8081 viewers")
    ap.add_argument("--rxl-endpoint", default="udpout:127.0.0.1:14555",
                    help="rxl_link_node's rxl_endpoint, from the aircraft's side")
    ap.add_argument("--feed-port", type=int, default=FEED_PORT,
                    help="udp port panel_feed (in crsd-sim) sends the boat's map layers to; "
                         "a test panel needs its own, as with --port")
    ap.add_argument("--container", default=os.environ.get("RX26_CONTAINER", "crsd-sim"))
    ap.add_argument("--lake", action="store_true",
                    help="LAKE MODE: the REAL boat, you play the UAV. Runs inside `asv` on the Jetson "
                         "(lake_panel.py, LAKE_MODE.md); no simulator, no gazebo")
    ap.add_argument("--datum", default=os.environ.get("LAKE_DATUM", ""),
                    help="lake mode: the map origin 'lat,lon' = the nav datum = panel_feed's origin "
                         "(default: $LAKE_DATUM)")
    ap.add_argument("--bind", default="0.0.0.0", help="the HTTP server's address")
    ap.add_argument("--dry-run", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--dry-run-mission-s", type=float, default=20.0, help=argparse.SUPPRESS)
    a = ap.parse_args()

    if a.lake:
        from crusader_sim.lake_panel import LakePanel, parse_datum
        try:
            a.origin = parse_datum(a.datum)
        except ValueError as e:
            ap.error("--lake needs --datum lat,lon (or $LAKE_DATUM): %s" % e)
        panel = LakePanel(a)
    else:
        panel = Panel(a)
    srv = ThreadingHTTPServer((a.bind, a.port), make_handler(panel))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=panel.loop, daemon=True).start()
    if panel.sensors is not None:
        panel.sensors.start()
    panel.feed.start()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: panel.quit.set())
    print("task1_panel%s on http://%s:%d%s" % (
        " LAKE MODE" if a.lake else "", "<this host>" if a.lake else "localhost", a.port,
        " (DRY RUN)" if a.dry_run else ""), flush=True)
    while not panel.quit.wait(0.5):
        pass
    print("task1_panel: stopping (%s)" % ("the radio goes silent; a running goal is cancelled" if a.lake
                                          else "the sim keeps running"), flush=True)
    panel.shutdown()
    srv.shutdown()


if __name__ == "__main__":
    main()
