"""lake_panel — LAKE MODE of the Task 1 panel: the REAL boat, and a person playing Ekko.

    python3 -u -m crusader_sim.task1_panel --lake --datum LAT,LON --port 8095

Runs in: the `asv` container on the Jetson (ROS 2 sourced, PYTHONPATH on the checkout's
crusader_sim). lake_rig_up.sh starts it with everything else; LAKE_MODE.md is the procedure.
The laptop only runs a browser at http://<jetson>:8095 and PULLS everything over HTTP: no UDP
reaches it (WSL NAT, firewall = a security setting), and nothing is pushed.

WHAT IT IS. task1_panel in the sim lays out a course, launches Gazebo and answers the boat's
checkpoints as the UAV. At the lake there is no simulator: the buoys are real, the field has to
be BUILT from what the boat sees and where it is, and the boat is the real one. Everything about
the RXL handshake is the sim panel's (Panel, inherited): whole-field plans, 5 s resends, SEND /
ACK / SEND+ACK / auto-ACK, checkpoint labels, the UAV position-error setting (default 0 here).
This file adds only what the water changes:

  field          clicked camera tracks, "PIN AT BOAT" (the boat's fresh pose averaged over the last ~2 s, with an
                 optional "bow offset": that many metres ahead along the heading, to nose up to a buoy),
                 typed lat/lon, a loaded course; each buoy is
                 ENTRY / EXIT / RED / GREEN / BLACK, at most 10 (RXL). COMMIT FIELD puts it on the
                 air (this is what LAUNCH SIM is in the sim); after that positions are locked and
                 colours change through SEND, exactly as in the sim.
  boat           pose, trail, FCU mode + armed: the feed's pose/fcu layers (panel_feed.py), not gz.
  dead-man       the field is resent ONLY while a browser has polled /api/state in the last 3 s.
                 If the WiFi drops the resends stop, and the boat's own plan-freshness guard
                 (plan_timeout_s 15) aborts the mission ~15 s later. The same gate holds back
                 auto-ACK: no operator, no UAV.
  START          lake_goal (a child process; the tier you chose, Advanced or Disruptive (default), the
                 approach point you clicked, 600 s). Enabled only while the feed says ARMED + GUIDED, which is the PILOT's doing: this panel
                 cannot arm, disarm or change the mode, and lake_goal refuses to start unless the
                 autopilot says both anyway.
  ABORT          SIGINT to lake_goal = CANCEL the goal, then a banner: the boat is still armed and
                 may be moving under its last command; the pilot takes it with the RC (SC to HOLD).
  STOP UAV       closes the radio: all transmission stops. A running goal then aborts on the boat's
                 own plan-freshness guard.
  SAVE AS COURSE the field as a crusader_sim course YAML (origin = the datum, metres east/north of
                 it, the same schema as courses/*.yaml) so the same layout can be replayed in
                 Gazebo; loading one (template / load) is the reverse.

SAFETY BOUNDARY, kept by construction and by test/test_lake.py (which reads this file's code):
this process never arms, disarms, sets a mode, publishes /crsd/set_mode or /crsd/rc_override,
opens a MAVLink port (14550 / 14551 / 14552) or stops a protected node, and the page does not
present itself as an e-stop. The RC's SB switch is the only e-stop. Only the actions in ACTIONS are
routes; the sim's launch / attach / stop_sim (which would run gz_sim_up.sh) are not.

LOCKS. As Panel's: tx_lock -> link -> logs, tx_lock -> self.lock; the feed's own lock is a leaf.
The dead-man timestamp is a plain float (an atomic read), never taken under a lock.
"""
import json
import math
import os
import signal
import sys
import threading
import time

from crusader_sim import course as C
from crusader_sim.panel_feed import latlon_to_enu
from crusader_sim.paths import courses_dir
from crusader_sim.task1_panel import (
    LABEL, MAX_BUOYS, MIN_SPACING_M, NAME_RE, PANEL_COURSE, RESEND_S, STUB_MISSION, UAV_DEFAULT, Panel,
    Proc, check_layout, course_of, course_yaml, field_gates, layout_of, write_atomic, _names, _num, _safe)

DEADMAN_S = 3.0                 # a browser must have polled /api/state this recently for the field to be resent
MAX_RADIUS_M = 500.0            # the same bound act_layout puts on a position
START_TIMEOUT_S = 600.0         # lake_goal --timeout-s
ABORT_KILL_S = 25.0             # lake_goal needs <= 15 s to confirm a cancel; past this it is hung
TRAIL_STEP_M = 0.2
TRAIL_MAX = 5000
TRAIL_PERIOD_S = 0.25
ORIGIN_TOL_DEG = 1e-6           # ~0.1 m: a datum that differs by more is another place
DATUM_WARN_M = 0.5
POSE_WINDOW_S = 2.0             # PIN AT BOAT averages the fresh pose samples of this long
SAMPLE_GAP_S = 0.03             # two samples closer than this in message time are the same /crsd/pose message
PIN_MIN_SAMPLES = 4             # fewer fresh samples than this in the window: refuse (the feed sends ~4 a second)
PIN_WARN_SPREAD_M = 0.5         # the boat moved this far from its mean pose in the window: warn
PIN_MAX_SPREAD_M = 1.5          # ... this far: refuse, the average is not where the boat is
HEADING_MAX_AGE_S = 1.0         # the newest sample that HAD a heading must be this fresh for a bow offset
PIN_MIN_HEADING_SAMPLES = 3
MAX_BOW_OFFSET_M = 10.0
GUIDED = "GUIDED"
LAKE_TIERS = ("advanced", "disruptive")     # goal_client.TIERS minus core: with no UAV there is nothing to play
DEFAULT_TIER = "disruptive"
ACTIONS = ("layout", "template", "load", "clear", "save", "pin", "add_latlon", "approach", "commit",
           "stop_uav", "stage", "discard", "send", "ack", "send_ack", "auto_ack", "uav_error", "reroll",
           "start", "abort", "clear_trail")
AUTOSAVE = "autosave"          # layouts/autosave.yaml: the field in progress, kept across a panel restart


def parse_datum(text):
    """'lat,lon' -> {"lat", "lon"}. ValueError for a blank, a non-number, a point off the globe,
    or (0, 0): the default of an unset datum is not a place."""
    parts = str(text).split(",")
    if len(parts) != 2:
        raise ValueError("expected 'lat,lon', got %r" % (text,))
    lat, lon = float(parts[0]), float(parts[1])
    if not (math.isfinite(lat) and math.isfinite(lon) and abs(lat) <= 90 and abs(lon) <= 180):
        raise ValueError("%r is not a latitude/longitude" % (text,))
    if (lat, lon) == (0.0, 0.0):
        raise ValueError("0,0 is not a datum")
    return {"lat": lat, "lon": lon}


def fresh_data(view):
    """A feed layer view's data when the feed calls it fresh, else None (a blank, never the last value)."""
    return view["data"] if view["status"] == "fresh" and view["data"] else None


def round_or_none(v, ndigits):
    return None if v is None else round(v, ndigits)


class PoseAverager:
    """The boat's pose over the last `window_s`, for PIN AT BOAT: one sample per NEW feed packet whose pose layer
    is fresh (blanks over guesses: a stale pose adds nothing, and a stale feed refuses the pin). `note()` is fed
    by the panel's loop and by every pin / state call; it is idempotent per packet. A leaf lock: nothing is called
    while it is held. The clock is wall time, like the layer's age it subtracts."""

    def __init__(self, window_s=POSE_WINDOW_S, clock=time.time):
        self.window_s, self.clock = window_s, clock
        self.samples = []                       # [(message time, x, y, yaw or None)], oldest first
        self._packets = None
        self._lock = threading.Lock()

    def note(self, feed):
        """Offer the feed's current view (FeedReceiver.view()); keeps a sample when it is a new fresh pose."""
        v = feed["pose"]
        d = fresh_data(v)
        if d is None:
            return
        with self._lock:
            if feed["packets"] == self._packets:
                return
            self._packets = feed["packets"]
            now = self.clock()
            t = now - (v["age"] or 0.0)
            if self.samples and t - self.samples[-1][0] < SAMPLE_GAP_S:
                return
            self.samples.append((t, d["x"], d["y"], d["yaw"]))
            keep = now - 5.0 * self.window_s
            self.samples = [s for s in self.samples if s[0] >= keep]

    def average(self, feed):
        """-> (summary, None), or (None, why a pin cannot be taken now). The summary: x, y (the mean), n, span_s,
        spread_m (the farthest sample from the mean), yaw (ENU radians, the circular mean; None without a heading),
        heading_deg (compass), heading_n, heading_age (s since the newest sample that had one), heading_spread_deg."""
        self.note(feed)
        if fresh_data(feed["pose"]) is None:
            return None, "no fresh boat pose from the feed (%s): nothing to pin" % feed["pose"]["status"]
        with self._lock:
            now = self.clock()
            win = [s for s in self.samples if s[0] >= now - self.window_s]
        if len(win) < PIN_MIN_SAMPLES:
            return None, "only %d fresh pose sample(s) in the last %.0f s (need %d): the feed is dropping out or has just started" % (
                len(win), self.window_s, PIN_MIN_SAMPLES)
        mx, my = sum(s[1] for s in win) / len(win), sum(s[2] for s in win) / len(win)
        out = {"x": mx, "y": my, "n": len(win), "span_s": win[-1][0] - win[0][0],
               "spread_m": max(math.hypot(s[1] - mx, s[2] - my) for s in win),
               "yaw": None, "heading_deg": None, "heading_n": 0, "heading_age": None, "heading_spread_deg": None}
        hs = [s for s in win if s[3] is not None]
        if hs:
            sx, sy = sum(math.cos(s[3]) for s in hs), sum(math.sin(s[3]) for s in hs)
            yaw = math.atan2(sy, sx)
            dev = max(abs(math.degrees(math.atan2(math.sin(s[3] - yaw), math.cos(s[3] - yaw)))) for s in hs)
            out.update(yaw=yaw, heading_deg=(90.0 - math.degrees(yaw)) % 360.0, heading_n=len(hs),
                       heading_age=now - hs[-1][0], heading_spread_deg=dev)
        return out, None


def pin_problem(avg):
    """Why a pin must not be taken from this summary (a bow offset of 0), or None."""
    if avg["spread_m"] > PIN_MAX_SPREAD_M:
        return ("the boat moved %.1f m during the last %.0f s: hold it still alongside the buoy, then pin"
                % (avg["spread_m"], POSE_WINDOW_S))
    return None


def heading_problem(avg):
    """Why the bow offset must be refused (the heading is blank or stale), or None. The heading comes from GPS
    yaw (the compass is disabled): it is null until the RTK moving baseline fixes."""
    if avg["yaw"] is None:
        return "no heading: the GPS yaw is not valid yet (RTK moving baseline), so the bow offset cannot be placed; pin with offset 0"
    if avg["heading_age"] > HEADING_MAX_AGE_S or avg["heading_n"] < PIN_MIN_HEADING_SAMPLES:
        return "the heading is stale (%.1f s old, %d sample(s)): the bow offset needs a heading younger than %.0f s; pin with offset 0 or wait" % (
            avg["heading_age"], avg["heading_n"], HEADING_MAX_AGE_S)
    return None


def pin_place(avg, offset_m):
    """(x, y) of a pin: the averaged position, `offset_m` ahead along the averaged heading (ENU: 0 rad = east)."""
    if not offset_m:
        return avg["x"], avg["y"]
    return avg["x"] + offset_m * math.cos(avg["yaw"]), avg["y"] + offset_m * math.sin(avg["yaw"])


def read_rig(path):
    """What lake_rig_up.sh chose (rig.json: tree, nav_mode and why, POOL, the overlay's keys), for the page.
    None when no file was given; {"error": why} when it was and cannot be read: the page says so rather than
    showing a rig it does not know."""
    if not path:
        return None
    try:
        with open(path, encoding="utf-8") as f:
            rig = json.load(f)
    except (OSError, ValueError) as e:
        return {"error": "rig file %s unreadable: %s" % (path, e)}
    return rig if isinstance(rig, dict) else {"error": "rig file %s is not a JSON object" % path}


def start_problem(fcu, committed, radio_up, running, operator_here):
    """Why START must be refused, or None. `fcu` is the feed's fcu layer view ({"status", "data"}).
    One function for the page's button and for act_start, so they cannot disagree."""
    if not committed or not radio_up:
        return "COMMIT the field first: the boat needs a field on the air"
    if running:
        return "a goal is already running"
    if not operator_here:
        return "no browser is polling: the dead-man is holding the radio silent"
    if fcu is None or fcu.get("status") != "fresh" or not fcu.get("data"):
        return "no fresh FCU status from the feed: is panel_feed up, is telemetry_bridge publishing?"
    d = fcu["data"]
    if not d.get("armed"):
        return "the autopilot is NOT ARMED: the pilot arms it with the RC"
    if str(d.get("mode")).upper() != GUIDED:
        return "the autopilot is in %s, not GUIDED: the pilot selects GUIDED with the RC" % d.get("mode")
    return None


class LakePanel(Panel):
    PAGE = "lake_panel.html"
    ACTIONS = ACTIONS

    def __init__(self, a):
        self.last_poll = time.time()
        self.prev_gap = 0.0
        self.withheld, self._withheld_t = 0, 0.0
        self.last_trip = None
        self._tripped = False
        self.approach = None
        self.aborted_at = None
        self.run_tier = None                    # the tier START chose for the current / last run
        self.pose_avg = PoseAverager()
        self._trail_t = 0.0
        super().__init__(a)
        self.rig = read_rig(getattr(a, "rig_file", ""))
        self.uav = dict(UAV_DEFAULT, radius_m=0.0)       # the lake UAV is exact until the operator says otherwise
        self.logs["radio"].add("lake mode: datum %.7f, %.7f; endpoint %s; dead-man %.0f s" % (
            self.origin["lat"], self.origin["lon"], a.rxl_endpoint, DEADMAN_S))

    # ---------------------------------------------------------- the pieces Panel leaves to a subclass
    def _panel_dir(self):
        d = getattr(self.a, "lake_dir", None) or os.path.expanduser("~/.cache/crusader_lake")
        os.makedirs(d, exist_ok=True)
        return d

    def _init_gz(self):
        pass                                    # no gazebo: the pose comes from the feed

    def _init_sensors(self):
        pass                                    # the sensor views are gz cameras; the lake's are the OAK-D's own tools

    def _init_layout(self):
        """The field in progress from the last run of this panel, if it was for THIS datum."""
        path = os.path.join(self.dir, "layouts", AUTOSAVE + ".yaml")
        if not os.path.isfile(path):
            return
        try:
            c = C.load(path)
            if not self._is_datum(c["origin"]):
                return                          # another place: its metres mean nothing here
            buoys, note = layout_of(c)
            Panel._set_layout(self, buoys, "restored the field in progress" + ("; " + note if note else ""))
            self._approach_of(c)
        except Exception as e:                  # noqa: BLE001 -- a bad autosave must not stop the panel
            self.errors["autosave"] = "could not restore the field in progress: %s" % e

    def _set_layout(self, buoys, note=""):
        super()._set_layout(buoys, note)
        self._autosave()

    def _autosave(self):
        """The field in progress, so a panel restart does not lose a field typed in by hand."""
        try:
            write_atomic(os.path.join(self.dir, "layouts", AUTOSAVE + ".yaml"),
                         course_yaml(course_of(AUTOSAVE, self.layout, origin=self.origin,
                                               approach=self.approach)))
        except OSError:
            pass                                # a convenience, never a reason to fail

    def _is_datum(self, o):
        """Is {"lat", "lon"} `o` this panel's datum (to ~0.1 m)?"""
        return (abs(o["lat"] - self.origin["lat"]) <= ORIGIN_TOL_DEG
                and abs(o["lon"] - self.origin["lon"]) <= ORIGIN_TOL_DEG)

    def _operator_present(self):
        return time.time() - self.last_poll <= DEADMAN_S

    def _logs_since(self, q):
        """Only the two buffers the lake page has tabs for (the sim's `sim` and `judge` are empty here)."""
        return {k: v for k, v in super()._logs_since(q).items() if k in ("radio", "mission")}

    def _gates(self):
        """n_gates from the boat's own passage report when it has one (one per PAIRED gate, which
        is what the checkpoints count), else min(#red, #green) of the field that was sent."""
        pas = self.feed.view()["passage"]
        n = pas["data"].get("n_gates") if pas["status"] == "fresh" and pas["data"] else None
        return n if isinstance(n, int) and not isinstance(n, bool) else field_gates(self.sent)

    def _script(self, *_a):
        raise RuntimeError("lake mode never runs the sim's scripts")

    def _mission_argv(self):
        if self.a.dry_run:
            return [sys.executable, "-u", "-c", STUB_MISSION, str(self.a.dry_run_mission_s)]
        return [sys.executable, "-u", "-m", "crusader_sim.lake_goal", "--approach", self._approach_arg(),
                "--timeout-s", "%g" % START_TIMEOUT_S, "--tier", self.run_tier or DEFAULT_TIER]

    def _approach_arg(self):
        with self.lock:
            ap = self.approach
        if ap is None:
            return "none"
        lat, lon = C.enu_to_latlon(ap["x"], ap["y"], self.origin)
        return "%.7f,%.7f" % (lat, lon)

    def _approach_of(self, course):
        ap = course.get("approach")
        self.approach = ({"x": float(ap["x"]), "y": float(ap["y"])}
                         if isinstance(ap, dict) and _num(ap.get("x")) and _num(ap.get("y")) else None)

    def _load_file(self, path, what):
        r = super()._load_file(path, what)
        if r.get("ok"):
            try:
                with self.lock:
                    self._approach_of(C.load(path))
                    self._autosave()
            except Exception:                   # noqa: BLE001 -- the buoys loaded; the approach is optional
                pass
        return r

    # ---------------------------------------------------------- the feed
    def _boat(self, feed):
        """(x, y, yaw or None, age) of the boat, from the feed's pose layer, or None."""
        p = fresh_data(feed["pose"])
        return None if p is None else (p["x"], p["y"], p["yaw"], feed["pose"]["age"])

    def _origin_problem(self, feed):
        """Why the field cannot be placed (the feed and this panel use different origins), or None.
        Both convert metres <-> lat/lon around their own origin, so they must be the same place."""
        o = feed["origin"]
        if feed["up"] and o and not self._is_datum(o):
            return ("panel_feed's origin (%.7f, %.7f) is not this panel's datum (%.7f, %.7f): every "
                    "boat layer is in a different frame. Restart the rig with ONE LAKE_DATUM" % (
                        o["lat"], o["lon"], self.origin["lat"], self.origin["lon"]))
        return None

    def _datum_note(self, feed):
        """A warning when the nav datum the boat actually uses is not this panel's origin. The feed
        converts the boat's layers correctly either way, so this is a heads-up, not an error."""
        d = feed["datum"]
        if not d:
            return None
        dn, de = latlon_to_enu(d["lat"], d["lon"], self.origin)[::-1]
        if math.hypot(dn, de) > DATUM_WARN_M:
            return ("the boat's nav datum is %.1f m from this panel's datum: the map's metres are "
                    "right, but bt_runner was started with another LAKE_DATUM" % math.hypot(dn, de))
        return None

    def _lake_tick(self):
        """Worker thread, every poll: the dead-man's transitions into the Radio log, and the
        boat's trail from the feed's pose."""
        now = time.time()
        gap = now - self.last_poll
        if gap > DEADMAN_S and not self._tripped:
            self._tripped = True
            self.last_trip = {"at": now, "gap_s": gap}
            self.logs["radio"].add("DEAD-MAN TRIPPED: no browser poll for %.0f s; the field is no longer "
                                   "resent (the boat aborts ~15 s after its last one)" % DEADMAN_S)
        elif gap <= DEADMAN_S and self._tripped:
            self._tripped = False
            self.logs["radio"].add("dead-man released: a browser is polling again; resends continue "
                                   "(%d withheld)" % self.withheld)
        if self._tripped and now - self._withheld_t >= RESEND_S:
            self._withheld_t = now              # one per resend period the field was NOT sent
            self.withheld += 1
        feed = self.feed.view()
        self.pose_avg.note(feed)
        if now - self._trail_t < TRAIL_PERIOD_S:
            return
        self._trail_t = now
        boat = self._boat(feed)
        if boat is None:
            return
        with self.lock:
            if not self.trail or math.hypot(boat[0] - self.trail[-1][0], boat[1] - self.trail[-1][1]) >= TRAIL_STEP_M:
                self.trail.append((round(boat[0], 2), round(boat[1], 2)))
                if len(self.trail) > TRAIL_MAX:
                    self.trail, self.trail_gen = self.trail[-TRAIL_MAX // 2:], self.trail_gen + 1

    def _radio_tick(self):
        _safe(self._lake_tick)
        super()._radio_tick()

    # ---------------------------------------------------------- the field
    def _add_buoy(self, x, y, state, what):
        """Append a buoy at (x, y) metres from the datum."""
        if state not in LABEL:
            return {"ok": False, "error": "bad role %r" % (state,)}
        if not (_num(x) and _num(y) and abs(x) <= MAX_RADIUS_M and abs(y) <= MAX_RADIUS_M):
            return {"ok": False, "error": "position out of range (%g m max from the datum)" % MAX_RADIUS_M}
        x, y = round(float(x), 2), round(float(y), 2)
        with self.lock:
            if not self._editable():
                return {"ok": False, "error": "the field is committed: STOP UAV to edit its positions"}
            if len(self.layout) >= MAX_BUOYS:
                return {"ok": False, "error": "%d buoys is the limit (RXL carries 10)" % MAX_BUOYS}
            for i, b in enumerate(self.layout):
                if math.hypot(b["x"] - x, b["y"] - y) < MIN_SPACING_M:
                    return {"ok": False, "error": "b%d is already within %.1f m of there: move or "
                            "recolour it instead" % (i, MIN_SPACING_M)}
            self._set_layout(self.layout + [{"x": x, "y": y, "state": state}], "added %s" % what)
            return {"ok": True, "id": len(self.layout) - 1}

    def act_pin(self, body):
        """PIN AT BOAT {state, offset_m}: a buoy where the boat is, the feed's fresh pose averaged over the last
        ~2 s; `offset_m` (default 0, at most 10) moves it that far AHEAD along the heading, so the operator can
        nose the bow up to a buoy. Refused, never guessed: no fresh pose, too few samples, a boat that moved more
        than PIN_MAX_SPREAD_M in the window, and for an offset a heading that is blank or stale."""
        off = body.get("offset_m")
        off = 0.0 if off is None else off
        if not (_num(off) and 0.0 <= off <= MAX_BOW_OFFSET_M):
            return {"ok": False, "error": "bow offset: a number of metres from 0 to %g" % MAX_BOW_OFFSET_M}
        avg, why = self.pose_avg.average(self.feed.view())
        why = why or pin_problem(avg) or (heading_problem(avg) if off > 0 else None)
        if why:
            return {"ok": False, "error": why}
        x, y = pin_place(avg, float(off))
        what = "at the boat (%d samples over %.1f s, spread %.2f m%s)" % (
            avg["n"], avg["span_s"], avg["spread_m"],
            ", %.1f m ahead on heading %03.0f deg, %.1f s old" % (off, avg["heading_deg"], avg["heading_age"]) if off > 0 else "")
        r = self._add_buoy(x, y, body.get("state", "off"), what)
        if r["ok"]:
            self.logs["radio"].add("[panel] pin b%d %s" % (r["id"], what))
            r["pin"] = self._pin_view(avg, None)
            r["pin"]["offset_m"] = float(off)
        return r

    def _pin_view(self, avg, why):
        """The `pin` block of /api/state (and of a pin's reply): what a pin taken now would be made of, and why not."""
        if avg is None:
            return {"problem": why, "n": 0, "window_s": POSE_WINDOW_S, "heading_problem": why}
        warn = (None if avg["spread_m"] <= PIN_WARN_SPREAD_M else
                "the boat moved %.2f m in the window: the pin is the AVERAGE, not where the bow is now" % avg["spread_m"])
        return {"problem": pin_problem(avg), "warn": warn, "n": avg["n"], "window_s": POSE_WINDOW_S,
                "span_s": round(avg["span_s"], 2), "spread_m": round(avg["spread_m"], 3),
                "heading_deg": round_or_none(avg["heading_deg"], 1), "heading_age": round_or_none(avg["heading_age"], 2),
                "heading_spread_deg": round_or_none(avg["heading_spread_deg"], 1),
                "heading_problem": heading_problem(avg), "max_offset_m": MAX_BOW_OFFSET_M}

    def act_add_latlon(self, body):
        lat, lon = body.get("lat"), body.get("lon")
        if not (_num(lat) and _num(lon) and abs(lat) <= 90 and abs(lon) <= 180):
            return {"ok": False, "error": "lat and lon must be numbers"}
        x, y = latlon_to_enu(float(lat), float(lon), self.origin)
        return self._add_buoy(x, y, body.get("state", "off"), "from %.7f, %.7f" % (lat, lon))

    def act_approach(self, body):
        """The point lake_goal drives to before it looks (a point you clicked). {clear: true}
        drops it (the goal then skips the drive); {at_boat: true} takes the boat's position."""
        if body.get("clear"):
            ap = None
        elif body.get("at_boat"):
            boat = self._boat(self.feed.view())
            if boat is None:
                return {"ok": False, "error": "no fresh boat pose from the feed"}
            ap = {"x": round(boat[0], 2), "y": round(boat[1], 2)}
        else:
            x, y = body.get("x"), body.get("y")
            if not (_num(x) and _num(y) and abs(x) <= MAX_RADIUS_M and abs(y) <= MAX_RADIUS_M):
                return {"ok": False, "error": "x and y: metres from the datum, within %g" % MAX_RADIUS_M}
            ap = {"x": round(float(x), 2), "y": round(float(y), 2)}
        with self.lock:
            if self.mission["running"]:
                return {"ok": False, "error": "a goal is running; its approach point is already sent"}
            self.approach = ap
            self._autosave()
        self.logs["mission"].add("[panel] approach point: %s" % (
            "none (the goal skips the drive)" if ap is None else "%.2f, %.2f m from the datum" % (ap["x"], ap["y"])))
        return {"ok": True}

    def act_clear_trail(self, _body):
        with self.lock:
            self.trail, self.trail_gen = [], self.trail_gen + 1
        return {"ok": True}

    def act_stage(self, body):
        if self.sim != "up":
            return {"ok": False, "error": "COMMIT the field first: before that, edit the field directly"}
        return super().act_stage(body)

    # ---------------------------------------------------------- COMMIT FIELD / STOP UAV
    def act_commit(self, _body):
        """COMMIT FIELD: put the field on the air. The lake's LAUNCH SIM."""
        feed = self.feed.view()
        with self.lock:
            if self.sim == "up":
                return {"ok": False, "error": "the field is already committed (STOP UAV to change it)"}
            errs, _ = check_layout(self.layout, start_clear_m=0.0)
            why = "; ".join(errs) or self._origin_problem(feed)
            if why:
                return {"ok": False, "error": why}
            self.launched = course_of(PANEL_COURSE, self.layout, origin=self.origin)
            self.sim, self.sim_step, self.sim_up_t = "up", "committed", time.time()
        self._radio_up()
        with self.lock:
            if self.link is None:
                self.sim, self.sim_step = "down", ""
                return {"ok": False, "error": self.errors.get("radio", "the radio did not open")}
        return {"ok": True}

    def act_stop_uav(self, _body):
        """STOP UAV: all transmission stops (the radio closes). The field you were running comes
        back as the one you edit, in the colours last SENT."""
        with self.lock:
            if self.sim != "up":
                return {"ok": False, "error": "nothing is being transmitted"}
            self.sim, self.sim_step = "down", ""
            self._set_layout([{"x": b["x"], "y": b["y"], "state": s}
                              for b, s in zip(self.layout, self.sent)], "the field as last sent")
            for rec in self._open():
                self._close(rec, "uav stopped")
        self._radio_down()
        self.logs["radio"].add("STOP UAV: the radio is closed, nothing is transmitted")
        return {"ok": True}

    # ---------------------------------------------------------- START / ABORT
    def _on_run_begin(self):
        self.aborted_at = None                  # a new run clears the last ABORT's banner; there is no referee here

    def _start_problem(self, feed):
        with self.lock:
            return start_problem(feed["fcu"], self.sim == "up", self.link is not None,
                                 self.mission["running"], self._operator_present())

    def act_start(self, body):
        """START {"tier": "advanced" | "disruptive"} (default disruptive): the goal's tier. Advanced plans once
        and asks no checkpoints; Disruptive asks at the entry orbit and every gate (task1_global.xml picks its
        subtree by it; the field you send is the same)."""
        tier = (body or {}).get("tier")
        tier = DEFAULT_TIER if tier is None else tier
        if tier not in LAKE_TIERS:
            return {"ok": False, "error": "tier must be advanced or disruptive"}
        if tier == "advanced" and isinstance(self.rig, dict) and self.rig.get("global_tree") is False:
            return {"ok": False, "error": "this rig runs the per-gate tree %s, which has no Advanced behaviour (it "
                    "always asks checkpoints): START Disruptive, or restart the rig with TREE=task1_global.xml"
                    % self.rig.get("tree")}
        feed = self.feed.view()
        why = self._start_problem(feed)
        if why:
            return {"ok": False, "error": why}
        with self.lock:
            self.run_tier = tier
        states = self._begin_run()
        self._transmit(states, "START")          # the field must be there before the goal
        self.logs["mission"].add("--- START: lake_goal, %s tier (the pilot armed it and chose GUIDED) ---" % tier)
        try:
            self.proc_mission = Proc("lake_goal", self._mission_argv(), self.logs["mission"],
                                     self._on_mission_line, self._on_mission_exit).start()
        except OSError as e:
            with self.lock:
                self.mission["running"] = False
            return {"ok": False, "error": "could not start lake_goal: %s" % e}
        return {"ok": True}

    def _stop_mission(self):
        """SIGINT to lake_goal: it CANCELS the goal and waits for the tree to finish it. A
        lake_goal that has not exited by ABORT_KILL_S is hung and is killed."""
        p = self.proc_mission
        if p is None or not p.running():
            return
        p.signal(signal.SIGINT)
        t = threading.Timer(ABORT_KILL_S, lambda: p.running() and p.signal(signal.SIGKILL))
        t.daemon = True
        t.start()

    def act_abort(self, _body):
        with self.lock:
            running = self.mission["running"]
            self.aborted_at = time.time()
        self._stop_mission()
        self.logs["mission"].add("[panel] ABORT: lake_goal told to CANCEL the goal%s. The boat is still ARMED: "
                                 "switch SC to HOLD / use the RC." % ("" if running else " (none was running)"))
        return {"ok": True}

    # ---------------------------------------------------------- courses
    def _course_text(self, name, feed):
        """The committed field (or, before COMMIT, the one being built) as a course YAML."""
        boat = self._boat(feed)
        start = None if boat is None else {"x": round(boat[0], 2), "y": round(boat[1], 2),
                                           "yaw_deg": round(math.degrees(boat[2] or 0.0), 1)}
        with self.lock:
            if self.sim == "up" and self.launched is not None:
                buoys = [{"x": e["x"], "y": e["y"], "state": s}
                         for e, s in zip(self.launched["elements"], self.sent)]
            else:
                buoys = list(self.layout)
            ap = self.approach
        return course_yaml(course_of(name, buoys, origin=self.origin, boat_start=start, approach=ap))

    def act_save(self, body):
        """SAVE AS COURSE: <dir>/layouts/<name>.yaml, in crusader_sim's course schema; the page
        offers it as a download (GET /api/course/<name>.yaml)."""
        name = body.get("name")
        if not (isinstance(name, str) and NAME_RE.match(name)):
            return {"ok": False, "error": "name: letters, digits, - and _ (max 40)"}
        text = self._course_text(name, self.feed.view())
        path = os.path.join(self.dir, "layouts", name + ".yaml")
        write_atomic(path, text)
        self.logs["radio"].add("saved course %s (%d buoys): %s" % (name, text.count("type: robobuoy"), path))
        return {"ok": True, "path": path, "download": "/api/course/%s.yaml" % name}

    def download(self, name):
        try:
            with open(os.path.join(self.dir, "layouts", name + ".yaml"), "rb") as f:
                return f.read(), name + ".yaml"
        except OSError:
            return None

    # ---------------------------------------------------------- state
    def state(self, q):
        now = time.time()
        self.prev_gap, self.last_poll = now - self.last_poll, now     # THE dead-man's heartbeat
        radio_up, pending = self._link_pending()
        logs = self._logs_since(q)
        feed = self.feed.view()                  # its own leaf lock, never under self.lock
        boat = self._boat(feed)
        pin = self._pin_view(*self.pose_avg.average(feed))
        problem = self._origin_problem(feed)
        with self.lock:
            errs, warns = check_layout(self.layout, start_clear_m=0.0)
            errors = dict(self.errors)
            if feed["error"]:
                errors["feed"] = feed["error"]
            if problem:
                errors["origin"] = problem
            note = self._datum_note(feed)
            if note:
                warns = warns + [note]
            o = self.origin
            buoys = []
            for b in self.layout:
                la, lo = C.enu_to_latlon(b["x"], b["y"], o)
                buoys.append(dict(b, lat=round(la, 7), lon=round(lo, 7)))
            return {
                "lake": True, "dry_run": self.a.dry_run, "errors": errors,
                "mode": "live" if self.sim == "up" else "build",
                "datum": dict(o), "approach": self._approach_view(),
                "layout": {"rev": self.layout_rev, "buoys": buoys, "note": self.layout_note,
                           "errors": errs + ([problem] if problem else []), "warnings": warns},
                "templates": _names(courses_dir()),
                "layouts": [n for n in _names(os.path.join(self.dir, "layouts")) if n != AUTOSAVE],
                "radio": self._radio_state(radio_up, pending),
                "deadman": {"ok": True, "window_s": DEADMAN_S, "gap_s": round(self.prev_gap, 2),
                            "withheld": self.withheld, "last_trip": self.last_trip},
                "fcu": feed["fcu"], "pin": pin, "boat": None if boat is None else {
                    "x": boat[0], "y": boat[1], "yaw": boat[2], "age": boat[3]},
                "start_block": start_problem(feed["fcu"], self.sim == "up", radio_up,
                                             self.mission["running"], True),
                "mission": dict(self.mission, aborted_at=self.aborted_at, tier=self.run_tier),
                "tiers": list(LAKE_TIERS), "rig": self.rig, "feed": feed,
                "trail": self._trail_since(q), "logs": logs,
                **self._field_state(self.sim == "up"),
            }

    def _approach_view(self):
        """The approach point with its lat/lon, or None. Call with self.lock held."""
        ap = self.approach
        if ap is None:
            return None
        la, lo = C.enu_to_latlon(ap["x"], ap["y"], self.origin)
        return dict(ap, lat=round(la, 7), lon=round(lo, 7))
