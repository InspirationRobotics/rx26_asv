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

  field          clicked camera tracks, "PIN AT BOAT", typed lat/lon, a loaded course; each buoy is
                 ENTRY / EXIT / RED / GREEN / BLACK, at most 10 (RXL). COMMIT FIELD puts it on the
                 air (this is what LAUNCH SIM is in the sim); after that positions are locked and
                 colours change through SEND, exactly as in the sim.
  boat           pose, trail, FCU mode + armed: the feed's pose/fcu layers (panel_feed.py), not gz.
  dead-man       the field is resent ONLY while a browser has polled /api/state in the last 3 s.
                 If the WiFi drops the resends stop, and the boat's own plan-freshness guard
                 (plan_timeout_s 15) aborts the mission ~15 s later. The same gate holds back
                 auto-ACK: no operator, no UAV.
  START          lake_goal (a child process; tier 2, the approach point you clicked, 600 s). Enabled
                 only while the feed says ARMED + GUIDED, which is the PILOT's doing: this panel
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
GUIDED = "GUIDED"
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
        self._trail_t = 0.0
        super().__init__(a)
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
            o = c["origin"]
            if (abs(o["lat"] - self.origin["lat"]) > ORIGIN_TOL_DEG
                    or abs(o["lon"] - self.origin["lon"]) > ORIGIN_TOL_DEG):
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
                "--timeout-s", "%g" % START_TIMEOUT_S]

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
    def _fresh(self, feed, layer):
        """The layer's data when the feed calls it fresh, else None (a blank, never the last value)."""
        v = feed[layer]
        return v["data"] if v["status"] == "fresh" else None

    def _boat(self, feed):
        """(x, y, yaw or None, age) of the boat, from the feed's pose layer, or None."""
        p = self._fresh(feed, "pose")
        return None if p is None else (p["x"], p["y"], p["yaw"], feed["pose"]["age"])

    def _origin_problem(self, feed):
        """Why the field cannot be placed (the feed and this panel use different origins), or None.
        Both convert metres <-> lat/lon around their own origin, so they must be the same place."""
        o = feed["origin"]
        if feed["up"] and o and (abs(o["lat"] - self.origin["lat"]) > ORIGIN_TOL_DEG
                                 or abs(o["lon"] - self.origin["lon"]) > ORIGIN_TOL_DEG):
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
        if now - self._trail_t < TRAIL_PERIOD_S:
            return
        self._trail_t = now
        boat = self._boat(self.feed.view())
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
        """PIN AT BOAT: a buoy where the boat is NOW (the feed's pose, fresh or nothing)."""
        boat = self._boat(self.feed.view())
        if boat is None:
            return {"ok": False, "error": "no fresh boat pose from the feed: nothing to pin"}
        return self._add_buoy(boat[0], boat[1], body.get("state", "off"), "at the boat")

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
    def _start_problem(self, feed):
        with self.lock:
            return start_problem(feed["fcu"], self.sim == "up", self.link is not None,
                                 self.mission["running"], self._operator_present())

    def act_start(self, _body):
        feed = self.feed.view()
        why = self._start_problem(feed)
        if why:
            return {"ok": False, "error": why}
        with self.tx_lock:
            self.link.rewind()
        with self.lock:
            self.checkpoints = []
            self.mission = self._fresh_mission()
            self.mission["running"] = True
            self.aborted_at = None
            self.trail, self.trail_gen = [], self.trail_gen + 1
            states = list(self.sent)
        self._transmit(states, "START")          # the field must be there before the goal
        self.logs["mission"].add("--- START: lake_goal (the pilot armed it and chose GUIDED) ---")
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
                "fcu": feed["fcu"], "boat": None if boat is None else {
                    "x": boat[0], "y": boat[1], "yaw": boat[2], "age": boat[3]},
                "start_block": start_problem(feed["fcu"], self.sim == "up", radio_up,
                                             self.mission["running"], True),
                "mission": dict(self.mission, aborted_at=self.aborted_at), "feed": feed,
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
