"""squirt_core — the squirter calibration session, with no ROS and no HTTP.

WHAT IT IS FOR. The Task 3 nozzle is fixed (about 45 deg, no pan or tilt), so the
aim is where the boat sits: its distance from the face sets the hit height, its
sideways position sets left/right. This session finds, per target, the range
(and sideways offset) at which the stream hits, from one person's verdicts:

    fire a short burst -> "the boat is too far FORWARD / BACK / LEFT / RIGHT / on"

VERDICTS ARE ABOUT THE BOAT, NOT THE WATER. "Too far forward" means the boat
should move back, whichever side of the arc the stream is on. That makes each
verdict a one-sided bound on the measured range, and the fore/aft estimate is a
bracket: "too far forward at 0.92 m" means the answer is above 0.92 m.

THE RANGE IS THE ONE MEASURED BEFORE THE BURST (median of the last
range_window_s): the spray itself returns LiDAR points while it is in the air.

Everything takes `now` (monotonic seconds) from the caller, so the fake boat and
the tests drive time. The App's methods are called from the HTTP threads, the
ROS executor and the tick thread; they all take self.lock.
"""
import hashlib
import json
import os
import statistics
import threading
import time
from collections import deque

from nozzle_model import NozzleModel, target_height
from steady_core import SteadyMonitor

# ------------------------------------------------------------------ config

# Every key the tool reads. crusader_params.yaml's `squirt_cal` section must name
# exactly these (load_config refuses unknown and missing keys): a typo that
# silently fell back to a default is how a tool ends up calibrated against a
# nozzle height nobody measured.
DEFAULTS = dict(
    port=8094,
    log_dir="/root/robotx_ws/logs/squirt_cal",
    snapshot_url="http://127.0.0.1:8080/stream/raw",
    wall_range_topic="/crsd/wall_range",   # = wall_range_node.wall_topic
    pump_rc_channel=10,          # = telemetry_bridge.pump_rc_channel (log-only fallback)
    pump_on_threshold_us=1700,
    burst_choices_s=[0.2, 0.3, 0.5],
    burst_default_s=0.3,
    arm_timeout_s=15.0,          # give up waiting for steady after this
    min_gap_s=2.0,               # between shots; the bridge has its own too
    verdict_after_s=0.8,         # after the burst ends: the water's flight + a look
    pilot_burst_max_s=3.0,       # a pilot squirt with no OFF edge ends here
    range_window_s=0.5,          # pre-fire range = median over this window
    range_max_age_s=0.5,         # an older range is shown as stale
    step_m=0.10,                 # search step until a bracket exists
    on_range_tol_m=0.02,         # "ON RANGE" within this
    min_hits_for_band=3,
    square_tol_deg=3.0,          # "square up" hint beyond this wall angle
    # steadiness (steady_core)
    steady_rate_max_dps=4.0,
    steady_band_deg=1.5,
    steady_hold_s=0.5,
    steady_mean_s=5.0,
    sticks_quiet_s=1.5,
    stick_tol_us=25,
    att_min_hz=10.0,
    # starting point only (nozzle_model); MEASURE these in Phase 0
    nozzle_x_m=0.45,             # nozzle ahead of the LiDAR's body origin
    nozzle_height_m=0.40,        # above the water
    nozzle_elev_deg=45.0,
    nozzle_range_m=3.0,          # level range at nozzle_elev_deg
    deck_height_m=0.30,          # face panel bottom above the water
    face_setback_m=0.0,          # face behind the edge the LiDAR ranges
    branch="near",               # near = on the way up; far = coming down
    # targets, as parallel lists (rcl params cannot be a list of dicts)
    target_ids=["ul_top", "lr_top"],
    target_labels=["Upper-left window, top edge", "Lower-right window, top edge"],
    target_edge_mm=[895.0, 645.0],   # above the face panel's bottom edge
    target_lat_m=[0.22, -0.23],      # window centre from the face centre, + = left
)


def load_config(section=None):
    """The tool's config: DEFAULTS, or a crusader_params.yaml section that must
    name exactly the same keys. Raises KeyError naming what is wrong."""
    if section is None:
        return dict(DEFAULTS)
    section = dict(section)
    unknown = sorted(set(section) - set(DEFAULTS))
    missing = sorted(set(DEFAULTS) - set(section))
    if unknown or missing:
        raise KeyError(f"squirt_cal config: unknown={unknown} missing={missing}")
    n = len(section["target_ids"])
    for k in ("target_labels", "target_edge_mm", "target_lat_m"):
        if len(section[k]) != n:
            raise KeyError(f"squirt_cal config: {k} has {len(section[k])} "
                           f"entries, target_ids has {n}")
    if section["burst_default_s"] not in section["burst_choices_s"]:
        raise KeyError("squirt_cal config: burst_default_s is not one of "
                       "burst_choices_s")
    return section


def config_hash(cfg):
    return hashlib.sha256(json.dumps(cfg, sort_keys=True).encode()).hexdigest()[:10]


# --------------------------------------------------------------- estimator

class AxisEstimator:
    """Bracket one measured coordinate from one-sided verdicts.

    Observations are (value, d): d = +1 "the right value is ABOVE this one",
    -1 "below", 0 "on". With consistent verdicts the answer is the midpoint of
    the tightest bracket; with contradictions (rocking, a misjudged shot) it is
    the threshold that the fewest verdicts disagree with. Hits, once there are
    any, outrank the bracket: they are direct evidence of where it works.
    """

    def __init__(self, obs=()):
        self.obs = list(obs)

    def add(self, value, d):
        self.obs.append((float(value), int(d)))

    def estimate(self, min_hits_for_band=3):
        """The answer so far. With hits, the aim is the MIDDLE of the band that
        hits, not the first spot that happened to: the first hit is usually at
        the band's near edge (that is where the search came from), and a boat
        parked on an edge misses half the time. Each band edge is placed halfway
        between the outermost hit and the nearest miss beyond it."""
        above = [v for v, d in self.obs if d > 0]
        below = [v for v, d in self.obs if d < 0]
        hits = sorted(v for v, d in self.obs if d == 0)
        lo = max(above) if above else None
        hi = min(below) if below else None
        consistent = lo is None or hi is None or lo < hi
        centre, how = None, "no data"
        edge_lo = edge_hi = None
        if hits:
            lo_near = max((v for v in above if v < hits[0]), default=None)
            hi_near = min((v for v in below if v > hits[-1]), default=None)
            edge_lo = None if lo_near is None else (lo_near + hits[0]) / 2.0
            edge_hi = None if hi_near is None else (hi_near + hits[-1]) / 2.0
            if edge_lo is not None and edge_hi is not None:
                centre, how = (edge_lo + edge_hi) / 2.0, "middle of the hit band"
            else:
                centre, how = statistics.median(hits), f"median of {len(hits)} hit(s)"
        elif lo is not None and hi is not None:
            if consistent:
                centre, how = (lo + hi) / 2.0, "bracket midpoint"
            else:
                centre, how = self._stump(), "fewest contradicting verdicts"
        band = None
        if len(hits) >= min_hits_for_band:
            band = (hits[0], hits[-1])
        return dict(centre=centre, how=how, lo=lo, hi=hi, consistent=consistent,
                    n=len(self.obs), n_hits=len(hits), band=band,
                    edge_lo=edge_lo, edge_hi=edge_hi,
                    hit_min=hits[0] if hits else None,
                    hit_max=hits[-1] if hits else None)

    def _stump(self):
        vals = sorted({v for v, _ in self.obs})
        cands = [vals[0] - 1e-3] + [(a + b) / 2 for a, b in zip(vals, vals[1:])] \
            + [vals[-1] + 1e-3]

        def errors(c):
            return (sum(1 for v, d in self.obs if d > 0 and v >= c)
                    + sum(1 for v, d in self.obs if d < 0 and v <= c)
                    + sum(1 for v, d in self.obs if d == 0 and abs(v - c) > 0.05))
        scored = [(errors(c), c) for c in cands]
        best = min(e for e, _ in scored)
        return statistics.median([c for e, c in scored if e == best])

    def next_value(self, step, min_hits_for_band=3):
        """Where to shoot next, or None with no data.

        After the first hit it probes half a step past the hits, first on the
        far side and then on the near side, until each side has a miss: two or
        three shots that turn "it hit here once" into "it hits between these",
        whose middle is the answer and whose width is the tolerance the tree
        has to hold."""
        e = self.estimate(min_hits_for_band)
        if e["n_hits"]:
            if e["edge_hi"] is None:
                return e["hit_max"] + step / 2.0
            if e["edge_lo"] is None:
                return e["hit_min"] - step / 2.0
            return e["centre"]
        if e["centre"] is not None:
            return e["centre"]
        if e["lo"] is not None:
            return e["lo"] + step
        if e["hi"] is not None:
            return e["hi"] - step
        return None


FA = {"fwd": +1, "ok": 0, "back": -1}      # boat too far forward -> range must grow
LAT = {"left": -1, "ok": 0, "right": +1}   # boat too far left -> offset must shrink


# ------------------------------------------------------------------ the log

class ShotLog:
    """One session = one directory: shots.jsonl (append-only) + snapshots.

    Retractions are appended as their own line rather than rewriting the file,
    so a crash mid-session never costs a shot that was already written."""

    def __init__(self, root, now_wall=None):
        self.root = root
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now_wall))
        self.name = stamp
        self.dir = os.path.join(root, stamp) if root else None
        self.error = ""
        if self.dir:
            try:
                os.makedirs(self.dir, exist_ok=True)
            except OSError as e:
                self.error = f"cannot create {self.dir}: {e}"
                self.dir = None

    def write(self, record):
        if not self.dir:
            return
        try:
            with open(os.path.join(self.dir, "shots.jsonl"), "a") as f:
                f.write(json.dumps(record) + "\n")
        except OSError as e:
            self.error = f"write failed: {e}"

    def path(self, name):
        return os.path.join(self.dir, name) if self.dir else None


# ------------------------------------------------------------------ the app

IDLE, ARMED, FIRING, VERDICT = "idle", "armed", "firing", "verdict"


class App:
    """The session: live inputs, the shot state machine, the estimates.

    The adapter supplies inputs by calling on_* and is asked to fire through
    adapter.can_fire() / adapter.fire(burst_s, seq). The HTTP layer calls
    snapshot() and action()."""

    def __init__(self, cfg, adapter, clock=time.monotonic, wall=time.time,
                 log_root=None):
        self.cfg = cfg
        self.adapter = adapter
        self.clock, self.wall = clock, wall
        self.lock = threading.RLock()
        self.steady = SteadyMonitor(
            rate_max_dps=cfg["steady_rate_max_dps"], band_deg=cfg["steady_band_deg"],
            hold_s=cfg["steady_hold_s"], mean_s=cfg["steady_mean_s"],
            sticks_quiet_s=cfg["sticks_quiet_s"], stick_tol_us=cfg["stick_tol_us"],
            min_hz=cfg["att_min_hz"])
        self.ranges = deque()        # (t, valid, range, angle, lat or None)
        self.pose = None             # (lat, lon, heading)
        self.fcu = None              # (mode, armed)
        self.mission_active = False
        self.tape_range = None
        self.targets = list(zip(cfg["target_ids"], cfg["target_labels"],
                                cfg["target_edge_mm"], cfg["target_lat_m"]))
        self.target = self.targets[0][0]
        self.burst_s = cfg["burst_default_s"]
        self.arm_mode = "steady"
        self.state = IDLE
        self.armed_at = None
        self.pending = None          # the shot being fired / awaiting a verdict
        self.shots = []
        self.last_fire_t = None
        self.message = "ready"
        self.events = deque(maxlen=30)
        self._event_seq = 0
        self._seq = 0
        self.cfg_hash = config_hash(cfg)
        self.log_root = log_root if log_root is not None else cfg["log_dir"]
        self.log = ShotLog(self.log_root, self.wall())
        try:
            self.model = NozzleModel.from_range(
                cfg["nozzle_range_m"], cfg["nozzle_elev_deg"], cfg["nozzle_height_m"])
        except ValueError:
            self.model = None

    # ------------------------------------------------ inputs (adapter calls)

    def on_att(self, t, roll, pitch, roll_rate, pitch_rate):
        with self.lock:
            self.steady.feed_att(t, roll, pitch, roll_rate, pitch_rate)

    def on_sticks(self, t, sticks):
        with self.lock:
            self.steady.feed_sticks(t, sticks)

    def on_range(self, t, valid, range_m=None, angle_deg=None, lat_m=None):
        with self.lock:
            self.ranges.append((t, bool(valid), range_m, angle_deg, lat_m))
            while self.ranges and self.ranges[0][0] < t - 10.0:
                self.ranges.popleft()

    def on_pose(self, lat, lon, heading):
        with self.lock:
            self.pose = (lat, lon, heading)

    def on_fcu(self, mode, armed):
        with self.lock:
            self.fcu = (mode, armed)

    def on_mission(self, active):
        with self.lock:
            self.mission_active = bool(active)

    def on_pump_edge(self, t, on):
        """The pump output (or the pilot's switch) changed. A rising edge that
        is not our own burst is the pilot firing: that is a shot too."""
        with self.lock:
            if on:
                if self.state == FIRING and self.pending and \
                        self.pending["source"] == "tool":
                    return                           # our own burst
                if self.state == VERDICT and self.pending:
                    self._finish(discarded=True, why="superseded by the next shot")
                if self.state == ARMED:
                    self.state = IDLE
                self._begin_shot(t, source="pilot", burst_s=None)
                self._event("fired")
            elif self.state == FIRING and self.pending and \
                    self.pending["source"] == "pilot" and \
                    self.pending["burst_s"] is None:
                self.pending["burst_s"] = round(t - self.pending["t_fire"], 3)

    # ------------------------------------------------ the tick

    def tick(self, now=None):
        now = self.clock() if now is None else now
        self.adapter.poll(now)
        with self.lock:
            if self.state == ARMED:
                self._tick_armed(now)
            elif self.state == FIRING:
                self._tick_firing(now)

    def _tick_armed(self, now):
        if now - self.armed_at > self.cfg["arm_timeout_s"]:
            why = ", ".join(self.steady.status(now)["reasons"]) or "?"
            self.state = IDLE
            self.message = f"never got steady in {self.cfg['arm_timeout_s']:.0f} s ({why})"
            self._event("refused")
            return
        if self.arm_mode == "now" or self.steady.status(now)["steady"]:
            self._fire(now)

    def _tick_firing(self, now):
        p = self.pending
        if p is None:
            self.state = IDLE
            return
        if p["source"] == "pilot" and p["burst_s"] is None:
            if now - p["t_fire"] < self.cfg["pilot_burst_max_s"]:
                return
            p["burst_s"] = self.cfg["pilot_burst_max_s"]
        if not p.get("snapshot_taken") and now - p["t_fire"] >= min(0.3, p["burst_s"]):
            p["snapshot_taken"] = True
            name = f"shot_{p['id']:03d}.jpg"
            path = self.log.path(name)
            if path and self.adapter.snapshot(path):
                p["snapshot"] = name
        if now >= p["t_fire"] + p["burst_s"] + self.cfg["verdict_after_s"]:
            pp = self.steady.peak_to_peak(p["t_fire"], p["t_fire"] + p["burst_s"] + 0.3)
            if pp:
                p["pp_roll_deg"], p["pp_pitch_deg"] = round(pp[0], 2), round(pp[1], 2)
            self.state = VERDICT
            self.message = "where should the boat go?"
            self._event("verdict")

    # ------------------------------------------------ firing

    def fire_path(self):
        """(ok, reason): can the TOOL fire right now? The pilot always can."""
        if self.mission_active:
            return False, "a mission is running"
        return self.adapter.can_fire()

    def _fire(self, now):
        ok, why = self.fire_path()
        if ok:
            self._seq += 1
            ok, why = self.adapter.fire(self.burst_s, self._seq)
        if not ok:
            self.state = IDLE
            self.message = f"not fired: {why}"
            self._event("refused")
            return
        self._begin_shot(now, source="tool", burst_s=self.burst_s)
        self.pending["fire_result"] = why
        self.message = "fired"
        self._event("fired")

    def _pre_fire(self, t):
        """Median range/angle/offset over the window BEFORE t."""
        w = self.cfg["range_window_s"]
        s = [r for r in self.ranges if t - w <= r[0] < t + 1e-9 and r[1]]
        if s:
            lats = [r[4] for r in s if r[4] is not None]
            return dict(range_m=round(statistics.median(r[2] for r in s), 4),
                        angle_deg=round(statistics.median(r[3] for r in s), 2),
                        lat_m=round(statistics.median(lats), 4) if lats else None,
                        range_n=len(s), range_src="lidar")
        if self.tape_range is not None:
            return dict(range_m=self.tape_range, angle_deg=None, lat_m=None,
                        range_n=0, range_src="tape")
        return dict(range_m=None, angle_deg=None, lat_m=None, range_n=0,
                    range_src="none")

    def _begin_shot(self, t, source, burst_s):
        st = self.steady.status(t)
        att = self.steady.latest()
        shot = dict(
            id=len(self.shots) + 1, session=self.log.name, target=self.target,
            source=source, mode=self.arm_mode if source == "tool" else "pilot",
            t_fire=t, t_wall=round(self.wall(), 3), burst_s=burst_s,
            roll_deg=round(att[1], 2) if att else None,
            pitch_deg=round(att[2], 2) if att else None,
            steady=st["steady"], steady_reasons=list(st["reasons"]),
            rate_max_dps=st["rate_max_dps"], att_hz=st["att_hz"],
            sticks_quiet_s=None if st["sticks_quiet_s"] is None
            else round(st["sticks_quiet_s"], 2),
            pose=self.pose, fcu=self.fcu, cfg=self.cfg_hash,
            pp_roll_deg=None, pp_pitch_deg=None, snapshot=None,
            fa=None, lat=None, discarded=False, why="")
        shot.update(self._pre_fire(t))
        self.pending = shot
        self.state = FIRING
        self.last_fire_t = t

    # ------------------------------------------------ verdicts

    def _finish(self, discarded=False, why="", fa=None, lat=None):
        p = self.pending
        p.update(discarded=discarded, why=why, fa=fa, lat=lat)
        p.pop("snapshot_taken", None)
        self.shots.append(p)
        self.log.write(p)
        self.pending = None
        self.state = IDLE

    def verdict(self, fa, lat):
        if fa not in FA or lat not in LAT:
            return False, f"bad verdict {fa!r}/{lat!r}"
        if self.state not in (FIRING, VERDICT) or self.pending is None:
            return False, "no shot is waiting for a verdict"
        self._finish(fa=fa, lat=lat)
        words = {("ok", "ok"): "on target"}
        self.message = words.get((fa, lat), f"logged: {fa}/{lat}")
        return True, self.message

    # ------------------------------------------------ estimates and advice

    def _live_shots(self, target):
        return [s for s in self.shots
                if s["target"] == target and not s["discarded"]
                and not s.get("retracted")]

    def estimates(self, target):
        fa, lat = AxisEstimator(), AxisEstimator()
        for s in self._live_shots(target):
            if s["range_m"] is not None and s["fa"] in FA:
                fa.add(s["range_m"], FA[s["fa"]])
            if s["lat_m"] is not None and s["lat"] in LAT:
                lat.add(s["lat_m"], LAT[s["lat"]])
        k = self.cfg["min_hits_for_band"]
        return (fa, fa.estimate(k)), (lat, lat.estimate(k))

    def start_range(self, target):
        """The nozzle model's first guess for this target, or None."""
        if self.model is None:
            return None
        tgt = next(x for x in self.targets if x[0] == target)
        z = target_height(self.cfg["deck_height_m"], tgt[2])
        x = self.model.solve_x(z, self.cfg["branch"])
        if x is None:
            return None
        return round(x + self.cfg["nozzle_x_m"] + self.cfg["face_setback_m"], 3)

    def live(self, now):
        s = [r for r in self.ranges if r[1]]
        if s and now - s[-1][0] <= self.cfg["range_max_age_s"]:
            t, _, rng, ang, lat = s[-1]
            return dict(src="lidar", range_m=rng, angle_deg=ang, lat_m=lat,
                        age_s=round(now - t, 2))
        if self.tape_range is not None:
            return dict(src="tape", range_m=self.tape_range, angle_deg=None,
                        lat_m=None, age_s=None)
        age = round(now - self.ranges[-1][0], 1) if self.ranges else None
        return dict(src="none", range_m=None, angle_deg=None, lat_m=None, age_s=age)

    def advice(self, now):
        cfg = self.cfg
        (fa_est, fa), (lat_est, lat) = self.estimates(self.target)
        goal = fa_est.next_value(cfg["step_m"], cfg["min_hits_for_band"])
        goal_how = fa["how"]
        if goal is not None and fa["centre"] is None:
            goal_how = f"one-sided so far: stepping {cfg['step_m'] * 100:.0f} cm past the miss"
        if goal is None:
            goal, goal_how = self.start_range(self.target), "nozzle model (a guess)"
        live = self.live(now)
        out = dict(goal_range_m=None if goal is None else round(goal, 3),
                   goal_how=goal_how, fa_dir=None, fa_cm=None, fa_text="",
                   lat_text="", angle_text="", warn=self._reach_warning(fa))
        rng = live["range_m"]
        if goal is None:
            out["fa_text"] = "take a first shot anywhere sensible"
        elif rng is None:
            out["fa_text"] = f"aim for {goal:.2f} m (no range measurement)"
        else:
            d = goal - rng
            cm = abs(d) * 100
            out["fa_cm"] = round(cm, 1)
            if abs(d) <= cfg["on_range_tol_m"]:
                out.update(fa_dir="on", fa_text="ON RANGE")
            elif d > 0:
                out.update(fa_dir="back", fa_text=f"{cm:.0f} cm BACK")
            else:
                out.update(fa_dir="fwd", fa_text=f"{cm:.0f} cm FORWARD")
        lat_goal = lat_est.next_value(cfg["step_m"] / 2, cfg["min_hits_for_band"])
        out["goal_lat_m"] = None if lat_goal is None else round(lat_goal, 3)
        if lat_goal is not None and live["lat_m"] is not None:
            d = lat_goal - live["lat_m"]
            if abs(d) <= cfg["on_range_tol_m"]:
                out["lat_text"] = "sideways: on"
            else:
                out["lat_text"] = (f"{abs(d) * 100:.0f} cm "
                                   + ("LEFT" if d > 0 else "RIGHT"))
        else:
            last = [s for s in self._live_shots(self.target) if s["lat"]]
            if last and last[-1]["lat"] != "ok":
                away = "right" if last[-1]["lat"] == "left" else "left"
                out["lat_text"] = f"last shot: too far {last[-1]['lat']}, nudge {away}"
        ang = live["angle_deg"]
        if ang is not None and abs(ang) > cfg["square_tol_deg"]:
            out["angle_text"] = f"square up: turn {'left' if ang > 0 else 'right'} " \
                                f"{abs(ang):.0f} deg"
        return out

    def model_reach(self, target):
        """Metres the model's arc tops out above (+) or below (-) this target's
        edge. Negative = the nozzle as configured cannot reach it at all."""
        if self.model is None:
            return None
        tgt = next(x for x in self.targets if x[0] == target)
        z = target_height(self.cfg["deck_height_m"], tgt[2])
        return round(self.model.apex()[1] - z, 3)

    def _reach_warning(self, fa):
        """The failure mode worth shouting about: shooting all afternoon at an
        edge the stream cannot get to. The data says so once the bracket has
        closed round the top of the arc with no hit in it."""
        reach = self.model_reach(self.target)
        if fa["n"] >= 6 and fa["n_hits"] == 0 and fa["lo"] is not None \
                and fa["hi"] is not None and abs(fa["hi"] - fa["lo"]) < 2 * self.cfg["step_m"]:
            return (f"{fa['n']} shots, no hit, and the bracket has closed at "
                    f"{fa['lo']:.2f}-{fa['hi']:.2f} m: the stream probably cannot "
                    "reach this edge. Raise or steepen the nozzle.")
        if reach is not None and reach < 0:
            return (f"the nozzle model tops out {-reach * 100:.1f} cm BELOW this "
                    "edge (check deck_height_m / nozzle_* against the boat)")
        return ""

    def yaml_snippet(self):
        lines = [f"# squirt_cal session {self.log.name}, {len(self.shots)} shot(s), "
                 f"config {self.cfg_hash}",
                 "# Ranges are what /crsd/wall_range reads BEFORE the burst.",
                 "squirt_calibration:"]
        for tid, label, _, _ in self.targets:
            (_, fa), (_, lat) = self.estimates(tid)
            if fa["centre"] is None:
                why = "no data yet" if not fa["n"] else (
                    f"{fa['n']} verdict(s), all on one side: not bracketed yet")
                lines.append(f"  {tid}_range_m: null   # {label}: {why}")
                continue
            note = fa["how"]
            if fa["band"]:
                note += f", hits {fa['band'][0]:.2f}-{fa['band'][1]:.2f}"
            elif fa["lo"] is not None or fa["hi"] is not None:
                lo = "?" if fa["lo"] is None else f"{fa['lo']:.2f}"
                hi = "?" if fa["hi"] is None else f"{fa['hi']:.2f}"
                note += f", bracket {lo}-{hi}"
            lines.append(f"  {tid}_range_m: {fa['centre']:.3f}   # {note}")
            if lat["centre"] is not None:
                lines.append(f"  {tid}_lat_m: {lat['centre']:.3f}   # {lat['how']}")
        return "\n".join(lines)

    # ------------------------------------------------ the page

    def _event(self, kind):
        self._event_seq += 1
        self.events.append((self._event_seq, kind))

    @staticmethod
    def _shot_row(s):
        keys = ("id", "target", "source", "burst_s", "range_m", "range_src",
                "angle_deg", "lat_m", "roll_deg", "pitch_deg", "pp_roll_deg",
                "pp_pitch_deg", "steady", "sticks_quiet_s", "fa", "lat",
                "discarded", "why", "snapshot", "t_wall")
        row = {k: s.get(k) for k in keys}
        row["retracted"] = bool(s.get("retracted"))
        return row

    def snapshot(self, layers=()):
        now = self.clock()
        with self.lock:
            st = self.steady.status(now)
            ok, why = self.fire_path()
            est = {}
            for tid, label, edge, latm in self.targets:
                (_, fa), (_, lat) = self.estimates(tid)
                est[tid] = dict(label=label, fa=fa, lat=lat,
                                start_range_m=self.start_range(tid),
                                model_reach_m=self.model_reach(tid))
            out = dict(
                t=round(now, 2), state=self.state, message=self.message,
                session=self.log.name, log_error=self.log.error,
                target=self.target,
                targets=[dict(id=t[0], label=t[1]) for t in self.targets],
                burst_s=self.burst_s, burst_choices=self.cfg["burst_choices_s"],
                arm_mode=self.arm_mode,
                armed_left_s=None if self.state != ARMED else round(
                    self.cfg["arm_timeout_s"] - (now - self.armed_at), 1),
                fire_ok=ok, fire_why=why, adapter=self.adapter.info(),
                live=self.live(now), tape_range=self.tape_range,
                steady=st, advice=self.advice(now), estimates=est,
                pending=None if self.pending is None else self._shot_row(self.pending),
                shots=[self._shot_row(s) for s in self.shots[-60:]],
                n_shots=len(self.shots), yaml=self.yaml_snippet(),
                events=list(self.events), fcu=self.fcu,
                mission_active=self.mission_active)
            if "fake" in layers:
                out["fake"] = self.adapter.fake_state(self.target)
            if "all" in layers:
                out["shots_all"] = [self._shot_row(s) for s in self.shots]
            return out

    def action(self, path, payload):
        now = self.clock()
        with self.lock:
            if path == "/arm":
                return self._arm(now, payload)
            if path == "/cancel":
                if self.state == ARMED:
                    self.state = IDLE
                    self.message = "cancelled"
                    return dict(ok=True, message="cancelled")
                return dict(ok=False, message="nothing armed")
            if path == "/verdict":
                ok, msg = self.verdict(payload.get("fa"), payload.get("lat"))
                return dict(ok=ok, message=msg)
            if path == "/discard":
                if self.pending is None or self.state not in (FIRING, VERDICT):
                    return dict(ok=False, message="no shot to discard")
                self._finish(discarded=True, why=str(payload.get("why", "discarded")))
                self.message = "discarded"
                return dict(ok=True, message="discarded")
            if path == "/undo":
                return self._undo()
            if path == "/mode":
                if payload.get("mode") not in ("steady", "now"):
                    return dict(ok=False, message=f"bad mode {payload.get('mode')!r}")
                if self.state != IDLE:
                    return dict(ok=False, message=f"busy ({self.state})")
                self.arm_mode = payload["mode"]
                return dict(ok=True, message=f"fire {'when steady' if self.arm_mode == 'steady' else 'now'}")
            if path == "/target":
                ids = [t[0] for t in self.targets]
                if payload.get("id") not in ids:
                    return dict(ok=False, message=f"unknown target {payload.get('id')!r}")
                self.target = payload["id"]
                return dict(ok=True, message=f"target {self.target}")
            if path == "/burst":
                try:
                    b = float(payload.get("burst_s"))
                except (TypeError, ValueError):
                    return dict(ok=False, message="burst_s must be a number")
                if b not in self.cfg["burst_choices_s"]:
                    return dict(ok=False, message=f"burst {b} s is not offered")
                self.burst_s = b
                return dict(ok=True, message=f"burst {b} s")
            if path == "/tape":
                v = payload.get("range_m")
                if v is None:
                    self.tape_range = None
                    return dict(ok=True, message="tape range cleared")
                try:
                    v = float(v)
                except (TypeError, ValueError):
                    return dict(ok=False, message="range_m must be a number")
                if not 0.1 <= v <= 5.0:
                    return dict(ok=False, message="tape range must be 0.1-5 m")
                self.tape_range = round(v, 3)
                return dict(ok=True, message=f"tape range {v:.2f} m")
            if path == "/session/new":
                if self.state in (ARMED, FIRING):
                    return dict(ok=False, message="finish the current shot first")
                if self.state == VERDICT:
                    self._finish(discarded=True, why="session ended")
                self.shots = []
                self.log = ShotLog(self.log_root, self.wall())
                self.message = f"new session {self.log.name}"
                return dict(ok=True, message=self.message)
            if path.startswith("/fake/"):
                return self.adapter.fake_action(path, payload, self)
        return dict(ok=False, message=f"unknown action {path}")

    def _arm(self, now, payload):
        if self.state != IDLE:
            return dict(ok=False, message=f"busy ({self.state})")
        if self.last_fire_t is not None and now - self.last_fire_t < self.cfg["min_gap_s"]:
            return dict(ok=False, message="too soon after the last shot")
        mode = payload.get("mode", "steady")
        if mode not in ("steady", "now"):
            return dict(ok=False, message=f"bad mode {mode!r}")
        ok, why = self.fire_path()
        if not ok:
            return dict(ok=False, message=f"cannot fire: {why}")
        self.arm_mode = mode
        self.state = ARMED
        self.armed_at = now
        self.message = "waiting for steady" if mode == "steady" else "firing"
        self._tick_armed(now)
        return dict(ok=True, message=self.message)

    def _undo(self):
        """Take back the last verdict (a fat finger on a phone). The shot goes
        back to waiting for a verdict; the log gets a retraction line."""
        if self.state != IDLE or not self.shots:
            return dict(ok=False, message="nothing to undo")
        last = self.shots.pop()
        self.log.write(dict(retract=last["id"], session=last["session"],
                            t_wall=round(self.wall(), 3)))
        for k in ("fa", "lat"):
            last[k] = None
        last.update(discarded=False, why="")
        self.pending = last
        self.state = VERDICT
        self.message = f"shot {last['id']}: verdict taken back"
        return dict(ok=True, message=self.message)
