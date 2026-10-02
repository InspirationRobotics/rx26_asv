"""task1_judge — an independent Task 1 referee, from the boat's TRUE path.

    python3 -m crusader_sim.task1_judge --course task1_core     # standalone, Ctrl-C for the verdict
    (task1_goal runs one alongside every mission and prints its verdict)

Runs in: the crsd-sim container. Reads only /sim/crusader/odometry (ground truth)
and the course file — nothing the boat's stack computes — so it can disagree
with the tree, which is the point: the tree's own "passed correctly" counter is
the tree grading itself.

Also imported by the Task 1 panel (plain python3 on the WSL host, no ROS — so
rclpy is imported only inside main()), which changes beacons mid-run through
set_states() and feeds positions to update() itself.

Rules (handbook 3.3.2:19-33):
    flashing RED   kept to STARBOARD      flashing GREEN   kept to PORT
    ENTRY (flashing BLUE) circled CLOCKWISE,  EXIT (steady BLUE) COUNTER-clockwise
    no contact with any buoy
EVERY red and green buoy is scored on its own, paired or not (handbook 3.3.2 defines no
gates): the boat PASSES a buoy at the moment it goes from ahead of the track to behind
it (the track abeam of the buoy, travel direction taken over the last >= 0.3 m), and
red must then be on the boat's starboard side, green on its port. A buoy the boat
passes more than once (an orbit, a loop) is judged on its CLOSEST pass; a buoy it never
passes is "not passed" and fails. This is the verdict: `pass` needs every red and green
correct, no contact, ENTRY circled clockwise and EXIT counter-clockwise.
Gates are kept as an informational extra (each red paired with its nearest green,
<= 15 m, judged when the boat's path crosses the segment between them; "correct" =
red on the starboard side at the crossing) and are NOT part of `pass`: a pair the
boat has to take apart (a red on the left of its green) is never crossed.
A circle is >= 330 deg of unwrapped bearing swept around the buoy while within
circle_radius_m of it.

Clearance (docs/nav2_avoidance_spec.md section 7) is reported, never part of
`pass`: the referee grades the task, clearance is our engineering metric. Per
object (every buoy, plus the course's platforms, docks and launch pads) the
verdict has the minimum CENTRE-TO-SURFACE distance (boat centre to the object's
box: the planner's contract), the minimum HULL GAP (hull rectangle to the box,
only while yaw is known) and when each happened. `non_gate_centre_m` leaves out
samples inside a gate's corridor for that gate's own two buoys, because crossing
a gate is meant to pass close to them. `--selftest` checks the arithmetic.
"""
import argparse
import collections
import json
import math
import threading
import time

from crusader_sim import course as C

# Contact = a buoy's collision box (0.43 m square, gen_world) reaching the hull's
# 1.0 x 0.6 m footprint (crusader_hull.yaml hull length_m/beam_m). Given the
# boat's yaw that is tested against the hull rectangle; without it, a circle of
# half-beam + half-width, which is right abeam but blind ahead: a bow pressed
# against a buoy sits 0.70 m centre to centre, and a 0.55 m circle missed
# exactly that (Task 1 panel, real sim, 2026-09-30).
HULL_HALF_L, HULL_HALF_B = 0.5, 0.3
BUOY_HALF_M = 0.22
CONTACT_SLACK_M = 0.03
CONTACT_M = HULL_HALF_B + BUOY_HALF_M + CONTACT_SLACK_M     # 0.55, the no-yaw circle
CIRCLE_DEG = 330.0
GATE_PAIR_M = 15.0
# A buoy is passed when it goes from ahead of the boat to behind it. "Ahead" is judged against the
# direction of travel taken over the last PASS_CHORD_M of track (a single 50 ms step is noise), and
# the track history is thinned to one point per PASS_STEP_M so a hovering boat does not grow it.
PASS_CHORD_M = 0.3
PASS_STEP_M = 0.01
SIDE_CONSTRAINED = ("flash_red", "flash_green")
# clearance: the planner's 0.8 m contract less 0.1 m for quantisation (spec 7), and
# the half-width of the gate corridor whose samples are left out of the non-gate figure
CLEARANCE_OK_M = 0.70
GATE_CORRIDOR_M = 3.0
# gen_world.dock(): 0.5 m fingers 2.0 m long, three 1.5 m slips, a 1.0 m deep deck;
# platforms and launch pads are 2 x 2 m (gen_world.platform / launch_pad)
DOCK_SLIP_M, DOCK_FINGER_M, DOCK_FINGER_LEN_M, DOCK_DECK_M = 1.5, 0.5, 2.0, 1.0
PAD_HALF_M = 1.0
# A rectangle is (cx, cy, yaw, half_x, half_y): centre, orientation, and half
# extents along its own x and y axes. Every footprint below is a list of them.


def _corners(rect):
    cx, cy, yaw, hx, hy = rect
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]


def _point_rect_dist(x, y, rect):
    """Distance from a point to a rectangle's surface; 0 inside it."""
    cx, cy, yaw, hx, hy = rect
    dx, dy = x - cx, y - cy
    lx = math.cos(yaw) * dx + math.sin(yaw) * dy
    ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return math.hypot(max(abs(lx) - hx, 0.0), max(abs(ly) - hy, 0.0))


def _point_segment_dist(p, a, b):
    ex, ey = b[0] - a[0], b[1] - a[1]
    t = ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / (ex * ex + ey * ey)
    t = min(max(t, 0.0), 1.0)
    return math.hypot(p[0] - a[0] - t * ex, p[1] - a[1] - t * ey)


def _rect_gap(a, b):
    """Distance between two rectangles' surfaces; 0 when they touch or overlap."""
    ca, cb = _corners(a), _corners(b)

    def span(corners, axis):
        v = [axis[0] * x + axis[1] * y for x, y in corners]
        return min(v), max(v)

    # separating-axis test on the four face normals: overlapping on all of them = touching
    axes = [(math.cos(r[2] + k * math.pi / 2), math.sin(r[2] + k * math.pi / 2))
            for r in (a, b) for k in (0, 1)]
    if not any(span(ca, ax)[1] < span(cb, ax)[0] or span(cb, ax)[1] < span(ca, ax)[0] for ax in axes):
        return 0.0
    # disjoint convex polygons: the nearest pair is a corner of one against an edge of the other
    edges = lambda c: [(c[i], c[(i + 1) % 4]) for i in range(4)]
    return min([_point_segment_dist(p, *e) for p in ca for e in edges(cb)]
               + [_point_segment_dist(p, *e) for p in cb for e in edges(ca)])


def _footprint(e):
    """The rectangles a course element occupies, in the world frame: a platform or
    launch pad is one 2 x 2 m square; a dock is its deck plus four fingers
    (gen_world.dock's frame: (x, y) = the middle of the deck's BACK edge, facing_deg
    = the way the bays point out, local +u out and +r to the viewer's right).
    Anything else (a buoy is handled by the caller) has no footprint."""
    kind, x, y = e.get("type"), float(e["x"]), float(e["y"])
    if kind in ("platform", "launch_pad"):
        return [(x, y, math.radians(float(e.get("yaw_deg", 0.0))), PAD_HALF_M, PAD_HALF_M)]
    if kind != "dock":
        return []
    f = math.radians(float(e.get("facing_deg", 180.0)))
    uc, us = math.cos(f), math.sin(f)

    def rect(u, r, su, sr):
        return (x + u * uc - r * us, y + u * us + r * uc, f, su / 2, sr / 2)

    width = 4 * DOCK_FINGER_M + 3 * DOCK_SLIP_M
    out = [rect(DOCK_DECK_M / 2, 0.0, DOCK_DECK_M, width)]
    for i in range(4):
        r = -width / 2 + DOCK_FINGER_M / 2 + i * (DOCK_FINGER_M + DOCK_SLIP_M)
        out.append(rect(DOCK_DECK_M + DOCK_FINGER_LEN_M / 2, r, DOCK_FINGER_LEN_M, DOCK_FINGER_M))
    return out


def _keep_min(m, key, t_key, value, t):
    """Record value (and when) in m if it beats the minimum held under key."""
    if value < m[key]:
        m[key], m[t_key] = value, t


def _round_or_none(v, n=3):
    """A minimum that was never measured is None in the verdict, not infinity."""
    return None if v is None or math.isinf(v) else round(v, n)


def _in_corridor(p, a, b, half_width_m):
    """Is p between the buoys a and b along their line, within half_width_m of it?"""
    ax, ay = b[0] - a[0], b[1] - a[1]
    length = math.hypot(ax, ay)
    if length == 0.0:
        return False
    along = ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / length
    off = abs(_cross(ax, ay, p[0] - a[0], p[1] - a[1])) / length
    return 0.0 <= along <= length and off <= half_width_m


def _cross(ax, ay, bx, by):
    return ax * by - ay * bx


def _segments_intersect(p1, p2, q1, q2):
    d1 = _cross(q2[0] - q1[0], q2[1] - q1[1], p1[0] - q1[0], p1[1] - q1[1])
    d2 = _cross(q2[0] - q1[0], q2[1] - q1[1], p2[0] - q1[0], p2[1] - q1[1])
    d3 = _cross(p2[0] - p1[0], p2[1] - p1[1], q1[0] - p1[0], q1[1] - p1[1])
    d4 = _cross(p2[0] - p1[0], p2[1] - p1[1], q2[0] - p1[0], q2[1] - p1[1])
    return (d1 * d2 < 0) and (d3 * d4 < 0)


_COLOUR_WORD = {"flash_red": "red", "flash_green": "green"}


def _describe_pass(rec):
    """One buoy's line in the verdict. rec is _track_passes's record, or None (never passed)."""
    if rec is None:
        return "not passed"
    return (f"{'correct' if rec['ok'] else 'WRONG SIDE'} ({_COLOUR_WORD[rec['colour']]} kept to "
            f"{rec['side']}, passed at {rec['range']:.1f} m)")


def _touching(x, y, yaw, bx, by):
    """Is the buoy at (bx, by) touching a hull centred at (x, y)? With yaw, the
    buoy's centre within BUOY_HALF_M + slack of the hull rectangle; without, the
    CONTACT_M circle."""
    dx, dy = bx - x, by - y
    if yaw is None:
        return math.hypot(dx, dy) < CONTACT_M
    fwd = math.cos(yaw) * dx + math.sin(yaw) * dy
    left = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return math.hypot(max(abs(fwd) - HULL_HALF_L, 0.0), max(abs(left) - HULL_HALF_B, 0.0)) \
        < BUOY_HALF_M + CONTACT_SLACK_M


class Task1Judge:
    def __init__(self, course, circle_radius_m=8.0, echo=True):
        self.buoys = C.buoys(course)
        self.radius = circle_radius_m
        self.echo = echo
        self.gates = []
        self.circles = {}
        self.contacts = set()
        self.prev = None
        self.events = []
        # per-buoy side scoring: name -> the closest pass so far (see _track_passes); the track
        # history the travel direction comes from; name -> "ahead" figure at the last sample
        self.passes = {}
        self._hist = collections.deque(maxlen=400)
        self._ahead = {}
        # clearance: every buoy is the BUOY_HALF_M box; platforms, docks and pads come
        # from the course's elements. Positions never move (set_states changes beacons)
        self.shapes = {b[0]: [(b[1], b[2], 0.0, BUOY_HALF_M, BUOY_HALF_M)] for b in self.buoys}
        for e in course.get("elements", []):
            rects = _footprint(e)
            if rects:
                self.shapes[e["name"]] = rects
        self.clear = {}               # name -> the minima so far, see _track_clearance
        self._t0 = None
        # the panel calls set_states from its web thread while positions arrive
        # on another; update() must never walk a half-rebuilt gate list
        self._lock = threading.Lock()
        self._rebuild()

    def _rebuild(self):
        """Gates and circles from the buoys' CURRENT beacons — the one pairing
        rule, for the course as loaded and after every set_states.

        What was already earned survives a beacon change: a judged gate keeps
        its result (it was graded under the beacons showing when the boat
        crossed it), even if its buoys no longer pair — and it is not re-opened
        when the same two buoys pair again with the colours swapped. A circle
        keeps its swept angle while its buoy still wants the same direction;
        a changed direction starts from zero."""
        reds = [b for b in self.buoys if b[3] == "flash_red"]
        greens = [b for b in self.buoys if b[3] == "flash_green"]
        judged = {frozenset((g["red"], g["green"])): g for g in self.gates if g["result"]}
        gates = []
        for r in reds:
            g = min(greens, key=lambda g: math.hypot(g[1] - r[1], g[2] - r[2]), default=None)
            if g is not None and math.hypot(g[1] - r[1], g[2] - r[2]) <= GATE_PAIR_M:
                gates.append(judged.pop(frozenset((r[0], g[0])), None)
                             or {"red": r[0], "green": g[0], "r": (r[1], r[2]),
                                 "g": (g[1], g[2]), "result": None})
        self.gates = list(judged.values()) + gates
        circles = {}
        for b in self.buoys:
            if b[3] in ("flash_blue", "steady_blue"):
                want = "cw" if b[3] == "flash_blue" else "ccw"
                old = self.circles.get(b[0])
                circles[b[0]] = old if old and old["want"] == want else {
                    "want": want, "xy": (b[1], b[2]), "swept": 0.0, "last": None, "done": None}
        self.circles = circles

    def set_states(self, states):
        """Change beacons mid-run: {buoy name: beacon state}. Buoys not named
        keep theirs. Returns the changes as "name old->new" strings."""
        with self._lock:
            known = {b[0] for b in self.buoys}
            for name, state in states.items():
                if name not in known:
                    raise ValueError(f"no buoy {name!r} in this course")
                if state not in C.BEACON_STATES:
                    raise ValueError(f"{name}: beacon {state!r} is not one of {C.BEACON_STATES}")
            changed = [f"{b[0]} {b[3]}->{states[b[0]]}" for b in self.buoys
                       if states.get(b[0], b[3]) != b[3]]
            if changed:
                self.buoys = [b[:3] + (states.get(b[0], b[3]),) + b[4:] for b in self.buoys]
                self._rebuild()
                self._say("states changed: " + ", ".join(changed))
            return changed

    def update(self, x, y, yaw=None, t=None):
        """The boat's true position; yaw (rad, ENU) makes the contact test the
        hull's rectangle rather than a circle, and gives the hull gap. t is the
        sample's time in seconds; omitted, it is the time since the first sample."""
        with self._lock:
            if t is None:
                self._t0 = time.monotonic() if self._t0 is None else self._t0
                t = time.monotonic() - self._t0
            self._update(x, y, yaw, t)

    def _track_clearance(self, x, y, yaw, t):
        """Fold one sample into every object's minima: centre-to-surface, hull gap
        (only with yaw), and the same centre figure over the samples OUTSIDE the
        gate corridors of that object's own gate (an object in no gate has all
        its samples count)."""
        in_gate = set()
        for g in self.gates:
            if _in_corridor((x, y), g["r"], g["g"], GATE_CORRIDOR_M):
                in_gate.update((g["red"], g["green"]))
        hull = None if yaw is None else (x, y, yaw, HULL_HALF_L, HULL_HALF_B)
        for name, rects in self.shapes.items():
            m = self.clear.setdefault(name, {"centre": math.inf, "t": None, "hull": math.inf,
                                             "t_hull": None, "non_gate": math.inf})
            centre = min(_point_rect_dist(x, y, r) for r in rects)
            _keep_min(m, "centre", "t", centre, t)
            if name not in in_gate:
                m["non_gate"] = min(m["non_gate"], centre)
            if hull is not None:
                _keep_min(m, "hull", "t_hull", min(_rect_gap(hull, r) for r in rects), t)

    def _travel_dir(self, p):
        """The direction of travel at p: from the newest earlier track point at least
        PASS_CHORD_M back (the oldest one, when the whole track is shorter than that). None
        while the boat has not moved."""
        for q in reversed(self._hist):
            if math.hypot(p[0] - q[0], p[1] - q[1]) >= PASS_CHORD_M:
                return (p[0] - q[0], p[1] - q[1])
        if self._hist:
            d = (p[0] - self._hist[0][0], p[1] - self._hist[0][1])
            if math.hypot(*d) > 1e-6:
                return d
        return None

    def _track_passes(self, p, t):
        """Score every red and green buoy on its own. A pass is the moment a buoy goes from
        ahead of the boat to behind it; the side it is on then, against the direction of
        travel, is the result (red must be to starboard, green to port). Of several passes
        (an orbit sweeps the whole field) the CLOSEST one is the buoy's pass: a buoy the track
        goes past at 4 m and is circled at 12 m is judged at 4. Only buoys that are red or
        green at the time are scored; a pass already earned survives a later recolour."""
        d = self._travel_dir(p)
        if d is None:
            return
        for name, bx, by, state, *_ in self.buoys:
            if state not in SIDE_CONSTRAINED:
                self._ahead.pop(name, None)
                continue
            rel = (bx - p[0], by - p[1])
            ahead = d[0] * rel[0] + d[1] * rel[1]
            before, self._ahead[name] = self._ahead.get(name), ahead
            if before is None or not (before > 0.0 >= ahead):
                continue
            rng = math.hypot(*rel)
            old = self.passes.get(name)
            if old is not None and old["range"] <= rng:
                continue
            side = "starboard" if _cross(d[0], d[1], rel[0], rel[1]) < 0 else "port"
            want = "starboard" if state == "flash_red" else "port"
            self.passes[name] = {"colour": state, "side": side, "range": rng, "t": t,
                                 "ok": side == want}
            if old is None or old["ok"] != (side == want):
                self._say(f"passed {name} ({_COLOUR_WORD[state]}) with it to {side} at {rng:.1f} m: "
                          f"{'correct' if side == want else 'WRONG SIDE'}")

    def _buoy_results(self):
        """[(name, record or None)] for every buoy that is scored: red or green now, or passed
        while it was. A record is _track_passes's; None = never passed."""
        names = {b[0]: b[3] in SIDE_CONSTRAINED for b in self.buoys}
        return [(n, self.passes.get(n)) for n in names if names[n] or n in self.passes]

    def _update(self, x, y, yaw, t):
        self._track_clearance(x, y, yaw, t)
        p = (x, y)
        if self.prev is None:
            self._hist.append(p)                    # the track starts here
        if self.prev is not None and p != self.prev:
            for gate in self.gates:
                if gate["result"] is None and _segments_intersect(self.prev, p, gate["r"], gate["g"]):
                    # travel direction d; red is to STARBOARD iff cross(d, red - p) < 0
                    d = (p[0] - self.prev[0], p[1] - self.prev[1])
                    side = _cross(d[0], d[1], gate["r"][0] - p[0], gate["r"][1] - p[1])
                    gate["result"] = "correct" if side < 0 else "WRONG WAY"
                    self._say(f"gate {gate['red']}/{gate['green']}: {gate['result']}")
            self._track_passes(p, t)
            if not self._hist or math.hypot(p[0] - self._hist[-1][0], p[1] - self._hist[-1][1]) >= PASS_STEP_M:
                self._hist.append(p)
        for name, c in self.circles.items():
            dx, dy = x - c["xy"][0], y - c["xy"][1]
            if math.hypot(dx, dy) <= self.radius:
                a = math.atan2(dy, dx)
                if c["last"] is not None:
                    c["swept"] += (a - c["last"] + math.pi) % (2 * math.pi) - math.pi
                c["last"] = a
                deg = math.degrees(c["swept"])
                if c["done"] is None and abs(deg) >= CIRCLE_DEG:
                    got = "ccw" if deg > 0 else "cw"
                    c["done"] = "correct" if got == c["want"] else f"WRONG ({got})"
                    self._say(f"circled {name} {got}: {c['done']}")
            else:
                c["last"] = None
        for b in self.buoys:
            if b[0] not in self.contacts and _touching(x, y, yaw, b[1], b[2]):
                self.contacts.add(b[0])
                self._say(f"CONTACT with {b[0]}")
        self.prev = p

    def _say(self, s):
        self.events.append(s)
        if self.echo:
            print(f"[judge] {s}", flush=True)

    def _clearance_verdict(self):
        """The `min_clearance` block. A figure never measured is None, not a number:
        the hull gap needs yaw, and a gate-only object has no non-gate samples."""
        by_object = {n: {"centre_m": _round_or_none(m["centre"]), "hull_m": _round_or_none(m["hull"]),
                         "t": _round_or_none(m["t"], 2), "t_hull": _round_or_none(m["t_hull"], 2)}
                     for n, m in self.clear.items()}
        measured = {n: m for n, m in self.clear.items() if not math.isinf(m["centre"])}
        worst = min(measured, key=lambda n: measured[n]["centre"], default=None)
        non_gate = min((m["non_gate"] for m in measured.values()), default=math.inf)
        hull = min((m["hull"] for m in measured.values()), default=math.inf)
        return {"centre_m": None if worst is None else _round_or_none(measured[worst]["centre"]),
                "worst": worst, "non_gate_centre_m": _round_or_none(non_gate),
                "hull_m": _round_or_none(hull),
                "by_object": by_object,
                # nothing measured, or only gate samples, cannot show a violation
                "clearance_ok": math.isinf(non_gate) or non_gate >= CLEARANCE_OK_M}

    def verdict(self):
        with self._lock:
            gates_ok = sum(1 for g in self.gates if g["result"] == "correct")
            circles = {n: (c["done"] or f"not circled ({math.degrees(c['swept']):.0f} deg)")
                       for n, c in self.circles.items()}
            results = self._buoy_results()
            buoys_ok = sum(1 for _, r in results if r is not None and r["ok"])
            ok = (buoys_ok == len(results) and not self.contacts
                  and all(c["done"] == "correct" for c in self.circles.values()))
            return {"pass": ok, "buoys_correct": buoys_ok, "buoys": len(results),
                    "buoys_detail": {n: _describe_pass(r) for n, r in results},
                    # the gate pairing is informational (module docstring): it is not in `pass`
                    "gates_correct": gates_ok, "gates": len(self.gates),
                    "gates_detail": {f"{g['red']}/{g['green']}": g["result"] or "not crossed"
                                     for g in self.gates},
                    "circles": circles, "contacts": sorted(self.contacts),
                    "min_clearance": self._clearance_verdict()}


def format_verdict(v):
    if "buoys" in v:                        # a verdict from before per-buoy scoring has no buoys
        head = (f"buoys {v['buoys_correct']}/{v['buoys']} on their side (gates "
                f"{v['gates_correct']}/{v['gates']} crossed)")
    else:
        head = f"gates {v['gates_correct']}/{v['gates']} correct"
    lines = [f"[judge] VERDICT: {'PASS' if v['pass'] else 'FAIL'} — {head}, "
             f"contacts {v['contacts'] or 'none'}"]
    lines += [f"[judge]   buoy {k}: {r}" for k, r in v.get("buoys_detail", {}).items()]
    lines += [f"[judge]   gate {k}: {r}" for k, r in v["gates_detail"].items()]
    lines += [f"[judge]   circle {k}: {r}" for k, r in v["circles"].items()]
    mc = v.get("min_clearance")
    if mc is not None:                      # a verdict from before clearance has none
        m = lambda x: "n/a" if x is None else f"{x:.2f}"
        lines.append(f"[judge]   min clearance {m(mc['centre_m'])} m ({mc['worst'] or 'n/a'}), "
                     f"non-gate {m(mc['non_gate_centre_m'])} m, hull {m(mc['hull_m'])} m")
    return "\n".join(lines)


def _selftest():
    """The clearance arithmetic against hand-computed figures. Plain python3, no ROS.
    Returns the number of failed checks."""
    def run(elements, a, b, yaw=0.0, step=0.05):
        j = Task1Judge({"elements": elements}, echo=False)
        n = max(1, round(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
        for i in range(n + 1):
            j.update(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n, yaw, t=i * step)
        return j.verdict()["min_clearance"]

    black = lambda name, x, y: {"type": "robobuoy", "name": name, "x": x, "y": y, "beacon": "off"}
    gate = [{"type": "robobuoy", "name": "red", "x": 0.0, "y": -0.8, "beacon": "flash_red"},
            {"type": "robobuoy", "name": "grn", "x": 0.0, "y": 0.8, "beacon": "flash_green"}]
    dock = {"type": "dock", "name": "dock", "x": 0.0, "y": 20.0, "facing_deg": -90.0}
    fails = []

    def check(what, got, want, tol=0.01):
        ok = got is not None and abs(got - want) <= tol if isinstance(want, float) else got == want
        print(f"  {'ok  ' if ok else 'FAIL'} {what}: {got} (want {want})")
        if not ok:
            fails.append(what)

    # the spec's case: abeam of a black buoy, 0.5 m off its centre
    mc = run([black("b", 10, 0)], (0, 0.5), (20, 0.5))
    check("abeam 0.5 m: centre_m = 0.5 - 0.22", mc["centre_m"], 0.28)
    check("abeam 0.5 m: clearance_ok", mc["clearance_ok"], False)
    mc = run([black("b", 10, 0)], (0, 1.0), (20, 1.0))
    check("abeam 1.0 m: centre_m", mc["centre_m"], 0.78)
    check("abeam 1.0 m: hull_m = 0.78 - 0.3 half-beam", mc["hull_m"], 0.48)
    check("abeam 1.0 m: clearance_ok", mc["clearance_ok"], True)
    mc = run([black("b", 10, 0)], (0, 0), (9.0, 0))
    check("head-on to 1.0 m: hull_m = 0.78 - 0.5 half-length", mc["hull_m"], 0.28)
    mc = run([{"type": "platform", "name": "plat", "x": 10.0, "y": 0.0}], (0, 2.0), (20, 2.0))
    check("platform 2 m square, 2.0 m off its centre: centre_m", mc["centre_m"], 1.0)
    mc = run([dock], (0, 10), (0, 18), yaw=math.pi / 2)
    check("dock, mid-slip with the hull's bow at 18.5: centre_m to a finger", mc["centre_m"], 0.75)
    check("dock: hull_m to a finger = 0.75 - 0.3", mc["hull_m"], 0.45)
    # a gate's own buoys are excused inside its corridor, everything else is not
    mc = run(gate, (-10, 0), (10, 0))
    check("gate crossed mid-way: centre_m = 0.8 - 0.22 (reported)", mc["centre_m"], 0.58)
    check("gate crossed mid-way: non-gate figure is the approach, not the crossing",
          mc["non_gate_centre_m"] is not None and mc["non_gate_centre_m"] > 2.0, True)
    check("gate crossed mid-way: clearance_ok", mc["clearance_ok"], True)
    mc = run(gate + [black("b", 0.0, 5.0)], (-10, 0), (10, 0))
    check("gate plus a black buoy at 5 m: still ok", mc["clearance_ok"], True)
    mc = run(gate + [black("b", 6.0, 0.4)], (-10, 0), (10, 0))
    check("gate plus a black buoy 0.4 m off the track: not ok", mc["clearance_ok"], False)
    check("... and it is the worst", mc["worst"], "b")
    print("task1_judge selftest: " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
    return len(fails)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--course", default="task1_core")
    ap.add_argument("--selftest", action="store_true", help="check the clearance arithmetic and exit")
    a = ap.parse_args()
    if a.selftest:
        raise SystemExit(1 if _selftest() else 0)
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String

    judge = Task1Judge(C.load(a.course))
    rclpy.init()
    node = Node("task1_judge")
    pub = node.create_publisher(String, "/sim/task1_judge", 10)
    node.create_subscription(
        Odometry, "/sim/crusader/odometry",
        lambda m: judge.update(m.pose.pose.position.x, m.pose.pose.position.y), 10)
    node.create_timer(1.0, lambda: pub.publish(String(data=json.dumps(judge.verdict()))))
    print(f"[judge] {len(judge.gates)} gates, circles {list(judge.circles)}; Ctrl-C for the verdict")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(format_verdict(judge.verdict()))


if __name__ == "__main__":
    main()
