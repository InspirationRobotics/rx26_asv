# VENDORED, UNMODIFIED BELOW THIS BLOCK, from the CV half of Task 3:
#   github.com/ChristopherCa55/firefighting-cv  ffcv/core/dock_sequence_core.py
#   commit e1173c5d19b9a42bf00c66bf4b58ccaaa17623c6 (2026-09-25)
#
# The simulated camera (world.py) runs THIS - the timing stage the dock
# detector will run on the boat - over synthetic per-frame window states, so
# the tree sees DockObservation.target_pattern appear with the real latency
# (two full code cycles, debounce, gap bridging) rather than an idealised one.
#
# Do not edit it here. If the CV side changes it, copy the new file over this
# one, update the commit line above, and re-run tools/task3_sim/test_world.py.
# The spec says it moves into crusader_perception as-is; when it does, import
# it from there and delete this copy.
"""Timing layer for Task 3: per-window light sequences -> patterns -> events.

ROS-free, pure Python. Feed it one observation per camera frame (time in
seconds + per-window states from colour_core); it debounces the states into
segments, recognises the light PATTERN on each window and emits events.

Patterns (handbook 3.3.4, "docking mission.pdf"; still confirm against the
rebuilt bay's light controller, whose timing tolerance is unknown):
    steady   one colour, continuously on                        (Core: RED target;
                                                                 GREEN for 5 s on a hit)
    flash    colour on 1 s / off 1 s, for 60 s                   (Advanced, after the
                                                                 5 s GREEN)
    code     c1 on 1 s, off 1 s, c2 on 1 s, off 2 s (5 s period, (Disruptive, after
             60 s); c1 = the colour after the long off; c1 may   GREEN 5 s + off 1 s)
             equal c2
    off      nothing lit

Events (dicts with "t", "type", "window", ...):
    pattern      a window's pattern became confirmed / changed
    hit          the target window turned GREEN (steady) for >= hit_green_min_s
    hit_done     GREEN held for green_hold_s (+- tol) and went away
    lost         a confirmed window has not been observed for > lost_s

Frame rate matters: the 2026-09-22 recordings are ~1 fps, which cannot resolve
1 s on / 1 s off (Nyquist) - the tests show the classifier refusing to call a
pattern at 1 fps. The recording checklist asks for >= 10 fps.
"""
from dataclasses import dataclass, field

LIT = ("red", "green", "blue")

DEFAULT = dict(debounce_s=0.25, gap_bridge_s=0.35, target_min_s=0.5, hit_green_min_s=0.4,
               green_hold_s=5.0, green_hold_tol_s=0.6, flash_on_s=1.0, flash_off_s=1.0,
               flash_tol_s=0.35, disruptive_long_off_s=2.0, min_flash_cycles=3, min_code_cycles=2,
               lost_s=2.0, min_fps=4.0)


@dataclass
class Segment:
    state: str
    t0: float
    t1: float

    @property
    def dur(self):
        return self.t1 - self.t0


@dataclass
class WindowTrack:
    """Debounced state of one window. 'unknown' and missing frames shorter than
    gap_bridge_s keep the current state; a new state must persist debounce_s."""
    cfg: dict
    segments: list = field(default_factory=list)
    cand: str = None
    cand_t0: float = None
    last_t: float = None
    last_seen: float = None
    dts: list = field(default_factory=list)

    def update(self, t, state):
        if self.last_t is not None:
            self.dts = (self.dts + [t - self.last_t])[-30:]
        self.last_t = t
        if state is None or state == "unknown":
            return
        self.last_seen = t
        if not self.segments:
            self.segments.append(Segment(state, t, t))
            return
        cur = self.segments[-1]
        if state == cur.state:
            cur.t1 = t
            self.cand = None
            return
        if self.cand != state:
            self.cand, self.cand_t0 = state, t
        # a change is accepted once it has persisted debounce_s, or immediately
        # when the frame rate is too low to wait (one frame then is the evidence)
        dt = self.frame_dt()
        if t - self.cand_t0 >= self.cfg["debounce_s"] or dt >= self.cfg["debounce_s"]:
            start = self.cand_t0
            # the boundary is half-way between the last old frame and the first new one
            cut = (cur.t1 + start) / 2.0 if start > cur.t1 else start
            cur.t1 = cut
            self.segments.append(Segment(state, cut, t))
            self.cand = None
            self.segments = self.segments[-40:]

    def frame_dt(self):
        if not self.dts:
            return 0.0
        s = sorted(self.dts)
        return s[len(s) // 2]


def _near(x, target, tol):
    return abs(x - target) <= tol


def classify(segments, cfg, fps):
    """-> (pattern, colours). Looks at the most recent closed segments."""
    if fps is not None and fps < cfg["min_fps"]:
        # too slow to resolve 1 s timing: only 'steady' can be called, and only
        # once it has lasted well beyond a flash period
        if segments and segments[-1].state in LIT and segments[-1].dur >= 3 * cfg["flash_on_s"] + 1:
            return "steady", (segments[-1].state,)
        return "unresolved", ()
    if not segments:
        return "off", ()
    cur = segments[-1]
    tol = cfg["flash_tol_s"]
    closed = segments[:-1]
    # code: [c1 1 s][off 1 s][c2 1 s][off 2 s], repeated. c1 is the colour AFTER
    # the long off; c1 may equal c2 (the handbook's pair is random), which the
    # long off still tells apart from a plain flash. The history can end at any
    # of the four phases, so every alignment is tried.
    need = 4 * cfg["min_code_cycles"]
    for shift in range(4):
        if len(closed) < need + shift:
            break
        last = closed[len(closed) - shift - need:len(closed) - shift]
        ok = True
        c1s, c2s = [], []
        for i, s in enumerate(last):
            ph = i % 4
            if ph == 0:
                ok &= s.state in LIT and _near(s.dur, cfg["flash_on_s"], tol)
                c1s.append(s.state)
            elif ph == 2:
                ok &= s.state in LIT and _near(s.dur, cfg["flash_on_s"], tol)
                c2s.append(s.state)
            elif ph == 1:
                ok &= s.state == "off" and _near(s.dur, cfg["flash_off_s"], tol)
            else:
                ok &= s.state == "off" and _near(s.dur, cfg["disruptive_long_off_s"], tol)
            if not ok:
                break
        if ok and len(set(c1s)) == 1 and len(set(c2s)) == 1:
            return "code", (c1s[0], c2s[0])
    # flash: alternating colour/off with 1 s / 1 s
    need = 2 * cfg["min_flash_cycles"]
    if len(closed) >= need:
        last = closed[-need:]
        on = [s for s in last if s.state in LIT]
        off = [s for s in last if s.state == "off"]
        if (len(on) == len(off) == need // 2 and len({s.state for s in on}) == 1 and
                all(_near(s.dur, cfg["flash_on_s"], tol) for s in on) and
                all(_near(s.dur, cfg["flash_off_s"], tol) for s in off) and
                all(a.state != b.state for a, b in zip(last, last[1:]))):
            return "flash", (on[0].state,)
    if cur.state in LIT and cur.dur >= max(cfg["target_min_s"], cfg["flash_on_s"] + tol):
        return "steady", (cur.state,)
    if cur.state == "off" and cur.dur >= cfg["disruptive_long_off_s"] + tol:
        return "off", ()
    return "pending", ()


class DockSequence:
    """Feed observations; read .events and .patterns.

        seq = DockSequence(cfg)
        seq.update(t, {0: "red", 1: "off"})        # window index -> state
    """

    def __init__(self, cfg=None):
        self.cfg = dict(DEFAULT, **(cfg or {}))
        self.tracks = {}
        self.patterns = {}
        self.events = []
        self.target = None
        self._green_since = None
        self._hit = False
        self._armed = False

    def update(self, t, window_states):
        for w, s in window_states.items():
            self.tracks.setdefault(w, WindowTrack(self.cfg)).update(t, s)
        new = []
        for w, tr in self.tracks.items():
            dt = tr.frame_dt()
            fps = 1.0 / dt if dt > 0 else None
            if tr.last_seen is not None and t - tr.last_seen > self.cfg["lost_s"]:
                if self.patterns.get(w, ("lost",))[0] != "lost":
                    self.patterns[w] = ("lost", ())
                    new.append(dict(t=t, type="lost", window=w))
                continue
            p = classify(tr.segments, self.cfg, fps)
            if p[0] in ("pending",):
                continue
            if self.patterns.get(w) != p:
                self.patterns[w] = p
                new.append(dict(t=t, type="pattern", window=w, pattern=p[0], colours=list(p[1])))
        # target = the window showing a lit pattern that is not steady GREEN
        lit = [w for w, p in self.patterns.items() if p[0] in ("steady", "flash", "code")
               and not (p[0] == "steady" and p[1] == ("green",))]
        if len(lit) == 1:
            self.target = lit[0]
        # a new hit can only follow a steady RED target (Core). After a hit the
        # Advanced / Disruptive signals may flash GREEN; that is not another hit.
        if self.target is not None and self.patterns.get(self.target) == ("steady", ("red",)):
            self._armed = True
        # hit: the target window turns steady GREEN
        if self.target is not None and self.target in self.tracks and self._armed:
            segs = self.tracks[self.target].segments
            cur = segs[-1] if segs else None
            if cur is not None and cur.state == "green":
                if self._green_since is None:
                    self._green_since = cur.t0
                if not self._hit and t - self._green_since >= self.cfg["hit_green_min_s"]:
                    self._hit = True
                    new.append(dict(t=t, type="hit", window=self.target, since=round(self._green_since, 3)))
            elif self._green_since is not None and cur is not None:
                held = cur.t0 - self._green_since
                if self._hit:
                    ok = abs(held - self.cfg["green_hold_s"]) <= self.cfg["green_hold_tol_s"]
                    new.append(dict(t=t, type="hit_done", window=self.target, held_s=round(held, 3),
                                    hold_ok=ok))
                    self._armed = False
                self._green_since, self._hit = None, False
        self.events += new
        return new
