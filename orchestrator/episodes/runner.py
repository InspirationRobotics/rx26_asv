"""EpisodeRunner — drives one scripted episode against a backend and records a trace.

Phase-0 scope: waypoint-following with timed events (keep-out / moving-object
injection, All Clear). The mission planner (Phase 4) will replace the naive
waypoint script with real task logic; the trace/metrics contract stays the same.

Threading-model compliance: run() checks a threading.Event every
cycle so a supervising harness (Level 1/2 loops, Phase 5) can tear an episode
down deterministically. Single-threaded otherwise.
"""
import math
import threading
from dataclasses import dataclass, field

from episodes.scenario import Scenario, KeepOut, MovingObject


@dataclass
class Sample:
    t: float
    x: float
    y: float
    heading: float
    speed: float
    # commanded goal at sample time (None = legacy fallback to final waypoint).
    # Lets objective-2 distinguish a COMMANDED hold (loiter, dp_hold) from a
    # local-minimum stall — same semantics as the onboard ProgressMonitor.
    goal_x: float = None
    goal_y: float = None


@dataclass
class EpisodeResult:
    scenario_name: str
    scenario_version: str
    seed: int
    trace: list = field(default_factory=list)          # [Sample]
    keepouts: list = field(default_factory=list)       # [KeepOut] incl. injected
    moving_objects: list = field(default_factory=list) # [MovingObject]
    waypoints_reached: list = field(default_factory=list)  # [bool] per waypoint
    timed_out: bool = False
    stopped_early: bool = False       # external stop_event fired
    wall_clock_s: float = 0.0


class EpisodeRunner:
    def __init__(self, scenario: Scenario, backend, seed: int = 0,
                 dt: float = 0.1, sample_every: int = 1,
                 stop_event: "threading.Event | None" = None,
                 advisor=None):
        """advisor (optional): object with
        advise(t, x, y, heading, goal_xy, keepouts, moving_objects)
            -> (goal_x, goal_y, speed_scale)
        called every step; its corrected goal is fed to the backend while
        waypoint acceptance is still checked against the ORIGINAL waypoint
        (advisory contract: the advisor shapes the path, never the mission)."""
        self.scenario = scenario
        self.backend = backend
        self.seed = seed
        self.dt = dt
        self.sample_every = max(1, sample_every)
        self.stop_event = stop_event or threading.Event()
        self.advisor = advisor

    def run(self) -> EpisodeResult:
        import time as _time
        sc = self.scenario
        res = EpisodeResult(sc.name, sc.version, self.seed,
                            keepouts=list(sc.keepouts),
                            moving_objects=list(sc.moving_objects))
        wall_start = _time.time()
        self.backend.reset(sc, self.seed)
        try:
            t = 0.0
            step_i = 0
            pending = sorted(sc.events, key=lambda e: e.t)
            for wi, (wx, wy) in enumerate(sc.waypoints):
                self.backend.set_target(wx, wy)
                reached = False
                while t < sc.timeout_s:
                    if self.stop_event.is_set():
                        res.stopped_early = True
                        return res
                    # fire due events
                    while pending and pending[0].t <= t:
                        self._apply_event(pending.pop(0), res, t)
                    if self.advisor is not None:
                        cx, cy, chdg, _ = self.backend.state()
                        gx, gy, scale = self.advisor.advise(
                            t, cx, cy, chdg, (wx, wy),
                            res.keepouts, res.moving_objects)
                        self.backend.set_target(gx, gy)
                        if hasattr(self.backend, "set_speed_scale"):
                            self.backend.set_speed_scale(scale)
                    self.backend.step(self.dt)
                    t += self.dt
                    step_i += 1
                    x, y, hdg, spd = self.backend.state()
                    if step_i % self.sample_every == 0:
                        res.trace.append(Sample(t, x, y, hdg, spd, wx, wy))
                    if math.hypot(wx - x, wy - y) <= sc.wp_radius:
                        reached = True
                        break
                res.waypoints_reached.append(reached)
                if t >= sc.timeout_s:
                    res.timed_out = True
                    # remaining waypoints unreached
                    res.waypoints_reached += [False] * (len(sc.waypoints) - wi - 1)
                    break
        finally:
            self.backend.shutdown()      # deterministic teardown, always
            res.wall_clock_s = _time.time() - wall_start
        return res

    def _apply_event(self, ev, res: EpisodeResult, t: float):
        if ev.type == "inject_keepout":
            res.keepouts.append(KeepOut(active_from=t, **ev.data))
        elif ev.type == "inject_moving":
            res.moving_objects.append(MovingObject(active_from=t, **ev.data))
        elif ev.type == "all_clear":
            ref = ev.data.get("ref_id")
            for k in res.keepouts:
                if k.zone_id == ref:
                    k.active_until = t
        else:
            # Phase 4: assistance_request etc. Unknown events must fail loudly,
            # not be silently skipped (known-failure-mode: silent fallback).
            raise ValueError(f"unknown event type {ev.type!r} at t={t}")
