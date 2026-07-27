"""PlannerEpisodeRunner — episodes driven by the boat's REAL MissionPlanner
(api.mission.planner) instead of the naive waypoint script.

RoboCommand events come from the scenario file and are delivered through the
planner's event QUEUE — the same ingestion boundary the live client uses — so
ack discipline, suspend/resume, and sink routing are exercised exactly as on
the boat. (Byte-level framing is covered separately by the live loopback test
against mock_robocommand.)

The avoidance sink injects keep-outs/moving objects into the EpisodeResult, so
the evaluator scores comms-sourced clearance with the same code path as ever,
and the APF advisor sees them immediately (grid+fence lockstep, sim edition).
"""
import queue
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # repo root

from robotx_2026.api.mission.events import (AllClear, AssistanceRequest,   # noqa: E402
                                            Clearance, KeepOutZone,
                                            MovingObjectReport)
from robotx_2026.api.mission.planner import MissionPlanner, PlannerConfig  # noqa: E402
from robotx_2026.api.mission.tasks.waypoint_mission import WaypointMission  # noqa: E402

from episodes.runner import EpisodeResult, Sample                       # noqa: E402
from episodes.scenario import KeepOut, MovingObject                     # noqa: E402


class SimComms:
    """Planner comms stub for sim episodes: the planner's own comms_log (sim
    time) is the scoring source; this just satisfies the send contract."""

    def send_status(self, kind, ref_id, t, position=None):
        pass


class EpisodeSink:
    """Avoidance sink -> EpisodeResult, mirroring the on-boat keepouts topic."""

    def __init__(self, result: EpisodeResult):
        self.result = result

    def keepout(self, zone_id, x, y, radius, t):
        self.result.keepouts.append(
            KeepOut(x=x, y=y, radius=radius, zone_id=zone_id, active_from=t))

    def clear(self, ref, t):
        for k in self.result.keepouts:
            if k.zone_id == ref:
                k.active_until = t

    def moving(self, ev, t):
        self.result.moving_objects.append(MovingObject(
            x0=ev.x, y0=ev.y, heading_deg=ev.heading_deg,
            speed_mps=ev.speed_mps, object_id=ev.object_id,
            system_type=ev.system_type, active_from=t))


def _event_to_msg(ev):
    d = ev.data
    if ev.type == "assistance_request":
        return AssistanceRequest(**d)
    if ev.type == "clearance":
        return Clearance(**d)
    if ev.type == "inject_keepout":
        return KeepOutZone(zone_id=d["zone_id"], x=d["x"], y=d["y"],
                           radius=d["radius"])
    if ev.type == "all_clear":
        return AllClear(**d)
    if ev.type == "inject_moving":
        return MovingObjectReport(
            object_id=d["object_id"], x=d["x0"], y=d["y0"],
            heading_deg=d["heading_deg"], speed_mps=d["speed_mps"],
            system_type=d.get("system_type", "usv"))
    raise ValueError(f"unknown scenario event type {ev.type!r}")


class PlannerEpisodeRunner:
    def __init__(self, scenario, backend, seed=0, dt=0.1, advisor=None,
                 loiter_radius=3.0, loiter_min_s=5.0, stop_event=None):
        self.scenario = scenario
        self.backend = backend
        self.seed = seed
        self.dt = dt
        self.advisor = advisor
        self.cfg = PlannerConfig(loiter_radius, loiter_min_s)
        self.stop_event = stop_event or threading.Event()

    def run(self):
        """Returns (EpisodeResult, MissionPlanner) — planner logs feed
        metrics.score_mission4."""
        import time as _time
        sc = self.scenario
        res = EpisodeResult(sc.name, sc.version, self.seed)
        wall_start = _time.time()
        self.backend.reset(sc, self.seed)

        mission = WaypointMission(sc.waypoints, sc.wp_radius)
        events_q = queue.Queue()
        planner = MissionPlanner(mission, SimComms(), events_q,
                                 avoidance_sink=EpisodeSink(res),
                                 config=self.cfg,
                                 strict=True)   # harness bugs must fail loudly
        pending = sorted(sc.events, key=lambda e: e.t)
        t = 0.0
        try:
            while t < sc.timeout_s and not self.stop_event.is_set():
                while pending and pending[0].t <= t:
                    events_q.put(_event_to_msg(pending.pop(0)))
                x, y, hdg, spd = self.backend.state()
                goal = planner.tick(t, x, y)
                if goal is None:
                    break                                  # mission complete
                if self.advisor is not None:
                    gx, gy, scale = self.advisor.advise(
                        t, x, y, hdg, goal,
                        res.keepouts, res.moving_objects)
                    self.backend.set_target(gx, gy)
                    if hasattr(self.backend, "set_speed_scale"):
                        self.backend.set_speed_scale(scale)
                else:
                    self.backend.set_target(*goal)
                self.backend.step(self.dt)
                t += self.dt
                x, y, hdg, spd = self.backend.state()
                # record the PLANNER goal (not the advisor-shaped one): a
                # commanded loiter must read as at-goal, not as a stall
                res.trace.append(Sample(t, x, y, hdg, spd, goal[0], goal[1]))
            res.stopped_early = self.stop_event.is_set()
            res.timed_out = t >= sc.timeout_s
            res.waypoints_reached = list(mission.reached)
        finally:
            self.backend.shutdown()
            res.wall_clock_s = _time.time() - wall_start
        return res, planner
