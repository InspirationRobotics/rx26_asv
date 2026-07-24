"""MissionPlanner — the layer ABOVE the task-execution layer (plan §3.4).

Owns: which task is active, the interrupt/suspend/resume cycle, and the
RoboCommand ack discipline. Does NOT own: motor commands (task layer),
avoidance (APF/fences via the avoidance sink), or the comms socket (client).

Behavior contract (Mission 4):
  * AssistanceRequest -> ACK_RECEIPT immediately. ACK_INTENT is sent ONLY at
    the moment the planner COMMITS to servicing the request (intent is a
    promise, never a reflex): the first request commits immediately; requests
    arriving while an interrupt is already active are queued (FIFO) and each
    gets its ACK_INTENT when its service actually begins. No acknowledged
    intent is ever silently dropped.
  * Suspend pushes resumable state; READINESS once on station; resume ONLY on
    the matching Clearance -> pop, mission.resume(ctx), RESUMPTION sent; then
    the next pending request (if any) is committed and serviced.
  * KeepOutZone / AllClear / MovingObjectReport -> ack'd and forwarded to the
    avoidance sink. NEVER stack operations; the mission keeps running.
  * Event drain is non-blocking AND bounded (max_events_per_tick) — the
    planner loop can neither stall on comms nor be starved by an event storm.
  * Per-event dispatch is exception-contained: at runtime (strict=False) a bad
    event is counted + logged and the loop continues — a malformed event must
    degrade to a dropped event, never a dead vessel. Tests/harness use
    strict=True so bugs still fail loudly where loud is safe.
  * If an interrupt waits on Clearance past interrupt_warn_s, a warning is
    recorded (self.warnings) for the node to surface — the planner never
    auto-resumes without Clearance (that would be a rules violation), but it
    must never be silently parked either.

Everything is logged with sim time for scoring: event_log (inbound),
comms_log (outbound acks), transitions, resume_record. The evaluator's
mission4 sub-metrics are computed from these logs, not from planner claims.

Task-contract requirement (relied on here): task.suspend() MUST be a pure,
idempotent snapshot — it is called both to push and to verify resume fidelity.
See tasks/waypoint_mission.py.

No ROS imports; unit-tested with FakeComms; wrapped by mission_planner_node.
"""
import queue
from collections import deque
from dataclasses import dataclass

from robotx_2026.api.mission.events import (AllClear, AssistanceRequest, Clearance, KeepOutZone,
                     MovingObjectReport, StatusKind)
from robotx_2026.api.mission.task_stack import TaskStack
from robotx_2026.api.mission.tasks.waypoint_mission import LoiterAssist


@dataclass
class PlannerConfig:
    loiter_radius: float = 3.0
    loiter_min_s: float = 5.0
    interrupt_warn_s: float = 120.0     # no Clearance for this long -> warning
    max_events_per_tick: int = 50       # drain cap (event-storm guard)


class MissionPlanner:
    MISSION, INTERRUPT, DONE = "mission", "interrupt", "done"

    def __init__(self, mission, comms, event_queue: "queue.Queue",
                 avoidance_sink=None, to_xy=None, config: PlannerConfig = None,
                 strict: bool = False):
        """strict=True: event-dispatch errors raise (tests/harness).
        strict=False (runtime default): errors are counted + recorded and the
        planner keeps running."""
        self.mission = mission
        self.comms = comms
        self.events = event_queue
        self.sink = avoidance_sink
        self.to_xy = to_xy
        self.cfg = config or PlannerConfig()
        self.strict = strict
        self.stack = TaskStack()
        self.state = self.MISSION
        self.interrupt = None            # LoiterAssist
        self.active_request = None       # request_id being serviced
        self.pending_requests = deque()  # FIFO of deferred AssistanceRequests
        self._readiness_sent = False
        self._clearance = False
        self._interrupt_start_t = None
        self._interrupt_warned = False
        # scoring / diagnostics logs
        self.event_log = []              # (t, event_type, ref_id)
        self.comms_log = []              # (t, kind_name, ref_id)
        self.transitions = []            # (t, from, to, reason)
        self.resume_record = None        # {"pre_suspend":..., "post_resume":...}
        self.warnings = []               # (t, message) — node surfaces these
        self.event_errors = 0
        self.errors = []

    # ---------- helpers ----------

    def _send(self, kind: StatusKind, ref_id, t, position=None):
        self.comms.send_status(kind, ref_id, t, position)
        self.comms_log.append((t, kind.name, ref_id))

    def _transition(self, t, new_state, reason):
        self.transitions.append((t, self.state, new_state, reason))
        self.state = new_state

    def _resolve_xy(self, ev):
        if ev.x is not None and ev.y is not None:
            return ev.x, ev.y
        if ev.latitude is None or self.to_xy is None:
            raise ValueError(f"event {ev} has no resolvable position")
        return self.to_xy(ev.latitude, ev.longitude)

    def _commit_assistance(self, ev, t, x, y):
        """The commitment point: ACK_INTENT + suspend + start LoiterAssist."""
        self._send(StatusKind.ACK_INTENT, ev.request_id, t, (x, y))
        ctx = self.mission.suspend()
        self.stack.push(ctx, f"assistance {ev.request_id}")
        self.interrupt = LoiterAssist(self._resolve_xy(ev),
                                      self.cfg.loiter_radius,
                                      self.cfg.loiter_min_s)
        self.active_request = ev.request_id
        self._readiness_sent = False
        self._clearance = False
        self._interrupt_start_t = t
        self._interrupt_warned = False
        self._transition(t, self.INTERRUPT, f"suspend for {ev.request_id}")

    # ---------- event handling (non-blocking, bounded, contained) ----------

    def _dispatch(self, ev, t, x, y):
        if isinstance(ev, AssistanceRequest):
            self.event_log.append((t, "assistance_request", ev.request_id))
            self._send(StatusKind.ACK_RECEIPT, ev.request_id, t, (x, y))
            if self.interrupt is None:
                self._commit_assistance(ev, t, x, y)
            else:
                # already servicing one: defer honestly — receipt sent, intent
                # withheld until this request's service actually begins
                self.pending_requests.append(ev)
        elif isinstance(ev, Clearance):
            self.event_log.append((t, "clearance", ev.request_id))
            if ev.request_id == self.active_request:
                self._clearance = True
        elif isinstance(ev, KeepOutZone):
            self.event_log.append((t, "keep_out_zone", ev.zone_id))
            self._send(StatusKind.KEEPOUT_ACK, ev.zone_id, t, (x, y))
            if self.sink:
                kx, ky = self._resolve_xy(ev)
                self.sink.keepout(ev.zone_id, kx, ky, ev.radius, t)
        elif isinstance(ev, AllClear):
            self.event_log.append((t, "all_clear", ev.ref_id))
            self._send(StatusKind.ALLCLEAR_ACK, ev.ref_id, t, (x, y))
            if self.sink:
                self.sink.clear(ev.ref_id, t)
        elif isinstance(ev, MovingObjectReport):
            self.event_log.append((t, "moving_object", ev.object_id))
            if self.sink:
                self.sink.moving(ev, t)
        else:
            raise ValueError(f"unknown event on planner queue: {ev!r}")

    def _drain_events(self, t, x, y):
        drained = 0
        while drained < self.cfg.max_events_per_tick:
            try:
                ev = self.events.get_nowait()
            except queue.Empty:
                return
            drained += 1
            try:
                self._dispatch(ev, t, x, y)
            except Exception as e:
                if self.strict:
                    raise
                # runtime: a bad event degrades to a dropped event + loud
                # counter — never a dead planner (audit finding #4)
                self.event_errors += 1
                self.errors.append(f"t={t:.1f} event dispatch failed: {e!r}")

    # ---------- main tick ----------

    def tick(self, t, x, y):
        """Returns the current goal (x, y) or None when everything is done."""
        self._drain_events(t, x, y)

        if self.state == self.MISSION:
            goal = self.mission.tick(t, x, y)
            if goal is None:
                self._transition(t, self.DONE, "mission complete")
            return goal

        if self.state == self.INTERRUPT:
            goal = self.interrupt.tick(t, x, y)
            if self.interrupt.on_station(t) and not self._readiness_sent:
                self._send(StatusKind.READINESS, self.active_request, t, (x, y))
                self._readiness_sent = True
            if (not self._interrupt_warned
                    and t - self._interrupt_start_t > self.cfg.interrupt_warn_s):
                self._interrupt_warned = True
                self.warnings.append(
                    (t, f"interrupt {self.active_request} stalled "
                        f"{t - self._interrupt_start_t:.0f}s without Clearance "
                        "(holding station — will not auto-resume)"))
            if self._readiness_sent and self._clearance:
                ctx = self.stack.pop()
                pre = ctx.to_json()
                self.mission.resume(ctx)
                self.resume_record = {
                    "pre_suspend": pre,
                    "post_resume": self.mission.suspend().to_json(),
                }
                self._send(StatusKind.RESUMPTION, self.active_request, t, (x, y))
                self._transition(t, self.MISSION,
                                 f"resumed after {self.active_request}")
                self.interrupt = None
                self.active_request = None
                if self.pending_requests:     # service the next commitment
                    self._commit_assistance(self.pending_requests.popleft(),
                                            t, x, y)
                    return self.interrupt.tick(t, x, y)
                return self.mission.tick(t, x, y)
            return goal

        return None                       # DONE
