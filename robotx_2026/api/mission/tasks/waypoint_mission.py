"""Task-layer building blocks (RX24 MissionHandler callback contract: fixed-rate
tick in, goal out).

TASK CONTRACT (all task types MUST honor this):
  * suspend() is a PURE, IDEMPOTENT snapshot — no side effects, safe to call
    any number of times. The planner calls it both to push resumable state and
    to VERIFY resume fidelity immediately after resume(); a suspend() that
    mutates state (flushes a buffer, decrements a counter) will corrupt the
    task the moment that verification runs.
  * resume(ctx) validates the context structurally and fails fast on
    mismatch — a bad context must error at the resume boundary with a clear
    message, not as an IndexError three ticks later.

WaypointMission — the generic transit task (gate_navigator wraps into this).
LoiterAssist — the Mission-4 Core interrupt task, with dwell HYSTERESIS:
entering within loiter_radius starts the dwell, but only drifting beyond
loiter_radius * hysteresis resets it — pose jitter straddling the boundary
must not livelock READINESS (audit finding #2).

No ROS imports; unit-tested.
"""
import math

from robotx_2026.api.mission.task_stack import TaskContext


class WaypointMission:
    NAME = "waypoint_mission"

    def __init__(self, waypoints, wp_radius: float = 2.0):
        # coerce to native floats at the boundary: numpy scalars from
        # geo.latlon_to_xy must never leak into serialized contexts
        self.waypoints = [(float(w[0]), float(w[1])) for w in waypoints]
        self.wp_radius = float(wp_radius)
        self.wp_index = 0
        self.reached = [False] * len(self.waypoints)

    def tick(self, t, x, y):
        """Returns current goal (x, y) or None when the mission is complete."""
        while self.wp_index < len(self.waypoints):
            wx, wy = self.waypoints[self.wp_index]
            if math.hypot(wx - x, wy - y) <= self.wp_radius:
                self.reached[self.wp_index] = True
                self.wp_index += 1
                continue
            return (wx, wy)
        return None

    @property
    def done(self):
        return self.wp_index >= len(self.waypoints)

    def suspend(self) -> TaskContext:
        """Pure snapshot (see TASK CONTRACT above)."""
        return TaskContext(self.NAME, {
            "wp_index": int(self.wp_index),
            "reached": [bool(r) for r in self.reached],
        })

    def resume(self, ctx: TaskContext):
        if ctx.task_name != self.NAME:
            raise ValueError(f"context is for {ctx.task_name!r}, not {self.NAME!r}")
        state = ctx.state
        wp_index = state.get("wp_index")
        reached = state.get("reached")
        # structural validation: fail fast at the resume boundary, not as an
        # IndexError deep in the resumed happy path (audit finding #7)
        if not isinstance(wp_index, int) or isinstance(wp_index, bool) \
                or not 0 <= wp_index <= len(self.waypoints):
            raise ValueError(
                f"resume rejected: wp_index={wp_index!r} invalid for a "
                f"{len(self.waypoints)}-waypoint mission")
        if not isinstance(reached, list) \
                or len(reached) != len(self.waypoints):
            raise ValueError(
                f"resume rejected: reached list length "
                f"{len(reached) if isinstance(reached, list) else 'N/A'} != "
                f"{len(self.waypoints)} waypoints (stale/mismatched context?)")
        self.wp_index = wp_index
        self.reached = [bool(r) for r in reached]


class LoiterAssist:
    """Phases: TRANSIT -> LOITER (inside loiter_radius) -> on_station after
    loiter_min_s dwell. Hysteresis: dwell resets only beyond
    loiter_radius * hysteresis, so boundary jitter cannot livelock."""
    TRANSIT, LOITER = "transit", "loiter"

    def __init__(self, point, loiter_radius: float = 3.0,
                 loiter_min_s: float = 5.0, hysteresis: float = 1.15):
        self.point = (float(point[0]), float(point[1]))
        self.loiter_radius = loiter_radius
        self.exit_radius = loiter_radius * hysteresis
        self.loiter_min_s = loiter_min_s
        self.phase = self.TRANSIT
        self._enter_t = None

    def tick(self, t, x, y):
        """Returns the assist point as the goal, always (hold station on it)."""
        d = math.hypot(self.point[0] - x, self.point[1] - y)
        if self.phase == self.TRANSIT:
            if d <= self.loiter_radius:
                self.phase = self.LOITER
                self._enter_t = t
        else:                                  # LOITER
            if d > self.exit_radius:           # hysteresis band
                self.phase = self.TRANSIT
                self._enter_t = None
        return self.point

    def on_station(self, t) -> bool:
        return (self.phase == self.LOITER
                and t - self._enter_t >= self.loiter_min_s)
