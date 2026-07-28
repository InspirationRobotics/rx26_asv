"""ProgressMonitor — the PREVENTATIVE onboard local-minima detector (objective 2).

Same signal definition as the evaluator's post-hoc objective2 (sustained low
speed + heading oscillation + no goal progress over a rolling window), but
incremental, so the mission planner sees "at risk" BEFORE a full stall and can
trigger an escape behavior at, or before, the moment of stagnation — not after a
long stall is confirmed (CLAUDE.md objective 2 wording).

States: OK -> AT_RISK (flag raised) -> STALLED (flag persisted past stall_after_s).
Recovery (progress resumes) returns to OK and counts an escape.

No ROS imports; unit-tested.
"""
import math
from collections import deque
from statistics import pstdev

from rx26_asv.api.common import geo


def heading_spread(headings) -> float:
    """Standard deviation of a set of headings, computed on the CIRCLE.

    A plain pstdev over raw angles breaks at the wrap: roa_apf_node feeds
    math.radians(compass_deg), i.e. 0..2*pi, so the discontinuity sits at due
    NORTH. Half a degree of jitter around north reads as ~3.13 rad of
    "oscillation" against a 0.05 rad floor, and objective-2 flags become a
    function of which way the course points.

    Unwrapping each sample against the first via wrap_pi removes the
    discontinuity. Off the wrap this is numerically identical to the old
    pstdev, so config/crusader_params.yaml thresholds (and the evaluator that
    shares them by anchor) need no retuning.
    """
    if len(headings) < 2:
        return 0.0
    ref = headings[0]
    return pstdev([ref + geo.wrap_pi(h - ref) for h in headings])


class ProgressMonitor:
    OK, AT_RISK, STALLED = "ok", "at_risk", "stalled"

    def __init__(self, window_s: float = 10.0, speed_floor: float = 0.3,
                 progress_floor: float = 0.5, heading_osc_floor: float = 0.05,
                 stall_after_s: float = 15.0, goal_radius: float = 2.0):
        self.window_s = window_s
        self.speed_floor = speed_floor
        self.progress_floor = progress_floor
        self.heading_osc_floor = heading_osc_floor
        self.stall_after_s = stall_after_s
        self.goal_radius = goal_radius
        self._buf = deque()          # (t, x, y, heading, speed, goal_dist)
        self.state = self.OK
        self.flag_start = None
        self.flags_raised = 0
        self.stalls = 0
        self.escapes = 0

    def update(self, t, x, y, heading, speed, goal_x, goal_y) -> str:
        gdist = math.hypot(goal_x - x, goal_y - y)
        self._buf.append((t, x, y, heading, speed, gdist))
        while self._buf and t - self._buf[0][0] > self.window_s:
            self._buf.popleft()

        if gdist <= self.goal_radius or len(self._buf) < 3 \
                or t - self._buf[0][0] < self.window_s * 0.9:
            return self.state        # at goal, or window not filled yet

        speeds = [s for _, _, _, _, s, _ in self._buf]
        headings = [h for _, _, _, h, _, _ in self._buf]
        progress = self._buf[0][5] - gdist
        at_risk = (sum(speeds) / len(speeds) < self.speed_floor
                   and progress < self.progress_floor
                   and heading_spread(headings) > self.heading_osc_floor)

        if at_risk:
            if self.state == self.OK:
                self.state = self.AT_RISK
                self.flag_start = t
                self.flags_raised += 1
            elif self.state == self.AT_RISK \
                    and t - self.flag_start > self.stall_after_s:
                self.state = self.STALLED
                self.stalls += 1
        else:
            if self.state in (self.AT_RISK, self.STALLED):
                self.escapes += 1
            self.state = self.OK
            self.flag_start = None
        return self.state

    def snapshot(self):
        return {"state": self.state, "flags_raised": self.flags_raised,
                "stalls": self.stalls, "escapes": self.escapes}
