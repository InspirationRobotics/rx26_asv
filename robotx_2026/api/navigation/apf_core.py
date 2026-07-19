"""APF advisory core — the diagram's 'Artificial Potential Field' block, bound to
the §3.2 arbitration contract:

  ADVISORY ONLY. This module never commands motors and never emits setpoints.
  It returns a corrected goal + speed scale that the ACTIVE task node (the
  gate_navigator wrapper in GUIDED, a correction term inside dp_hold in
  RC-override) blends into its own output. ArduRover AVOID_* remains the hard
  backstop underneath.

Local-minima handling split (deliberate):
  * this module DETECTS the classic APF trap (repulsion ~cancels attraction) and
    applies a small deterministic tangential bias to break head-on symmetry —
    that is potential-field shaping, allowed here;
  * real ESCAPE behaviors (tangential waypoint injection, margin changes) belong
    to the mission planner, where they are visible, logged state transitions and
    the designated Level-2 injection point. Do not grow them here.

Moving hazards (Mission 4 Disruptive): repelled from BOTH current and projected
position, each with the hard 10 m clearance radius.

No ROS imports; unit-tested; also drives the Gate-G3 episode runs.
"""
import math
from dataclasses import dataclass, field


@dataclass
class ApfParams:
    k_att: float = 1.0
    k_rep: float = 5.0
    influence_m: float = 6.0        # repulsion range beyond obstacle surface
    lookahead_m: float = 8.0        # corrected-goal projection distance
    slow_radius_m: float = 4.0      # start slowing inside this surface distance
    min_speed_scale: float = 0.2
    trap_ratio: float = 0.15        # |sum| < ratio*(|att|+|rep|) -> trap
    tangent_gain: float = 0.8
    project_s: float = 5.0          # moving-hazard projection horizon


@dataclass
class ObstaclePoint:
    x: float
    y: float
    radius: float
    source: str = "perception"      # "perception" | "comms"


@dataclass
class MovingHazard:
    x: float
    y: float
    vx: float
    vy: float
    clearance: float = 10.0         # Mission 4 hard radius


@dataclass
class Advisory:
    goal_x: float
    goal_y: float
    speed_scale: float
    repulsion: float                # |repulsive vector| — 0 means "no correction"
    potential_trap: bool
    min_surface_dist: float
    sources: list = field(default_factory=list)   # which source types contributed


def _repulse_from(px, py, ox, oy, surface_radius, params):
    """Repulsive vector away from one obstacle, or None if out of influence.
    Returns (vx, vy, surface_dist)."""
    dx, dy = px - ox, py - oy
    dist = math.hypot(dx, dy)
    d = dist - surface_radius                     # distance to the surface
    if d >= params.influence_m:
        return None
    d_eff = max(d, 0.1)
    mag = params.k_rep * (1.0 / d_eff - 1.0 / params.influence_m)
    if dist < 1e-6:
        return (mag, 0.0, d)                      # degenerate: push +x
    return (mag * dx / dist, mag * dy / dist, d)


def compute(pose_x, pose_y, goal_x, goal_y, obstacles=(), moving=(),
            params: ApfParams = None) -> Advisory:
    p = params or ApfParams()

    gdx, gdy = goal_x - pose_x, goal_y - pose_y
    gdist = math.hypot(gdx, gdy)
    if gdist < 1e-6:
        return Advisory(goal_x, goal_y, 1.0, 0.0, False, math.inf)
    ax, ay = p.k_att * gdx / gdist, p.k_att * gdy / gdist

    rx = ry = 0.0
    min_d = math.inf
    sources = set()
    for o in obstacles:
        r = _repulse_from(pose_x, pose_y, o.x, o.y, o.radius, p)
        if r:
            rx += r[0]
            ry += r[1]
            min_d = min(min_d, r[2])
            sources.add(o.source)
    for m in moving:
        for (hx, hy) in ((m.x, m.y),
                         (m.x + m.vx * p.project_s, m.y + m.vy * p.project_s)):
            r = _repulse_from(pose_x, pose_y, hx, hy, m.clearance, p)
            if r:
                rx += r[0]
                ry += r[1]
                min_d = min(min_d, r[2])
                sources.add("moving")

    rep_mag = math.hypot(rx, ry)
    tx, ty = ax + rx, ay + ry
    total = math.hypot(tx, ty)
    trap = rep_mag > 1e-6 and total < p.trap_ratio * (p.k_att + rep_mag)
    if trap:
        # deterministic clockwise tangential bias: rotate repulsion by -90 deg
        tx += p.tangent_gain * ry
        ty += p.tangent_gain * -rx
        total = math.hypot(tx, ty)

    if total < 1e-6:                              # fully degenerate: hold course
        tx, ty, total = gdx, gdy, gdist

    reach = min(p.lookahead_m, gdist) if rep_mag < 1e-6 else p.lookahead_m
    cgx = pose_x + tx / total * reach
    cgy = pose_y + ty / total * reach

    if min_d is math.inf or min_d >= p.slow_radius_m:
        scale = 1.0
    else:
        scale = max(p.min_speed_scale, min_d / p.slow_radius_m)

    return Advisory(cgx, cgy, scale, rep_mag, trap, min_d, sorted(sources))
