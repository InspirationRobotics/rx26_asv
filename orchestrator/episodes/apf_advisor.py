"""ApfAdvisor — binds the boat's real APF core (api.navigation.apf_core) into
the episode runner, so Gate G3 exercises the SAME advisory math that runs on
the boat, not a sim-only reimplementation.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # repo root

from rx26_asv.api.navigation.apf_core import (   # noqa: E402
    ApfParams, MovingHazard, ObstaclePoint, compute)


class ApfAdvisor:
    def __init__(self, scenario, params: ApfParams = None):
        self.params = params or ApfParams()
        self.static_obstacles = [
            ObstaclePoint(o.x, o.y, o.radius, "perception")
            for o in scenario.obstacles]
        self.trap_events = 0
        self.max_repulsion = 0.0

    def advise(self, t, x, y, heading, goal_xy, keepouts, moving_objects):
        obstacles = list(self.static_obstacles)
        for k in keepouts:
            if k.active_from <= t <= k.active_until:
                obstacles.append(ObstaclePoint(k.x, k.y, k.radius, "comms"))
        moving = []
        for m in moving_objects:
            if t >= m.active_from:
                mx, my = m.position(t)
                h = math.radians(m.heading_deg)
                moving.append(MovingHazard(
                    mx, my,
                    math.sin(h) * m.speed_mps, math.cos(h) * m.speed_mps,
                    clearance=type(m).CLEARANCE_M))
        adv = compute(x, y, goal_xy[0], goal_xy[1],
                      obstacles=obstacles, moving=moving, params=self.params)
        if adv.potential_trap:
            self.trap_events += 1
        self.max_repulsion = max(self.max_repulsion, adv.repulsion)
        return adv.goal_x, adv.goal_y, adv.speed_scale
