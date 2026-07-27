"""Kinematic backend — fast headless episodes for CI and planner unit tests.

Descends from RX24 dev/simple_motion_sim. Deliberately models ArduRover
GUIDED-mode behavior on this frame: the boat TURNS toward the target then surges —
it does not strafe (CLAUDE.md: GUIDED cannot strafe on OmniX Crusader). A holonomic
mode exists for future dp_hold-style tests but GUIDED emulation is the default.

Stdlib only. Deterministic for a given seed.
"""
import math
import random


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class KinematicBackend:
    def __init__(self, cruise_speed=2.0, turn_rate_dps=60.0, accel=1.0,
                 pivot_threshold_deg=45.0, noise_std=0.0):
        # defaults mirror known-good params: CRUISE_SPEED=2.0, WP_PIVOT_RATE=60
        self.cruise_speed = cruise_speed
        self.turn_rate = math.radians(turn_rate_dps)
        self.accel = accel
        self.pivot_threshold = math.radians(pivot_threshold_deg)
        self.noise_std = noise_std
        self._rng = random.Random(0)
        self.t = 0.0
        self.x = self.y = self.heading = self.speed = 0.0
        self.target = None
        self.speed_scale = 1.0

    # --- backend contract ---

    def reset(self, scenario, seed):
        self._rng = random.Random(seed)
        self.t = 0.0
        x0, y0 = scenario.waypoints[0]
        # start 5 m "south" of the first waypoint, facing it
        self.x, self.y = x0, y0 - 5.0
        self.heading = 0.0
        self.speed = 0.0
        self.target = None

    def set_target(self, x, y):
        self.target = (x, y)

    def set_speed_scale(self, scale):
        self.speed_scale = max(0.0, min(1.0, scale))

    def step(self, dt):
        self.t += dt
        if self.target is None:
            self.speed = max(0.0, self.speed - self.accel * dt)
        else:
            tx, ty = self.target
            bearing = math.atan2(tx - self.x, ty - self.y)  # 0 = +y, CW positive
            err = _wrap(bearing - self.heading)
            # yaw-rate-limited turn
            turn = max(-self.turn_rate * dt, min(self.turn_rate * dt, err))
            self.heading = _wrap(self.heading + turn)
            if abs(err) > self.pivot_threshold:
                # pivot in place (rover GUIDED behavior; WP_PIVOT_RATE regime)
                self.speed = max(0.0, self.speed - 2 * self.accel * dt)
            else:
                target_speed = self.cruise_speed * self.speed_scale
                self.speed = (min(target_speed, self.speed + self.accel * dt)
                              if self.speed < target_speed
                              else max(target_speed, self.speed - self.accel * dt))
        if self.noise_std > 0:
            self.heading = _wrap(self.heading + self._rng.gauss(0, self.noise_std) * dt)
        self.x += math.sin(self.heading) * self.speed * dt
        self.y += math.cos(self.heading) * self.speed * dt

    def state(self):
        return (self.x, self.y, self.heading, self.speed)

    def shutdown(self):
        self.target = None
