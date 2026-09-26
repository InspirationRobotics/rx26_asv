"""nozzle_model — where a fixed nozzle's stream crosses a vertical face.

A drag-free parabola. It is NOT what the tool calibrates against (the operator
does that, shot by shot); it is the tool's first guess at where to start, and the
fake boat's hidden truth. A real stream droops and falls short of this, most of
all far from the nozzle, so every number it gives is a starting point.

Frames: x is the HORIZONTAL distance from the nozzle to the face, heights are
above the WATER. `pitch_deg` is the boat's pitch, + = bow up, which adds straight
onto the nozzle's elevation (the nozzle points forward).

Stdlib only, so the task3 sim can import it later.
"""
import math

G = 9.81


class NozzleModel:
    """A nozzle at `height_m` above the water, fixed at `elev_deg`, exit speed v."""

    def __init__(self, elev_deg, speed_mps, height_m):
        if speed_mps <= 0:
            raise ValueError("speed_mps must be positive")
        self.elev_deg = float(elev_deg)
        self.v = float(speed_mps)
        self.h = float(height_m)

    @classmethod
    def from_range(cls, range_m, elev_deg, height_m):
        """From 'it reaches range_m on the level at elev_deg' (landing at the
        nozzle's own height). range = v^2 sin(2 elev) / g."""
        s = math.sin(math.radians(2.0 * elev_deg))
        if range_m <= 0 or s <= 0:
            raise ValueError("need a positive range and 0 < elev < 90")
        return cls(elev_deg, math.sqrt(G * range_m / s), height_m)

    def _k(self, pitch_deg):
        th = math.radians(self.elev_deg + pitch_deg)
        return th, G / (2.0 * self.v ** 2 * math.cos(th) ** 2)

    def height_at(self, x, pitch_deg=0.0):
        """Stream height above the water at horizontal distance x."""
        th, k = self._k(pitch_deg)
        return self.h + x * math.tan(th) - k * x * x

    def slope_at(self, x, pitch_deg=0.0):
        """dz/dx: + on the rising (near) branch, - on the falling (far) one."""
        th, k = self._k(pitch_deg)
        return math.tan(th) - 2.0 * k * x

    def apex(self, pitch_deg=0.0):
        """(x, z) of the top of the arc."""
        th, k = self._k(pitch_deg)
        x = math.tan(th) / (2.0 * k)
        return x, self.height_at(x, pitch_deg)

    def solve_x(self, z, branch="near", pitch_deg=0.0):
        """Horizontal distance at which the stream is at height z, or None if it
        never gets that high. `branch` "near" = on the way up, "far" = coming
        down. A root behind the nozzle (x < 0) is None too."""
        th, k = self._k(pitch_deg)
        # k x^2 - tan(th) x + (z - h) = 0
        a, b, c = k, -math.tan(th), z - self.h
        disc = b * b - 4 * a * c
        if disc < 0:
            return None
        r = math.sqrt(disc)
        x = (-b - r) / (2 * a) if branch == "near" else (-b + r) / (2 * a)
        return x if x >= 0 else None


def target_height(deck_m, top_mm):
    """A window edge's height above the water: the deck's height plus the
    edge's height above the face panel's bottom (the build guide's numbers)."""
    return deck_m + top_mm / 1000.0
