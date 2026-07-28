"""Occupancy grid core — world-frame sparse grid with cell aging (plan §3.3).

Evolved from RX24 lidar_core/occupancy_grid.py; same sparse-cell message shape,
plus the two extensions the diagram and Mission 4 require:

  * cell aging/decay (diagram: 'cell decay rate: tau = Xs'):
    perception-source cells decay as v(t) = v0 * exp(-dt/tau) and are pruned;
  * dual ingest with source tags: SOURCE_COMMS cells (RoboCommand keep-outs)
    NEVER decay — they persist until an explicit All Clear (clear_zone), and
    they are keyed by zone_id so a specific zone can be cleared.

World-frame anchored (plan §4.7): cells never smear when the boat turns, and
keep-outs (given in GPS coords) rasterize once.

No ROS imports; unit-tested. The node wrapper handles topics and msg conversion.
"""
import math
from dataclasses import dataclass

SOURCE_PERCEPTION = 0
SOURCE_COMMS = 1

# Half the diagonal of a unit cell. Used both to decide which cells a detection
# disk touches (_disk) and as the radius each occupied cell reports
# (occupied_points), so adjacent cells overlap into a continuous barrier.
HALF_DIAG = 0.7071067811865476


@dataclass
class CellState:
    value: float                 # confidence at last update, [0, value_max]
    t: float                     # last update time
    source: int
    zone_id: str = ""            # comms cells only


class OccupancyCore:
    def __init__(self, cell_size: float = 0.5, decay_tau: float = 30.0,
                 value_max: float = 100.0, occupied_threshold: float = 50.0,
                 prune_below: float = 5.0):
        self.cell_size = cell_size
        self.decay_tau = decay_tau
        self.value_max = value_max
        self.occupied_threshold = occupied_threshold
        self.prune_below = prune_below
        self.cells = {}              # (ix, iy) -> CellState

    # ---------- indexing ----------

    def _index(self, x: float, y: float):
        return (int(math.floor(x / self.cell_size)),
                int(math.floor(y / self.cell_size)))

    def _center(self, ix: int, iy: int):
        return ((ix + 0.5) * self.cell_size, (iy + 0.5) * self.cell_size)

    def _disk(self, x: float, y: float, radius: float):
        # Inclusion test uses the cell HALF-DIAGONAL, not the half-width: a point
        # can sit up to half a diagonal from its own cell's centre, so a
        # half-width test drops small-radius detections entirely (radius <
        # 0.104 m at cell_size=0.5 could stamp ZERO cells and vanish silently).
        # Losing an obstacle is strictly worse than an over-wide one — the same
        # invariant lidar_fusion upholds on passthrough. HALF_DIAG also matches
        # the radius occupied_points() reports, so the two stay consistent.
        margin = self.cell_size * HALF_DIAG
        r_cells = max(0, int(math.ceil(radius / self.cell_size)))
        cx, cy = self._index(x, y)
        for ix in range(cx - r_cells, cx + r_cells + 1):
            for iy in range(cy - r_cells, cy + r_cells + 1):
                px, py = self._center(ix, iy)
                if math.hypot(px - x, py - y) <= radius + margin:
                    yield (ix, iy)

    # ---------- ingest ----------

    def ingest_detection(self, x, y, radius, t, confidence=1.0,
                         source=SOURCE_PERCEPTION, zone_id=""):
        """Stamp a disk of cells. Perception hits refresh value+timestamp
        (max-combine so a weak hit never erases a strong one mid-decay)."""
        value = self.value_max * max(0.0, min(1.0, confidence))
        for key in self._disk(x, y, radius):
            cur = self.cells.get(key)
            if cur is not None and cur.source == SOURCE_COMMS and source != SOURCE_COMMS:
                continue             # perception never overwrites a keep-out cell
            if cur is not None and cur.source == source:
                value_now = self.decayed_value(cur, t)
                self.cells[key] = CellState(max(value, value_now), t, source, zone_id)
            else:
                self.cells[key] = CellState(value, t, source, zone_id)

    def clear_zone(self, zone_id: str) -> int:
        """All Clear for one keep-out zone. Returns cells removed."""
        keys = [k for k, c in self.cells.items()
                if c.source == SOURCE_COMMS and c.zone_id == zone_id]
        for k in keys:
            del self.cells[k]
        return len(keys)

    # ---------- decay / query ----------

    def decayed_value(self, cell: CellState, t: float) -> float:
        if cell.source == SOURCE_COMMS or self.decay_tau <= 0:
            return cell.value        # keep-outs persist until All Clear
        dt = max(0.0, t - cell.t)
        return cell.value * math.exp(-dt / self.decay_tau)

    def prune(self, t: float) -> int:
        keys = [k for k, c in self.cells.items()
                if c.source == SOURCE_PERCEPTION
                and self.decayed_value(c, t) < self.prune_below]
        for k in keys:
            del self.cells[k]
        return len(keys)

    def occupied_points(self, t: float):
        """[(x, y, radius, source)] for cells above threshold — APF input.
        radius = half cell diagonal so adjacent cells overlap into a barrier."""
        r = self.cell_size * HALF_DIAG
        out = []
        for (ix, iy), c in self.cells.items():
            if self.decayed_value(c, t) >= self.occupied_threshold:
                x, y = self._center(ix, iy)
                out.append((x, y, r, c.source))
        return out

    # ---------- msg conversion (interfaces/Occupancy shape) ----------

    def to_msg_dict(self, t: float, origin, position_xyh):
        cells = []
        for (ix, iy), c in self.cells.items():
            v = self.decayed_value(c, t)
            if v >= self.prune_below:
                cells.append({"x_coord": ix, "y_coord": iy,
                              "value": int(round(v)), "source": c.source})
        return {
            "origin": list(origin),
            "position": list(position_xyh),
            "cell_size": self.cell_size,
            "decay_tau": self.decay_tau,
            "value_range": int(self.value_max),
            "value_zero_point": 0,
            "cells": cells,
        }
