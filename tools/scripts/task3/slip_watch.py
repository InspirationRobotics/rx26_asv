#!/usr/bin/env python3
"""slip_watch.py - /crsd/dock_slot as one plain line a second (slip.sh runs it).

    OK  back wall 1.62 m | width 1.49 m | slip centre 0.03 m LEFT | slip points 1.2 deg LEFT | walls L R back
    --  no slip: no back wall | walls L R -

back wall = LiDAR to the back wall along the slip (tape it from the LiDAR).
slip centre = where the slip's centreline is from the boat: push the boat
20 cm toward one wall and it moves 0.20 the other way. slip points = which
way the slip's axis is turned from the bow.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from crusader_msgs.msg import DockSlot


def side(v, unit, digits):
    if v != v:  # NaN
        return "?"
    word = "LEFT" if v > 0 else "RIGHT"
    return f"{abs(v):.{digits}f} {unit} {word}" if abs(v) >= 10 ** -digits else f"0 {unit}"


class SlipWatch(Node):
    def __init__(self):
        super().__init__("slip_watch")
        self.last = None
        self.n = 0
        self.create_subscription(DockSlot, "/crsd/dock_slot", self.on_slot, 10)
        self.create_timer(1.0, self.show)
        self.t0 = time.monotonic()

    def on_slot(self, m):
        self.last = m
        self.n += 1

    def show(self):
        m = self.last
        if m is None:
            if time.monotonic() - self.t0 > 3.0:
                print("waiting for /crsd/dock_slot: is the LiDAR (crsd-livox) running?", flush=True)
            return
        walls = " ".join(["L" if m.has_left else "-", "R" if m.has_right else "-",
                          "back" if m.has_back else "-"])
        if m.valid:
            print(f"OK  back wall {m.back_range_m:.2f} m | width {m.width_m:.2f} m | "
                  f"slip centre {side(m.lateral_m, 'm', 2)} | "
                  f"slip points {side(m.angle_deg, 'deg', 1)} | walls {walls}"
                  f"   ({self.n} sweeps/s)", flush=True)
        else:
            print(f"--  no slip: {m.why or 'not found'} | walls {walls}   ({self.n} sweeps/s)",
                  flush=True)
        self.n = 0


def main():
    rclpy.init()
    node = SlipWatch()
    print("the LiDAR's slip, once a second. Ctrl-C to stop.", flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl-C
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
