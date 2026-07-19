#!/usr/bin/env python3
"""g2_error_logger — live position-error measurement for the G2 bench gate.

Subscribes /crsd/detections_world + /crsd/world_origin, converts a surveyed
ground-truth buoy position (RTK lat/lon) into world XY, and prints running
error statistics per label. Run in the container while the boat and buoy are
static at surveyed positions (docs/G2_bench_procedure.md).

  ros2 run ... or: python3 g2_error_logger.py --truth-lat 32.70213 \
      --truth-lon -117.25087 --label buoy_flash_red

Pass criterion (G2): mean error < 0.5 m at 10 m range, >= 15 fps sustained
(fps from /crsd/perception_health).
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from interfaces.msg import DetectionArray, LatLonHead  # noqa: E402
from robotx_2026.api.common import geo  # noqa: E402
from robotx_2026.api.common.node_main import run_node  # noqa: E402


class G2ErrorLogger(Node):
    def __init__(self, truth_latlon, label):
        super().__init__("g2_error_logger")
        self.truth_latlon = truth_latlon
        self.label = label
        self.truth_xy = None
        self.errors = []
        self.health = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LatLonHead, "/crsd/world_origin",
                                 self._origin_cb, latched)
        self.create_subscription(DetectionArray, "/crsd/detections_world",
                                 self._det_cb, 10)
        self.create_subscription(String, "/crsd/perception_health",
                                 self._health_cb, 10)
        self.create_timer(2.0, self._report)

    def _origin_cb(self, msg):
        self.truth_xy = geo.latlon_to_xy(*self.truth_latlon,
                                         (msg.latitude, msg.longitude))
        self.get_logger().info(f"truth in world frame: "
                               f"({self.truth_xy[0]:.2f}, {self.truth_xy[1]:.2f})")

    def _det_cb(self, msg):
        if self.truth_xy is None:
            return
        for d in msg.detections:
            if self.label and d.label != self.label:
                continue
            self.errors.append(math.hypot(d.x - self.truth_xy[0],
                                          d.y - self.truth_xy[1]))

    def _health_cb(self, msg):
        self.health = json.loads(msg.data)

    def _report(self):
        if not self.errors:
            self.get_logger().info("no matching detections yet ...")
            return
        errs = self.errors[-200:]
        mean = statistics.mean(errs)
        p95 = sorted(errs)[int(0.95 * (len(errs) - 1))]
        fps = self.health.get("fps") if self.health else "?"
        verdict = "PASS" if mean < 0.5 and (isinstance(fps, (int, float))
                                            and fps >= 15) else "..."
        self.get_logger().info(
            f"n={len(self.errors)} mean={mean:.3f}m p95={p95:.3f}m "
            f"fps={fps} -> {verdict} (need mean<0.5m @ 10m, fps>=15)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--truth-lat", type=float, required=True)
    ap.add_argument("--truth-lon", type=float, required=True)
    ap.add_argument("--label", default="", help="restrict to one canonical label")
    args = ap.parse_args()
    run_node(lambda: G2ErrorLogger((args.truth_lat, args.truth_lon), args.label))


if __name__ == "__main__":
    main()
