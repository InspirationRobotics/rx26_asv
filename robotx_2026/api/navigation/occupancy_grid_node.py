"""occupancy_grid_node — thin wrapper over occupancy_core (plan §3.3).

Subscribes:
  /crsd/detections_world   DetectionArray (frame="world") — perception ingest
  /crsd/keepouts           DetectionArray (frame="world") — comms ingest
                           (label = zone_id; radius <= 0 = All Clear). Same
                           topic contract as telemetry_bridge's fence path —
                           grid and fence stay in lockstep from one publisher.
  /crsd/pose, /crsd/world_origin — boat state for the Occupancy msg header
Publishes:
  /crsd/occupancy_grid     interfaces/Occupancy at 5 Hz (decayed, pruned)
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from interfaces.msg import Cell, DetectionArray, Grid, LatLonHead, Occupancy

from ..common import config as crsd_config
from ..common import geo
from ..common.node_main import run_node
from ..common.param_utils import declare_from_config, make_set_callback
from .occupancy_core import OccupancyCore, SOURCE_COMMS, SOURCE_PERCEPTION

PARAM_SPEC = {
    "cell_size": dict(read_only=True, lo=0.1, hi=5.0,
                      description="grid geometry — restart to change"),
    "decay_tau": dict(read_only=False, lo=1.0, hi=600.0,
                      description="perception-cell decay [s]"),
    "occupied_threshold": dict(read_only=False, lo=1.0, hi=100.0,
                               description="keep equal to roa_apf_node's copy "
                                           "(YAML anchor)"),
}
DYNAMIC_RANGES = {k: (v["lo"], v["hi"]) for k, v in PARAM_SPEC.items()
                  if not v["read_only"]}


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__("occupancy_grid_node")
        p = declare_from_config(self,
                                crsd_config.node_params("occupancy_grid_node"),
                                PARAM_SPEC)
        self.core = OccupancyCore(cell_size=p["cell_size"],
                                  decay_tau=p["decay_tau"],
                                  occupied_threshold=p["occupied_threshold"])
        self.add_on_set_parameters_callback(
            make_set_callback(self, DYNAMIC_RANGES, self._apply_params))
        self.origin = None
        self.pose_xyh = (0.0, 0.0, 0.0)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Occupancy, "/crsd/occupancy_grid", 10)
        self.create_subscription(DetectionArray, "/crsd/detections_world",
                                 self._detections_cb, 10)
        self.create_subscription(DetectionArray, "/crsd/keepouts",
                                 self._keepouts_cb, 10)
        self.create_subscription(LatLonHead, "/crsd/world_origin",
                                 self._origin_cb, latched)
        self.create_subscription(LatLonHead, "/crsd/pose", self._pose_cb, 10)
        self.create_timer(0.2, self._publish)          # 5 Hz

    def _apply_params(self, changes):
        if "decay_tau" in changes:
            self.core.decay_tau = changes["decay_tau"]
        if "occupied_threshold" in changes:
            self.core.occupied_threshold = changes["occupied_threshold"]

    def _origin_cb(self, msg):
        self.origin = (msg.latitude, msg.longitude)

    def _pose_cb(self, msg):
        if self.origin is None or math.isnan(msg.heading):
            return
        x, y = geo.latlon_to_xy(msg.latitude, msg.longitude, self.origin)
        self.pose_xyh = (x, y, math.radians(msg.heading))

    def _detections_cb(self, msg: DetectionArray):
        if msg.frame != "world":
            return
        t = time.monotonic()
        for d in msg.detections:
            self.core.ingest_detection(d.x, d.y, d.radius, t,
                                       confidence=d.confidence,
                                       source=SOURCE_PERCEPTION)

    def _keepouts_cb(self, msg: DetectionArray):
        if msg.frame != "world":
            return
        t = time.monotonic()
        for d in msg.detections:
            if d.radius <= 0:
                n = self.core.clear_zone(d.label)
                self.get_logger().info(f"grid All Clear {d.label}: {n} cells")
            else:
                self.core.ingest_detection(d.x, d.y, d.radius, t,
                                           source=SOURCE_COMMS, zone_id=d.label)

    def _publish(self):
        t = time.monotonic()
        self.core.prune(t)
        d = self.core.to_msg_dict(t, self.origin or (0.0, 0.0), self.pose_xyh)
        msg = Occupancy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.origin = d["origin"]
        msg.position = [float(v) for v in d["position"]]
        msg.cell_size = float(d["cell_size"])
        msg.decay_tau = float(d["decay_tau"])
        grid = Grid()
        grid.value_range = d["value_range"]
        grid.value_zero_point = d["value_zero_point"]
        for c in d["cells"]:
            cell = Cell()
            cell.x_coord, cell.y_coord = c["x_coord"], c["y_coord"]
            cell.value, cell.source = c["value"], c["source"]
            grid.cells.append(cell)
        msg.grid = grid
        self.pub.publish(msg)


def main(args=None):
    run_node(OccupancyGridNode, args=args)


if __name__ == "__main__":
    main()
