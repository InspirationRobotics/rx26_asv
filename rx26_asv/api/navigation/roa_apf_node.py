"""roa_apf_node — publishes the APF advisory (diagram's APF block, §3.2 contract).

Subscribes:
  /crsd/occupancy_grid   interfaces/Occupancy — obstacle cells (both sources)
  /crsd/pose, /crsd/world_origin — boat state
  /crsd/active_goal      interfaces/LatLonHead — the ACTIVE task's current goal,
                         published by the task layer (gate_navigator wrapper /
                         mission planner in Phase 4)
Publishes (5 Hz while a goal is active):
  /crsd/apf_advisory     interfaces/ApfAdvisory

Moving hazards (Mission 4 Disruptive): wired in Phase 4 when the RoboCommand
comms node exists to publish them — apf_core already supports them; this node
gains a /crsd/moving_hazards subscription then. Explicit TODO, not a silent gap.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from interfaces.msg import ApfAdvisory, LatLonHead, Occupancy

from rx26_asv.api.common import config as crsd_config
from rx26_asv.api.common import geo
from rx26_asv.api.common.node_main import run_node
from rx26_asv.api.common.param_utils import declare_from_config, make_set_callback
from rx26_asv.api.navigation.apf_core import ApfParams, ObstaclePoint, compute
from rx26_asv.api.navigation.occupancy_core import SOURCE_COMMS
from rx26_asv.api.navigation.progress_monitor import ProgressMonitor

PARAM_SPEC = {
    "rate_hz": dict(read_only=True, lo=1.0, hi=20.0),
    # occupied_threshold must track occupancy_grid_node's value (YAML anchor)
    "occupied_threshold": dict(read_only=False, lo=1.0, hi=100.0),
    # --- APF gains: primary ROS-side Level-1 tunables -> dynamic ---
    "apf_k_att": dict(read_only=False, lo=0.1, hi=10.0),
    "apf_k_rep": dict(read_only=False, lo=0.1, hi=50.0),
    "apf_influence_m": dict(read_only=False, lo=1.0, hi=30.0),
    "apf_lookahead_m": dict(read_only=False, lo=2.0, hi=30.0),
    "apf_slow_radius_m": dict(read_only=False, lo=0.5, hi=20.0),
    "apf_min_speed_scale": dict(read_only=False, lo=0.05, hi=1.0),
    "apf_trap_ratio": dict(read_only=False, lo=0.01, hi=0.9),
    "apf_tangent_gain": dict(read_only=False, lo=0.0, hi=5.0),
    "apf_project_s": dict(read_only=False, lo=0.0, hi=30.0),
    # --- progress monitor: stateful -> read_only (tune in YAML, restart
    # between runs; runtime changes would corrupt flag/stall accounting) ---
    "monitor_window_s": dict(read_only=True, lo=2.0, hi=60.0),
    "monitor_speed_floor": dict(read_only=True, lo=0.0, hi=2.0),
    "monitor_progress_floor": dict(read_only=True, lo=0.0, hi=10.0),
    "monitor_heading_osc_floor": dict(read_only=True, lo=0.0, hi=1.0),
    "monitor_stall_after_s": dict(read_only=True, lo=1.0, hi=120.0),
}
DYNAMIC_RANGES = {k: (v["lo"], v["hi"]) for k, v in PARAM_SPEC.items()
                  if not v["read_only"]}


class RoaApfNode(Node):
    def __init__(self):
        super().__init__("roa_apf_node")
        p = declare_from_config(self, crsd_config.node_params("roa_apf_node"),
                                PARAM_SPEC)
        self._p = p
        self.origin = None
        self.pose = None                     # (x, y, heading_rad)
        self.speed = 0.0                     # m/s, from /crsd/pose ground_speed
        self.goal = None                     # (x, y) world
        self.obstacles = []

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(ApfAdvisory, "/crsd/apf_advisory", 10)
        self.create_subscription(LatLonHead, "/crsd/world_origin",
                                 self._origin_cb, latched)
        self.create_subscription(LatLonHead, "/crsd/pose", self._pose_cb, 10)
        self.create_subscription(LatLonHead, "/crsd/active_goal",
                                 self._goal_cb, 10)
        self.create_subscription(Occupancy, "/crsd/occupancy_grid",
                                 self._grid_cb, 10)
        self.monitor = ProgressMonitor(
            window_s=p["monitor_window_s"],
            speed_floor=p["monitor_speed_floor"],
            progress_floor=p["monitor_progress_floor"],
            heading_osc_floor=p["monitor_heading_osc_floor"],
            stall_after_s=p["monitor_stall_after_s"])
        self.params = ApfParams(**{k[len("apf_"):]: v for k, v in p.items()
                                   if k.startswith("apf_")})
        self.occupied_threshold = p["occupied_threshold"]
        self.add_on_set_parameters_callback(
            make_set_callback(self, DYNAMIC_RANGES, self._apply_params))
        self.create_timer(1.0 / p["rate_hz"], self._tick)

    def _apply_params(self, changes):
        for name, value in changes.items():
            if name.startswith("apf_"):
                setattr(self.params, name[len("apf_"):], value)
            elif name == "occupied_threshold":
                self.occupied_threshold = value

    def _origin_cb(self, msg):
        self.origin = (msg.latitude, msg.longitude)

    def _pose_cb(self, msg):
        if self.origin is None or math.isnan(msg.heading):
            return
        x, y = geo.latlon_to_xy(msg.latitude, msg.longitude, self.origin)
        self.pose = (x, y, math.radians(msg.heading))
        # Objective-2's at-risk detector ANDs a speed floor with the progress
        # and heading tests. This was left at its 0.0 initial value, which
        # satisfied the floor on every tick and collapsed the detector to two
        # signals — flagging a boat at full cruise that was merely arcing around
        # an obstacle. telemetry_bridge sources this from GLOBAL_POSITION_INT.
        self.speed = msg.ground_speed

    def _goal_cb(self, msg):
        if self.origin is None:
            return
        self.goal = geo.latlon_to_xy(msg.latitude, msg.longitude, self.origin)

    def _grid_cb(self, msg: Occupancy):
        r = msg.cell_size * 0.7071
        threshold = self.occupied_threshold      # same YAML anchor as the grid
        self.obstacles = [
            ObstaclePoint((c.x_coord + 0.5) * msg.cell_size,
                          (c.y_coord + 0.5) * msg.cell_size, r,
                          "comms" if c.source == SOURCE_COMMS else "perception")
            for c in msg.grid.cells if c.value >= threshold]

    def _tick(self):
        if self.pose is None or self.goal is None or self.origin is None:
            return
        x, y, heading = self.pose
        t = self.get_clock().now().nanoseconds / 1e9
        adv = compute(x, y, self.goal[0], self.goal[1],
                      obstacles=self.obstacles, params=self.params)
        state = self.monitor.update(t, x, y, heading, self.speed,
                                    self.goal[0], self.goal[1])
        lat, lon = geo.xy_to_latlon(adv.goal_x, adv.goal_y, self.origin)
        msg = ApfAdvisory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.corrected_lat, msg.corrected_lon = lat, lon
        msg.speed_scale = float(adv.speed_scale)
        msg.repulsion = float(adv.repulsion)
        msg.potential_trap = adv.potential_trap
        msg.progress_state = state
        msg.sources = list(adv.sources)
        self.pub.publish(msg)
        if state != ProgressMonitor.OK:
            self.get_logger().warn(f"progress: {state} "
                                   f"(trap={adv.potential_trap})",
                                   throttle_duration_sec=2.0)


def main(args=None):
    run_node(RoaApfNode, args=args)


if __name__ == "__main__":
    main()
