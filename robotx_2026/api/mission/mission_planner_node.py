"""mission_planner_node — on-boat wrapper for the MissionPlanner core.

Sits ABOVE the task layer (plan §3.4): publishes the active goal that the
gate_navigator wrapper / roa_apf consume, forwards RoboCommand keep-outs to the
shared /crsd/keepouts contract (grid + fence in lockstep), and owns the
RoboCommandClient whose listener thread feeds the planner's event queue.

Topics:
  sub  /crsd/pose, /crsd/world_origin
  pub  /crsd/active_goal   interfaces/LatLonHead  (the ACTIVE task's goal)
  pub  /crsd/keepouts      interfaces/DetectionArray (world; label=zone_id,
                           radius<=0 = All Clear — same contract as bridge/grid)

Mission source: `mission_file` param — JSON {"waypoints": [[lat, lon], ...]}.
Node idles (loudly, throttled) until origin + mission are available.

TODO (explicit, not a silent gap): moving-object reports are logged and held;
the /crsd/moving_hazards topic + roa_apf subscription land when the msg carrying
velocity is added. The APF core already supports them.
"""
import json
import math
import queue

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from interfaces.msg import Detection, DetectionArray, LatLonHead

from robotx_2026.api.common import config as crsd_config
from robotx_2026.api.common import geo
from robotx_2026.api.common.node_main import run_node
from robotx_2026.api.common.param_utils import declare_from_config
from robotx_2026.api.mission.planner import MissionPlanner, PlannerConfig
from robotx_2026.api.mission.robocomms import RoboCommandClient
from robotx_2026.api.mission.tasks.waypoint_mission import WaypointMission

PARAM_SPEC = {
    "robocommand_host": dict(read_only=True),
    "robocommand_port": dict(read_only=True, lo=1, hi=65535),
    "vehicle_id": dict(read_only=True),
    "tick_rate_hz": dict(read_only=True, lo=1.0, hi=20.0),
    "mission_file": dict(read_only=True),
    "wp_radius": dict(read_only=True, lo=0.5, hi=10.0),
    "loiter_radius": dict(read_only=True, lo=1.0, hi=20.0),
    "loiter_min_s": dict(read_only=True, lo=0.0, hi=60.0),
}


class NodeSink:
    """Avoidance sink -> /crsd/keepouts publications."""

    def __init__(self, node):
        self._node = node

    def _publish(self, zone_id, x, y, radius):
        msg = DetectionArray()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.frame = "world"
        d = Detection()
        d.label = zone_id
        d.x, d.y, d.radius = float(x), float(y), float(radius)
        d.source = Detection.SOURCE_COMMS
        msg.detections.append(d)
        self._node.keepouts_pub.publish(msg)

    def keepout(self, zone_id, x, y, radius, t):
        self._publish(zone_id, x, y, radius)

    def clear(self, ref, t):
        self._publish(ref, 0.0, 0.0, -1.0)          # radius<=0 = All Clear

    def moving(self, ev, t):
        self._node.get_logger().warn(
            f"moving object {ev.object_id} received — /crsd/moving_hazards "
            "wiring pending (see node TODO)", throttle_duration_sec=5.0)


class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__("mission_planner_node")
        p = declare_from_config(self,
                                crsd_config.node_params("mission_planner_node"),
                                PARAM_SPEC)
        self._p = p
        self.origin = None
        self.pose_xy = None
        self.planner = None
        self._sim_t = 0.0

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(LatLonHead, "/crsd/active_goal", 10)
        self.keepouts_pub = self.create_publisher(DetectionArray,
                                                  "/crsd/keepouts", 10)
        self.create_subscription(LatLonHead, "/crsd/world_origin",
                                 self._origin_cb, latched)
        self.create_subscription(LatLonHead, "/crsd/pose", self._pose_cb, 10)

        self.events_q = queue.Queue()
        self.comms = RoboCommandClient(p["robocommand_host"],
                                       p["robocommand_port"], self.events_q,
                                       vehicle_id=p["vehicle_id"])
        try:
            self.comms.connect()
            self.get_logger().info("RoboCommand link up")
        except OSError as e:
            # comms may come up later at the dock; planner still runs the
            # mission — but say so loudly, a dead link is a scoring failure
            self.get_logger().error(f"RoboCommand connect failed: {e} — "
                                    "events/acks OFFLINE until restart")

        self.create_timer(1.0 / p["tick_rate_hz"], self._tick)

    # ---------- inputs ----------

    def _origin_cb(self, msg):
        self.origin = (msg.latitude, msg.longitude)
        self._maybe_start()

    def _pose_cb(self, msg):
        if self.origin is None or math.isnan(msg.heading):
            return
        self.pose_xy = geo.latlon_to_xy(msg.latitude, msg.longitude, self.origin)

    def _maybe_start(self):
        if self.planner is not None or self.origin is None:
            return
        path = self._p["mission_file"]
        if not path:
            self.get_logger().warn("no mission_file set — planner idle")
            return
        with open(path) as f:
            latlon_wps = json.load(f)["waypoints"]
        wps = [geo.latlon_to_xy(lat, lon, self.origin)
               for lat, lon in latlon_wps]
        mission = WaypointMission(wps, self._p["wp_radius"])
        self.planner = MissionPlanner(
            mission, self.comms, self.events_q,
            avoidance_sink=NodeSink(self),
            to_xy=lambda lat, lon: geo.latlon_to_xy(lat, lon, self.origin),
            config=PlannerConfig(self._p["loiter_radius"],
                                 self._p["loiter_min_s"]))
        self.get_logger().info(f"mission started: {len(wps)} waypoints")

    # ---------- main loop ----------

    def _tick(self):
        # last-line-of-defense guard (audit finding #4): an exception escaping
        # a ROS timer callback can kill the executor — a dead planner node is
        # strictly worse than one dropped tick.
        try:
            self._tick_inner()
        except Exception as e:
            self._tick_errors = getattr(self, "_tick_errors", 0) + 1
            self.get_logger().error(
                f"planner tick failed ({self._tick_errors}x): {e!r}",
                throttle_duration_sec=1.0)

    def _tick_inner(self):
        self._check_comms_health()
        if self.planner is None or self.pose_xy is None:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        goal = self.planner.tick(t, *self.pose_xy)
        self._surface_planner_diagnostics()
        if goal is None:
            self.get_logger().info("mission complete",
                                   throttle_duration_sec=10.0)
            return
        lat, lon = geo.xy_to_latlon(goal[0], goal[1], self.origin)
        msg = LatLonHead()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.latitude, msg.longitude, msg.heading = lat, lon, 0.0
        self.goal_pub.publish(msg)

    def _check_comms_health(self):
        """Dead-man's switch on the RoboCommand link (audit finding #3)."""
        if self.comms._thread is not None and not self.comms.alive:
            h = self.comms.health()
            self.get_logger().error(
                f"RoboCommand link DEAD: {h['dead_reason']} "
                f"(last rx {h['last_rx_age_s']}s ago) — acks are NOT being "
                "delivered; boat may be parked in INTERRUPT",
                throttle_duration_sec=5.0)

    def _surface_planner_diagnostics(self):
        drained = getattr(self, "_warnings_seen", 0)
        for t, msg in self.planner.warnings[drained:]:
            self.get_logger().error(f"planner warning: {msg}")
        self._warnings_seen = len(self.planner.warnings)
        errs = getattr(self, "_event_errors_seen", 0)
        if self.planner.event_errors > errs:
            self.get_logger().error(
                f"planner dropped {self.planner.event_errors - errs} bad "
                f"event(s): {self.planner.errors[-1]}")
        self._event_errors_seen = self.planner.event_errors

    def destroy_node(self):
        self.comms.close()               # Event-based stop + join
        super().destroy_node()


def main(args=None):
    run_node(MissionPlannerNode, args=args)


if __name__ == "__main__":
    main()
