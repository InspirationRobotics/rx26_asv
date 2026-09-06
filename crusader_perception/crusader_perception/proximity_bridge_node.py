"""proximity_bridge — LiDAR clusters -> OBSTACLE_DISTANCE for the autopilot.

  in:  crsd/lidar_clusters      (Cluster3DArray, base_link, REP-103)
       crsd/avoidance_enable    (Bool, latched) — false stops emission entirely
  out: crsd/obstacle_distance   (ObstacleDistance) -> telemetry_bridge -> MAVLink
       crsd/proximity_health    (String, JSON)

WHY THIS EXISTS. `AVOID_ENABLE=3` and `AVOID_MARGIN=2.0` have been set on this
boat for months while `PRX1_TYPE=0` — avoidance switched on with no sensor
feeding it. param_guard even lists AVOID_ENABLE as PROTECTED ("avoidance on"),
so the stated policy and the actual configuration disagreed. This node is the
missing half.

It also earns its place as the LiDAR's functional check: if the proximity view in
Mission Planner tracks an object you walk around the boat, then the MID360, the
driver, the extrinsic signs, the levelling and the clustering are all working. No
other single test covers that chain.

DISABLING IT IS A FIRST-CLASS OPERATION, not an afterthought. Task 3 wants the
boat to approach a dock deliberately, and an avoidance layer that refuses to let
it near anything makes that impossible. Publishing false on crsd/avoidance_enable
stops emission; ArduPilot's proximity data then ages out on its own and avoidance
goes inert. No runtime parameter writes, nothing to restore afterwards, and
nothing that can be left half-applied if the mission aborts.

WHAT IT DELIBERATELY DOES NOT DO. It does not filter water. That is
lidar_cluster_node's job via water_z, and duplicating the judgement here would
mean two places to fix when it is wrong. If waves are reaching the autopilot as
obstacles, the fix is water_z (measured 0.24 m on 2026-09-05), not a second
filter here.
"""
import json
import math
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String

from crusader_msgs.msg import Cluster3DArray, ObstacleDistance

from crusader_common import config as crsd_config
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config
from crusader_common.stream_cache import StreamCache

from crusader_perception import proximity_core

PARAM_SPEC = {
    "clusters_topic": dict(read_only=True, description="Cluster3DArray input"),
    "output_topic": dict(read_only=True, description="ObstacleDistance output"),
    "send_rate_hz": dict(read_only=True, lo=1.0, hi=20.0,
                         description="how often to emit a sector map"),
    "fov_deg": dict(read_only=True, lo=10.0, hi=360.0,
                    description="forward field of view believed; the rest is "
                                "reported as no-reading, not as clear"),
    "increment_deg": dict(read_only=True, lo=1.0, hi=10.0,
                          description="angular width of one of the 72 sectors"),
    "min_range_m": dict(read_only=True, lo=0.05, hi=10.0,
                        description="below this a reading is not trusted"),
    "max_range_m": dict(read_only=True, lo=1.0, hi=100.0,
                        description="above this an obstacle is not reported"),
    "min_half_deg": dict(read_only=True, lo=0.5, hi=20.0,
                         description="floor on painted half-width so a distant "
                                     "object still fills its own sector"),
    "cluster_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                              description="s without clusters before we stop "
                                          "emitting rather than repeat"),
    "health_period_s": dict(read_only=True, lo=1.0, hi=60.0,
                            description="health log period"),
}


class ProximityBridge(Node):

    def __init__(self):
        super().__init__("proximity_bridge")
        p = declare_from_config(self,
                                crsd_config.node_params("proximity_bridge"),
                                PARAM_SPEC)
        self.p = p
        self._clusters = StreamCache(p["cluster_timeout_s"])
        self._enabled = True
        self._sent = 0
        self._suppressed = 0
        self._last_summary = (0, None, None)

        self.pub = self.create_publisher(ObstacleDistance, p["output_topic"], 10)
        self.health_pub = self.create_publisher(String, "crsd/proximity_health", 10)

        self.create_subscription(Cluster3DArray, p["clusters_topic"],
                                 self._on_clusters, 10)
        # Latched: a mission that disabled avoidance and then died must not have
        # its intent silently forgotten because this node restarted afterwards.
        self.create_subscription(
            Bool, "crsd/avoidance_enable", self._on_enable,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.create_timer(1.0 / p["send_rate_hz"], self._tick)
        self.create_timer(p["health_period_s"], self._health)
        self.get_logger().info(
            f"proximity: {p['clusters_topic']} -> {p['output_topic']} at "
            f"{p['send_rate_hz']:.0f} Hz, fov {p['fov_deg']:.0f} deg, "
            f"range {p['min_range_m']:.1f}-{p['max_range_m']:.0f} m")

    def _on_clusters(self, msg: Cluster3DArray):
        # (x, y, horizontal extent). extent_x/extent_y are AABB SIZES, not
        # corners, so the larger of the two is the width the object presents in
        # the worst case — which is the one to paint.
        pts = [(c.x, c.y, max(c.extent_x, c.extent_y)) for c in msg.clusters]
        self._clusters.set(pts, time.monotonic())

    def _on_enable(self, msg: Bool):
        want = bool(msg.data)
        if want != self._enabled:
            self.get_logger().warning(
                f"avoidance {'ENABLED' if want else 'DISABLED'} by "
                "crsd/avoidance_enable"
                + ("" if want else " — the autopilot's proximity data will age "
                                  "out and avoidance will go inert"))
        self._enabled = want

    def _tick(self):
        if not self._enabled:
            self._suppressed += 1
            return
        pts = self._clusters.get(time.monotonic())
        if pts is None:
            # Stale clusters: emit NOTHING rather than repeat the last map. A
            # repeated map is a claim that the world still looks like that, and
            # ArduPilot ages its own proximity data out correctly if we go quiet.
            self._suppressed += 1
            return

        sectors = proximity_core.build_sectors(
            pts,
            fov_deg=self.p["fov_deg"],
            increment_deg=self.p["increment_deg"],
            min_range_m=self.p["min_range_m"],
            max_range_m=self.p["max_range_m"],
            min_half_deg=self.p["min_half_deg"])

        m = ObstacleDistance()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.distances = sectors
        m.increment_deg = float(self.p["increment_deg"])
        m.angle_offset_deg = 0.0
        m.min_distance_cm = int(self.p["min_range_m"] * 100.0)
        m.max_distance_cm = int(self.p["max_range_m"] * 100.0)
        self.pub.publish(m)
        self._sent += 1
        self._last_summary = proximity_core.summarise(
            sectors, self.p["increment_deg"])

    def _health(self):
        n, nearest_cm, nearest_deg = self._last_summary
        payload = {
            "enabled": self._enabled,
            "sent": self._sent,
            "suppressed": self._suppressed,
            "sectors_filled": n,
            "nearest_cm": nearest_cm,
            "nearest_bearing_deg": nearest_deg,
            "clusters_fresh": self._clusters.fresh(time.monotonic()),
        }
        self.health_pub.publish(String(data=json.dumps(payload)))
        if not self._enabled:
            self.get_logger().info("avoidance disabled — not emitting")
        elif n == 0:
            self.get_logger().info(
                f"no obstacles in view (sent={self._sent})")
        else:
            self.get_logger().info(
                f"{n} sectors, nearest {nearest_cm/100.0:.1f} m at "
                f"{nearest_deg:.0f} deg CW (sent={self._sent})")


def main(args=None):
    run_node(ProximityBridge, args=args)


if __name__ == "__main__":
    main()
