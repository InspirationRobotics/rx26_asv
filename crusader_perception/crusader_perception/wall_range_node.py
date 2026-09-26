"""wall_range_node — the wall ahead and the slip fingers, from each LiDAR sweep.

Subscribes:
  /livox/lidar     sensor_msgs/PointCloud2  — the livox container's raw sweeps
  /crsd/attitude   crusader_msgs/Attitude   — roll/pitch, to pick the height band
Publishes:
  /crsd/wall_range        crusader_msgs/WallRange  (base_link, every sweep, valid or not)
  crsd/wall_range_health  std_msgs/String (JSON)   — why not, when not

The geometry is wall_fit_core (no ROS, tested against synthetic clouds). This
file is I/O only.

ONE SWEEP, NO ACCUMULATION. At 0.5-3 m a wall gives hundreds of returns per
sweep, and the answer has to be what the wall is doing NOW: squirt_cal takes the
median over the 0.5 s before each shot, and averaging over a moving boat here
would blur exactly that.

THE EXTRINSIC IS lidar_cluster_node's, read from its crusader_params.yaml
section, not copied into this node's. One sensor, one mounting: two copies would
drift, and the drift would show up as a wall range that is quietly off by the
difference (tools/lidar_view.py reads it the same way).

NOTE: not in core.launch.py. It has not run on the boat yet (README "nothing
ships until it has run on the boat"); start it by hand with the livox container
up. It has no device of its own, so that is always safe.
"""
import json
import math
import time

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from crusader_msgs.msg import Attitude, WallRange

from crusader_common import config as crsd_config
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config
from crusader_common.stream_cache import StreamCache

from crusader_perception import lidar_cluster_core as lcc
from crusader_perception import wall_fit_core as wf
from crusader_perception.lidar_cluster_node import pointcloud2_to_xyz

# All [RO]: a bench tool's geometry, changed in the YAML and restarted.
PARAM_SPEC = {
    "cloud_topic": dict(read_only=True, description="livox PointCloud2 in"),
    "wall_topic": dict(read_only=True, description="WallRange out"),
    "frame_id": dict(read_only=True, description="REP-103 body frame name"),
    "attitude_timeout_s": dict(read_only=True, lo=0.05, hi=5.0),
    "health_period_s": dict(read_only=True, lo=0.5, hi=60.0),
    "sector_deg": dict(read_only=True, lo=5.0, hi=90.0),
    "finger_sector_deg": dict(read_only=True, lo=5.0, hi=90.0),
    "r_min": dict(read_only=True, lo=0.0, hi=5.0),
    "r_max": dict(read_only=True, lo=0.5, hi=20.0),
    "z_min_above_water": dict(read_only=True, lo=-0.5, hi=2.0),
    "z_max_above_water": dict(read_only=True, lo=0.0, hi=5.0),
    "tol_m": dict(read_only=True, lo=0.005, hi=0.2),
    "iters": dict(read_only=True, lo=10, hi=2000),
    "max_points": dict(read_only=True, lo=100, hi=50000),
    "min_inliers": dict(read_only=True, lo=5, hi=10000),
    "min_span_m": dict(read_only=True, lo=0.05, hi=10.0),
    "max_angle_deg": dict(read_only=True, lo=1.0, hi=89.0),
    "finger_depth_m": dict(read_only=True, lo=0.1, hi=10.0),
    "finger_min_off_m": dict(read_only=True, lo=0.0, hi=5.0),
    "finger_max_off_m": dict(read_only=True, lo=0.1, hi=10.0),
    "finger_min_points": dict(read_only=True, lo=1, hi=10000),
    "finger_quantile": dict(read_only=True, lo=0.0, hi=0.5),
}


class WallRangeNode(Node):

    def __init__(self):
        super().__init__("wall_range_node")
        p = declare_from_config(self, crsd_config.node_params("wall_range_node"),
                                PARAM_SPEC)
        self.cp = lcc.params_from(crsd_config.node_params("lidar_cluster_node"))
        self.wp = wf.params_from(p)
        self.frame_id = p["frame_id"]
        self._att = StreamCache(p["attitude_timeout_s"])
        self._last = None
        self._levelled = False
        self._n = 0
        self._n_valid = 0

        self.pub = self.create_publisher(WallRange, p["wall_topic"], 10)
        self.health_pub = self.create_publisher(String, "crsd/wall_range_health", 10)
        self.create_subscription(PointCloud2, p["cloud_topic"], self._on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
        self.create_timer(p["health_period_s"], self._health)
        self.get_logger().info(
            f"wall range {p['cloud_topic']} -> {p['wall_topic']} "
            f"[sector {self.wp.sector_deg:.0f} deg, {self.wp.r_min:.1f}-"
            f"{self.wp.r_max:.1f} m, waterline z={self.cp.water_z:.2f}]")

    def _on_att(self, msg: Attitude):
        self._att.set((msg.roll, msg.pitch), time.monotonic())

    def _on_cloud(self, msg: PointCloud2):
        pts = pointcloud2_to_xyz(msg)
        pts = pts[lcc.near_mask(pts, self.cp.r_min)]
        pts = pts[lcc.fov_mask(pts, self.cp.fov_deg)]
        body = lcc.to_body(pts, self.cp)
        att = self._att.get(time.monotonic())
        self._levelled = att is not None
        roll, pitch = att if att is not None else (0.0, 0.0)
        fit = wf.fit(body, self.cp.water_z, self.wp, roll, pitch, seed=self._n)
        self._n += 1
        self._n_valid += fit.valid
        self._last = fit

        out = WallRange()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.valid = fit.valid
        out.range_m = fit.range_m
        out.angle_deg = fit.angle_deg
        out.n_inliers = fit.n_inliers
        out.rms_m = fit.rms_m if not math.isnan(fit.rms_m) else 0.0
        out.span_m = fit.span_m
        out.has_left = not math.isnan(fit.left_m)
        out.left_m = fit.left_m
        out.has_right = not math.isnan(fit.right_m)
        out.right_m = fit.right_m
        out.lat_m = fit.lat_m
        self.pub.publish(out)                    # EVERY sweep, valid or not

    def _health(self):
        if self._last is None:
            self.get_logger().warn(
                "no clouds yet: is the livox container publishing /livox/lidar, "
                "and this container on --network host?", throttle_duration_sec=15.0)
            return
        f = self._last
        self.health_pub.publish(String(data=json.dumps(dict(
            sweeps=self._n, valid_sweeps=self._n_valid, valid=f.valid, why=f.why,
            n_band=f.n_band, n_inliers=f.n_inliers,
            range_m=None if math.isnan(f.range_m) else round(f.range_m, 3),
            angle_deg=None if math.isnan(f.angle_deg) else round(f.angle_deg, 1),
            lat_m=None if math.isnan(f.lat_m) else round(f.lat_m, 3),
            skipped_m=None if math.isnan(f.skipped_m) else round(f.skipped_m, 3),
            levelled=self._levelled))))
        if not f.valid:
            self.get_logger().warn(f"no wall: {f.why}", throttle_duration_sec=10.0)
        if not self._levelled:
            self.get_logger().warn("/crsd/attitude stale: height band unlevelled",
                                   throttle_duration_sec=10.0)


def main(args=None):
    run_node(WallRangeNode, args=args)


if __name__ == "__main__":
    main()
