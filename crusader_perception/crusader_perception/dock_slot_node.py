"""dock_slot_node — the docking slip (side walls + back wall) from each LiDAR sweep.

Subscribes:
  /livox/lidar     sensor_msgs/PointCloud2  — the livox container's raw sweeps
  /crsd/attitude   crusader_msgs/Attitude   — roll/pitch, to level the cloud
Publishes:
  /crsd/dock_slot        crusader_msgs/DockSlot  (base_link, every sweep, valid or not)
  crsd/dock_slot_health  std_msgs/String (JSON)  — why not, when not

Task 3 with the pan/tilt cannon: the tree docks on this (SlotKeep holds the
LiDAR 1.1 m from the back wall, centred between the side walls, square to the
slip). The geometry is slot_fit_core (no ROS, tested on ray-cast sweeps); this
file is I/O only, the same shape as wall_range_node.

ONE SWEEP, NO ACCUMULATION, as wall_range_node: the answer is what the slip is
doing NOW. The tree filters (a median over half a second, a rate fit).

THE EXTRINSIC IS lidar_cluster_node's (its crusader_params.yaml section): one
sensor, one mounting. The LiDAR's x, y from there are also what the standoff is
measured from.
"""
import json
import math
import time

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from crusader_msgs.msg import Attitude, DockSlot

from crusader_common import config as crsd_config
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config
from crusader_common.stream_cache import StreamCache

from crusader_perception import lidar_cluster_core as lcc
from crusader_perception import slot_fit_core as sf
from crusader_perception.lidar_cluster_node import pointcloud2_to_xyz

# All [RO]: geometry, changed in the YAML and restarted.
PARAM_SPEC = {
    "cloud_topic": dict(read_only=True, description="livox PointCloud2 in"),
    "slot_topic": dict(read_only=True, description="DockSlot out"),
    "frame_id": dict(read_only=True, description="REP-103 body frame name"),
    "attitude_timeout_s": dict(read_only=True, lo=0.05, hi=5.0),
    "health_period_s": dict(read_only=True, lo=0.5, hi=60.0),
    "r_min": dict(read_only=True, lo=0.0, hi=5.0),
    "r_max": dict(read_only=True, lo=0.5, hi=20.0),
    "z_min_above_water": dict(read_only=True, lo=-0.5, hi=2.0),
    "z_max_above_water": dict(read_only=True, lo=0.0, hi=5.0),
    "side_min_off_m": dict(read_only=True, lo=0.0, hi=3.0),
    "side_max_off_m": dict(read_only=True, lo=0.1, hi=5.0),
    "side_x_min": dict(read_only=True, lo=-5.0, hi=5.0),
    "side_x_max": dict(read_only=True, lo=0.0, hi=20.0),
    "max_yaw_deg": dict(read_only=True, lo=1.0, hi=89.0),
    "parallel_deg": dict(read_only=True, lo=0.5, hi=45.0),
    "slip_width_m": dict(read_only=True, lo=0.5, hi=5.0),
    "width_tol_m": dict(read_only=True, lo=0.05, hi=3.0),
    "tol_m": dict(read_only=True, lo=0.005, hi=0.2),
    "iters": dict(read_only=True, lo=10, hi=2000),
    "max_points": dict(read_only=True, lo=100, hi=50000),
    "min_inliers": dict(read_only=True, lo=5, hi=10000),
    "min_span_m": dict(read_only=True, lo=0.05, hi=10.0),
    "back_margin_m": dict(read_only=True, lo=0.0, hi=1.0),
    "back_min_ahead_m": dict(read_only=True, lo=0.0, hi=5.0),
    "back_band_m": dict(read_only=True, lo=0.01, hi=0.5),
    "back_min_inliers": dict(read_only=True, lo=3, hi=10000),
    "back_min_span_m": dict(read_only=True, lo=0.05, hi=3.0),
}


def _f(v):
    """A float for the message: NaN stays NaN (it means "not seen")."""
    return float(v)


class DockSlotNode(Node):

    def __init__(self):
        super().__init__("dock_slot_node")
        p = declare_from_config(self, crsd_config.node_params("dock_slot_node"), PARAM_SPEC)
        self.cp = lcc.params_from(crsd_config.node_params("lidar_cluster_node"))
        self.sp = sf.params_from(p, lidar_x=self.cp.tx, lidar_y=self.cp.ty)
        self.frame_id = p["frame_id"]
        self._att = StreamCache(p["attitude_timeout_s"])
        self._last = None
        self._levelled = False
        self._n = 0
        self._n_valid = 0

        self.pub = self.create_publisher(DockSlot, p["slot_topic"], 10)
        self.health_pub = self.create_publisher(String, "crsd/dock_slot_health", 10)
        self.create_subscription(PointCloud2, p["cloud_topic"], self._on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
        self.create_timer(p["health_period_s"], self._health)
        self.get_logger().info(
            f"dock slot {p['cloud_topic']} -> {p['slot_topic']} "
            f"[{self.sp.slip_width_m:.2f} m slip, side walls {self.sp.side_min_off_m:.2f}-"
            f"{self.sp.side_max_off_m:.2f} m off, LiDAR at ({self.sp.lidar_x:.2f}, "
            f"{self.sp.lidar_y:.2f}), waterline z={self.cp.water_z:.2f}]")

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
        fit = sf.fit(body, self.cp.water_z, self.sp, roll, pitch, seed=self._n)
        self._n += 1
        self._n_valid += fit.valid
        self._last = fit

        out = DockSlot()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.valid = fit.valid
        out.why = fit.why
        out.angle_deg = _f(fit.angle_deg)
        out.lateral_m = _f(fit.lateral_m)
        out.width_m = _f(fit.width_m)
        out.has_left, out.has_right = fit.has_left, fit.has_right
        out.left_m, out.right_m = _f(fit.left_m), _f(fit.right_m)
        out.n_left, out.n_right = fit.n_left, fit.n_right
        out.has_back = fit.has_back
        out.back_range_m = _f(fit.back_range_m)
        out.n_back = fit.n_back
        out.rms_m = fit.rms_m if not math.isnan(fit.rms_m) else 0.0
        self.pub.publish(out)                    # EVERY sweep, valid or not

    def _health(self):
        if self._last is None:
            self.get_logger().warn(
                "no clouds yet: is the livox container publishing /livox/lidar, "
                "and this container on --network host?", throttle_duration_sec=15.0)
            return
        f = self._last

        def r(v, k=3):
            return None if math.isnan(v) else round(v, k)
        self.health_pub.publish(String(data=json.dumps(dict(
            sweeps=self._n, valid_sweeps=self._n_valid, valid=f.valid, why=f.why,
            n_band=f.n_band, back_range_m=r(f.back_range_m), lateral_m=r(f.lateral_m),
            angle_deg=r(f.angle_deg, 1), width_m=r(f.width_m), n_left=f.n_left,
            n_right=f.n_right, n_back=f.n_back, levelled=self._levelled))))
        if not f.valid:
            self.get_logger().warn(f"no slip: {f.why}", throttle_duration_sec=10.0)
        if not self._levelled:
            self.get_logger().warn("/crsd/attitude stale: cloud unlevelled",
                                   throttle_duration_sec=10.0)


def main(args=None):
    run_node(DockSlotNode, args=args)


if __name__ == "__main__":
    main()
