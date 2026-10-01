"""lidar_cluster_node — MID360 point cloud in, 3D object clusters out.

Subscribes:
  /livox/lidar     sensor_msgs/PointCloud2  — the livox container's raw sweeps
  /crsd/attitude   crusader_msgs/Attitude   — roll/pitch, to level the cloud
  /crsd/pose       crusader_msgs/LatLonHead — to motion-compensate accumulation
Publishes:
  crsd/lidar_clusters       crusader_msgs/Cluster3DArray  (base_link, every window)
  crsd/lidar_cluster_health std_msgs/String (JSON)        — where the points went
  /crsd/nav/obstacle_cloud  sensor_msgs/PointCloud2       (base_footprint, every
                            window) — the Nav2 costmap's LiDAR input, see below

All the geometry lives in lidar_cluster_core, which has no ROS imports and is
exercised against invented clouds. This file is I/O and parameter plumbing only,
mirroring buoy_detector.py.

WHY IT ACCUMULATES. The MID360 returns ~200k points/s over 360x59 degrees, so a
0.3 m buoy gives roughly 37 points per sweep at 5 m, 9 at 10 m and 2 at 20 m.
Measured against this clustering core, a single sweep reaches about 10 m and
five sweeps reach about 20 m. Livox's non-repetitive scan pattern means each
sweep adds genuinely new coverage rather than resampling the same lines, so
accumulation buys real range — but only if the boat's own motion is removed
first, or a 2 m/s drift smears every cluster over half a metre.

MOTION COMPENSATION, and what happens without it. Each retained sweep is stored
with the pose it was taken at, and rotated/translated into the CURRENT body
frame before the window is clustered. If pose goes stale the node falls back to
a SINGLE sweep and says so on the health topic: a sparser cloud that is correct
beats a denser one that is smeared, and silently accumulating uncompensated
would make every cluster drift with speed in a way that looks like bad
calibration.

THE NAVIGATION CLOUD (docs/nav2_avoidance_spec.md section 6). The same window that
is clustered also yields /crsd/nav/obstacle_cloud: every point that landed in a
dense-enough cluster, LEVELLED, voxel-downsampled, in `base_footprint` (REP-103
axes, no roll or pitch, origin at the hull-bottom datum). It is deliberately not
the cluster list. max_extent_m throws away docks, platforms and shorelines as
OBJECTS, and those are exactly what a hull must not hit. It is published
whenever a window is processed, EMPTY when `nav_cloud_enable` is false (the pool
switch: an empty cloud keeps the costmap current and marks nothing), and
WITHHELD while /crsd/attitude is stale, because an unlevelled water gate is a
guess and a costmap that goes non-current makes planned legs hold. Stamped with
RECEIPT time, not the driver's: TF is stamped on the host clock and the Livox
driver's clock is not proven to be it.

WHY NOT IN core.launch.py. Nothing here has run on the boat yet. It also has no
device of its own — it consumes the livox container's topic — so it is safe to
start by hand alongside the core stack.
"""
import json
import math
import time

import numpy as np

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from crusader_msgs.msg import Attitude, Cluster3D, Cluster3DArray, LatLonHead

from crusader_common import config as crsd_config
from crusader_common import geo
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config, make_set_callback
from crusader_common.stream_cache import StreamCache

from crusader_perception import lidar_cluster_core as core

# [RO] = structural/safety: change the YAML and restart. [DYN] = tunable at the
# bench with `ros2 param set`, range-validated. The extrinsic is deliberately
# [RO]: a live-tunable mounting geometry silently invalidates every position
# already published, and docs/G2 is the process for changing it.
PARAM_SPEC = {
    "cloud_topic": dict(read_only=True, description="livox PointCloud2 in"),
    "clusters_topic": dict(read_only=True, description="Cluster3DArray out"),
    "frame_id": dict(read_only=True, description="REP-103 body frame name"),
    # --- extrinsic [RO], bench-confirmed: docs/G2_lidar_orientation.md ---
    "lidar_sign_y": dict(read_only=True, lo=-1.0, hi=1.0,
                         description="raw +y is starboard -> -1"),
    "lidar_sign_z": dict(read_only=True, lo=-1.0, hi=1.0,
                         description="mounted upside down -> -1"),
    "lidar_x": dict(read_only=True, lo=-5.0, hi=5.0, description="BODY fwd+ [m]"),
    "lidar_y": dict(read_only=True, lo=-5.0, hi=5.0, description="BODY left+ [m]"),
    "lidar_z": dict(read_only=True, lo=-5.0, hi=5.0, description="BODY up+ [m]"),
    # --- sensor-frame self-return filters [DYN] ---
    "r_min": dict(read_only=False, lo=0.0, hi=5.0,
                  description="clear sphere around the SENSOR [m]"),
    "fov_deg": dict(read_only=False, lo=10.0, hi=360.0,
                    description="forward sector; hull blocks the rest"),
    # --- levelled-frame gates [DYN] ---
    "water_z": dict(read_only=False, lo=-2.0, hi=2.0,
                    description="waterline above the hull datum [m]"),
    "water_margin": dict(read_only=False, lo=0.0, hi=2.0),
    "z_ceiling": dict(read_only=False, lo=0.0, hi=20.0),
    "r_max": dict(read_only=False, lo=1.0, hi=100.0),
    # --- clustering [DYN] ---
    "leaf_size": dict(read_only=False, lo=0.02, hi=1.0),
    "eps_0": dict(read_only=False, lo=0.05, hi=3.0),
    "eps_r_ref": dict(read_only=False, lo=1.0, hi=100.0),
    "max_rings": dict(read_only=False, lo=1, hi=6),
    "min_density_points": dict(read_only=False, lo=1, hi=200),
    "min_points": dict(read_only=False, lo=1, hi=200),
    "max_extent_m": dict(read_only=False, lo=0.1, hi=100.0),
    # --- accumulation + freshness [DYN] ---
    "accumulate_sweeps": dict(read_only=False, lo=1, hi=20),
    "attitude_timeout_s": dict(read_only=False, lo=0.05, hi=5.0),
    "pose_timeout_s": dict(read_only=False, lo=0.05, hi=5.0),
    "health_period_s": dict(read_only=False, lo=0.5, hi=60.0),
    # --- navigation cloud (docs/nav2_avoidance_spec.md section 6) ---
    "nav_cloud_enable": dict(read_only=False,
                             description="False publishes EMPTY nav clouds: the "
                             "costmap stays current and nothing is marked"),
    "nav_cloud_topic": dict(read_only=True, description="PointCloud2 out"),
    "nav_cloud_frame": dict(read_only=True, description="= nav_frames_node."
                            "base_frame: levelled body axes, hull-bottom origin"),
    "nav_cloud_leaf_m": dict(read_only=False, lo=0.05, hi=0.5,
                             description="voxel edge of the nav cloud [m]"),
}
DYNAMIC_RANGES = {k: (v["lo"], v["hi"]) for k, v in PARAM_SPEC.items()
                  if not v["read_only"] and "lo" in v}


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract (N,3) xyz from a PointCloud2, tolerant of extra fields (Livox
    adds intensity/tag/line). Assumes little-endian float32 x/y/z.

    A stride view, not sensor_msgs_py.point_cloud2.read_points: that helper
    builds a structured array per call, and this runs at 10 Hz on 20k points
    beside a TensorRT engine.
    """
    offs = {f.name: f.offset for f in msg.fields}
    if not all(k in offs for k in ("x", "y", "z")):
        return np.empty((0, 3))
    n_pts = msg.width * msg.height
    if n_pts == 0:
        return np.empty((0, 3))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n_pts,
                                                                msg.point_step)

    def col(o):
        return raw[:, o:o + 4].copy().view(np.float32).reshape(-1)

    xyz = np.stack([col(offs["x"]), col(offs["y"]), col(offs["z"])], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def xyz_to_pointcloud2(pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """(N,3) -> a packed little-endian float32 x/y/z PointCloud2. N = 0 is a
    valid, empty cloud: that is what keeps a costmap's observation current."""
    pts = np.ascontiguousarray(pts, dtype="<f4").reshape(-1, 3)
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = 1, int(pts.shape[0])
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32,
                             count=1) for i, n in enumerate("xyz")]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True                     # no NaNs: the core never emits one
    msg.data = pts.tobytes()
    return msg


class LidarClusterNode(Node):

    def __init__(self):
        super().__init__("lidar_cluster_node")
        p = declare_from_config(self, crsd_config.node_params("lidar_cluster_node"),
                                PARAM_SPEC)
        self.params = core.params_from(p)
        self.accumulate = int(p["accumulate_sweeps"])
        self.frame_id = p["frame_id"]
        self.health_period_s = p["health_period_s"]
        self.nav_enable = bool(p["nav_cloud_enable"])
        self.nav_leaf = p["nav_cloud_leaf_m"]
        self.nav_frame = p["nav_cloud_frame"]

        self._att = StreamCache(p["attitude_timeout_s"])    # (roll, pitch)
        self._pose = StreamCache(p["pose_timeout_s"])       # (e, n, heading_rad)
        self._origin = None                 # first fix; only deltas matter here
        self._window = []                   # [(pts_body, pose, stamp)]
        self._last_stats = None
        self._n_windows = 0

        self.pub = self.create_publisher(Cluster3DArray, p["clusters_topic"], 10)
        self.health_pub = self.create_publisher(String,
                                                "crsd/lidar_cluster_health", 10)
        self.nav_pub = self.create_publisher(PointCloud2, p["nav_cloud_topic"],
                                             qos_profile_sensor_data)
        self.create_subscription(PointCloud2, p["cloud_topic"], self._on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
        self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
        self.create_timer(self.health_period_s, self._health)
        self.add_on_set_parameters_callback(
            make_set_callback(self, DYNAMIC_RANGES, self._apply))

        self.get_logger().info(
            f"clustering {p['cloud_topic']} -> {p['clusters_topic']} "
            f"[sign_y={self.params.sign_y:+.0f} sign_z={self.params.sign_z:+.0f} "
            f"fov={self.params.fov_deg:.0f} r_min={self.params.r_min:.2f} "
            f"accumulate={self.accumulate}]; nav cloud -> {p['nav_cloud_topic']} "
            f"[{self.nav_frame}, leaf={self.nav_leaf:.2f} m, "
            f"{'on' if self.nav_enable else 'OFF: empty clouds'}]")

    # ---------- dynamic params ----------

    def _apply(self, changes):
        for k, v in changes.items():
            if k == "accumulate_sweeps":
                self.accumulate = int(v)
                del self._window[:-1]           # shrink now, do not wait it out
            elif k == "health_period_s":
                self.health_period_s = v
            elif k == "nav_cloud_enable":
                self.nav_enable = bool(v)
            elif k == "nav_cloud_leaf_m":
                self.nav_leaf = v
            elif k == "attitude_timeout_s":
                self._att.timeout_s = v
            elif k == "pose_timeout_s":
                self._pose.timeout_s = v
            elif k in ("max_rings", "min_density_points", "min_points"):
                setattr(self.params, k, int(v))
            elif k in ("lidar_sign_y", "lidar_sign_z"):
                pass                            # [RO]; unreachable, kept explicit
            else:
                setattr(self.params, k, v)

    # ---------- inputs ----------

    def _on_att(self, msg: Attitude):
        self._att.set((msg.roll, msg.pitch), time.monotonic())

    def _on_pose(self, msg: LatLonHead):
        if math.isnan(msg.heading):
            return                              # GPS yaw unresolved; not a fix
        if self._origin is None:
            self._origin = (msg.latitude, msg.longitude)
        e, n = geo.latlon_to_xy(msg.latitude, msg.longitude, self._origin)
        self._pose.set((e, n, math.radians(msg.heading)), time.monotonic())

    # ---------- the work ----------

    def _on_cloud(self, msg: PointCloud2):
        t_rx = self.get_clock().now()           # FIRST: the nav cloud's stamp
        now = time.monotonic()
        pts = pointcloud2_to_xyz(msg)
        sweep_n = pts.shape[0]

        att = self._att.get(now)
        roll, pitch = att if att is not None else (0.0, 0.0)
        pose = self._pose.get(now)

        # Sensor-frame filters and the body transform happen per sweep, before
        # storage: the window then holds body points that only need a pose delta
        # to combine, and the expensive filtering is not redone every window.
        near = core.near_mask(pts, self.params.r_min)
        n_near = int((~near).sum())
        pts = pts[near]
        infov = core.fov_mask(pts, self.params.fov_deg)
        n_fov = int((~infov).sum())
        body = core.to_body(pts[infov], self.params)

        self._window.append((body, pose, msg.header.stamp))
        # Without a pose we cannot compensate, so we do not accumulate. A single
        # correct sweep beats a smeared stack; the health topic reports which.
        depth = self.accumulate if pose is not None else 1
        del self._window[:-depth]

        if pose is not None and len(self._window) > 1:
            merged = [core.compensate(w_pts, w_pose, pose) if w_pose is not None
                      else w_pts
                      for w_pts, w_pose, _ in self._window]
            allpts = np.vstack(merged)
        else:
            allpts = self._window[-1][0]

        # Stats mix two scopes on purpose, and the keys say which: near/fov are
        # THIS SWEEP (the filters run per sweep, before accumulation), while
        # water/sky/far/noise/clustered are the whole accumulated window. One
        # combined total would be meaningless — the window holds several sweeps
        # of already-filtered points.
        st = core.ClusterStats(n_in=int(allpts.shape[0]))
        st.n_near, st.n_fov = n_near, n_fov
        # The nav cloud is only worked out when it will be published: not in
        # the pool profile (empty cloud), and not while attitude is stale.
        nav_leaf = self.nav_leaf if self.nav_enable and att is not None else None
        clusters, st, nav_pts = core.process_body_nav(
            allpts, self.params, roll, pitch, levelled=att is not None, st=st,
            nav_leaf=nav_leaf)
        st.dropped = {"sweep_points": int(sweep_n),
                      "sweeps": len(self._window),
                      "compensated": pose is not None,
                      "attitude": att is not None}

        out = Cluster3DArray()
        out.header.stamp = msg.header.stamp     # newest sweep = when this is current
        out.header.frame_id = self.frame_id
        for c in clusters:
            m3 = Cluster3D()
            m3.x, m3.y, m3.z = c.x, c.y, c.z
            m3.extent_x, m3.extent_y, m3.extent_z = c.ex, c.ey, c.ez
            m3.n_points = c.n_points
            m3.range = c.range_m
            out.clusters.append(m3)
        self.pub.publish(out)                   # EVERY window, empty or not
        st.dropped["nav_cloud"] = self._publish_nav(nav_pts, att is not None,
                                                    t_rx)

        self._last_stats = st
        self._n_windows += 1

    def _publish_nav(self, nav_pts, levelled, t_rx):
        """Publish the navigation cloud; returns what was done, for the health JSON.

        Withheld, not empty, when attitude is stale: an empty cloud would keep
        the costmap current and say "all clear" about a water gate that could
        not be levelled. Silence is what makes it go non-current, and that is
        what makes planned legs hold. The pool profile is the opposite case on
        purpose: `nav_pts` is empty there, and an EMPTY cloud keeps the costmap
        current while marking nothing.
        """
        if not levelled:
            self.get_logger().warn(
                "attitude stale: nav cloud withheld; the costmap goes "
                "non-current and planned legs hold", throttle_duration_sec=10.0)
            return "withheld"
        self.nav_pub.publish(
            xyz_to_pointcloud2(nav_pts, self.nav_frame, t_rx.to_msg()))
        return "published" if self.nav_enable else "empty"

    # ---------- health ----------

    def _health(self):
        if self._last_stats is None:
            self.get_logger().warn(
                "no clouds yet — is the livox container publishing, and is this "
                "container on --network host? A RELIABLE subscriber matches a "
                "BEST_EFFORT publisher not at all.",
                throttle_duration_sec=15.0)
            return
        st = self._last_stats
        payload = st.as_dict()
        payload.update(st.dropped)
        payload["windows"] = self._n_windows
        self.health_pub.publish(String(data=json.dumps(payload)))
        # A stage eating the entire cloud is the failure that looks exactly like
        # a dead sensor from downstream, so it gets said out loud.
        if st.n_in and st.n_clusters == 0:
            self.get_logger().warn(
                f"0 clusters from {st.n_in} points "
                f"(near={st.n_near} fov={st.n_fov} water={st.n_water} "
                f"sky={st.n_sky} far={st.n_far} noise={st.n_noise}) — if one of "
                "those is the whole cloud, that gate is misconfigured",
                throttle_duration_sec=10.0)
        if not st.levelled:
            self.get_logger().warn(
                "/crsd/attitude stale — clustering unlevelled, so the water gate "
                "is only as good as the boat is flat", throttle_duration_sec=10.0)
        if not st.dropped.get("compensated", False) and self.accumulate > 1:
            self.get_logger().warn(
                "/crsd/pose stale — accumulation disabled, running on single "
                "sweeps (range drops to roughly 10 m)",
                throttle_duration_sec=10.0)


def main(args=None):
    run_node(LidarClusterNode, args=args)


if __name__ == "__main__":
    main()
