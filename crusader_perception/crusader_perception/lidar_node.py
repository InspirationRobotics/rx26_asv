"""lidar_node: the MID360's one consumer.

  in:  /livox/lidar (PointCloud2), /crsd/attitude (levelling), /crsd/pose (motion
       compensation), crsd/avoidance_enable (Bool, latched)
  out: crsd/lidar_clusters      Cluster3DArray, base_link, every window (target_tracker)
       /crsd/nav/obstacle_cloud PointCloud2, base_footprint, every window (the Nav2 costmap)
       crsd/obstacle_distance   ObstacleDistance at prox_rate_hz (the autopilot's avoidance)
       /crsd/wall_range         WallRange, every sweep, only with wall_enable (Task 3 nozzle)
       /crsd/lidar/preview      PointCloud2, base_link, at most 3000 points at 2 Hz, only
                                while something subscribes (the ground station's LiDAR view)
       /crsd/lidar/health       JSON every health_period_s: where the points went

The geometry is in lidar_cluster_core, proximity_core and wall_fit_core (no ROS, tested
against invented clouds); this file is I/O.

ACCUMULATION. A 0.3 m buoy gives ~37 points per sweep at 5 m, 9 at 10 m and 2 at 20 m, so
one sweep reaches about 10 m and five about 20 m. Each kept sweep is moved into the current
body frame by the pose delta before clustering; with a stale pose the node clusters single
sweeps, since a sparser correct cloud beats a smeared one.

THE NAV CLOUD is every point of a dense-enough cluster, levelled and voxel-thinned, including
clusters too big to be objects (docks, shorelines: what the hull must not hit). Empty when
nav_cloud_enable is false (the pool switch: the costmap stays current, nothing is marked);
WITHHELD while attitude is stale (an unlevelled water gate is a guess, and a costmap that goes
non-current makes planned legs hold). Stamped at receipt: the Livox clock is not the host's.

PROXIMITY goes silent rather than repeating a map once clusters are prox_timeout_s old, and
while crsd/avoidance_enable is false (Task 3 approaches a dock on purpose); ArduPilot ages
its proximity data out on its own. It does not filter water: water_z does, once.

THE WALL is one sweep, never accumulated: at 0.5-3 m a wall gives hundreds of returns, and
the nozzle needs what it is doing now.
"""
import json
import math
import time

import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String

from crusader_msgs.msg import (Attitude, Cluster3D, Cluster3DArray, LatLonHead,
                               ObstacleDistance, WallRange)

from crusader_common import geo
from crusader_common.node_main import declare_params, on_param_set, run_node
from crusader_common.stream_cache import StreamCache
from crusader_perception import lidar_cluster_core as core
from crusader_perception import proximity_core
from crusader_perception import wall_fit_core as wf

BODY_FRAME = "base_link"
NAV_CLOUD_TOPIC = "/crsd/nav/obstacle_cloud"       # = nav2_params.yaml's STVL topic
PREVIEW_MAX_POINTS = 3000
PREVIEW_PERIOD_S = 0.5

PARAMS = {
    "cloud_topic": "/livox/lidar",
    # extrinsic, sensor -> body (docs/G2_lidar_orientation.md): one sensor, one mounting
    "lidar_sign_y": -1.0,               # raw +y is starboard
    "lidar_sign_z": -1.0,               # mounted upside down
    "lidar_x": 0.32,
    "lidar_y": 0.05,
    "lidar_z": 0.52,                    # 0.28 m above the waterline
    # sensor-frame filters
    "r_min": 0.5,                       # the mount, cabling and deck under the sensor
    "fov_deg": 180.0,                   # the hull blocks the view aft of the beam
    # levelled-frame gates
    "water_z": 0.24,                    # measured waterline above the hull datum
    "water_margin": 0.15,
    "z_ceiling": 4.0,
    "r_max": 10.0,                      # pool value: raise to 25-40 on open water
    # voxel DBSCAN
    "leaf_size": 0.15,
    "eps_0": 0.3,
    "eps_r_ref": 15.0,
    "max_rings": 2,
    "min_density_points": 6,
    "min_points": 5,
    "max_extent_m": 8.0,                # bigger is shoreline, not an object
    "accumulate_sweeps": 5,             # 10 Hz sweeps: a 0.5 s window
    "attitude_timeout_s": 1.0,
    "pose_timeout_s": 1.0,
    # the Nav2 costmap's cloud
    "nav_cloud_enable": True,           # the pool switch, see the docstring
    "nav_cloud_frame": "base_footprint",
    "nav_cloud_leaf_m": 0.1,
    # crsd/obstacle_distance: 72 sectors; the aft ones go out as no reading, never as clear
    "prox_rate_hz": 5.0,
    "prox_increment_deg": 5.0,
    "prox_min_range_m": 0.5,
    "prox_max_range_m": 40.0,
    "prox_min_half_deg": 1.0,           # a distant buoy still fills its own sector
    "prox_timeout_s": 1.0,
    # /crsd/wall_range for the Task 3 nozzle: not yet run on the boat
    "wall_enable": False,
    "wall_sector_deg": 40.0,            # half-angle ahead for the wall
    "wall_finger_sector_deg": 90.0,     # the slip fingers are beside the boat
    "wall_r_min": 0.3,
    "wall_r_max": 4.0,
    "wall_z_min_above_water": 0.03,
    "wall_z_max_above_water": 1.5,
    "wall_tol_m": 0.03,                 # RANSAC inlier distance
    "wall_iters": 200,
    "wall_max_points": 4000,
    "wall_min_inliers": 40,
    "wall_min_span_m": 0.4,
    "wall_max_angle_deg": 35.0,
    "wall_finger_depth_m": 2.5,
    "wall_finger_min_off_m": 0.25,
    "wall_finger_max_off_m": 1.5,
    "wall_finger_min_points": 15,
    "wall_finger_quantile": 0.1,
    "health_period_s": 5.0,
}

# Tunable at the bench. The extrinsic is not: a live mounting change silently moves every
# position already published.
RANGES = {
    "r_min": (0.0, 5.0), "fov_deg": (10.0, 360.0),
    "water_z": (-2.0, 2.0), "water_margin": (0.0, 2.0), "z_ceiling": (0.0, 20.0),
    "r_max": (1.0, 100.0),
    "leaf_size": (0.02, 1.0), "eps_0": (0.05, 3.0), "eps_r_ref": (1.0, 100.0),
    "max_rings": (1, 6), "min_density_points": (1, 200), "min_points": (1, 200),
    "max_extent_m": (0.1, 100.0), "accumulate_sweeps": (1, 20),
    "attitude_timeout_s": (0.05, 5.0), "pose_timeout_s": (0.05, 5.0),
    "nav_cloud_leaf_m": (0.05, 0.5),
}
DYNAMIC = tuple(RANGES) + ("nav_cloud_enable",)


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """(N, 3) float xyz from a little-endian float32 PointCloud2 with any extra fields
    (Livox adds intensity, tag, line). A stride view: read_points builds a structured
    array per call, and this runs at 10 Hz on 20k points."""
    offs = {f.name: f.offset for f in msg.fields}
    n_pts = msg.width * msg.height
    if n_pts == 0 or not all(k in offs for k in "xyz"):
        return np.empty((0, 3))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n_pts, msg.point_step)

    def col(o):
        return raw[:, o:o + 4].copy().view(np.float32).reshape(-1)

    xyz = np.stack([col(offs["x"]), col(offs["y"]), col(offs["z"])], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def xyz_to_pointcloud2(pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """(N, 3) -> packed float32 x/y/z. N = 0 is a valid cloud: it keeps a costmap current."""
    pts = np.ascontiguousarray(pts, dtype="<f4").reshape(-1, 3)
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = 1, int(pts.shape[0])
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate("xyz")]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = pts.tobytes()
    return msg


def thin(pts, limit, rng):
    """At most `limit` rows of pts, picked uniformly."""
    if pts.shape[0] <= limit:
        return pts
    return pts[rng.choice(pts.shape[0], limit, replace=False)]


def finite_or_none(v, digits):
    return None if math.isnan(v) else round(v, digits)


class LidarNode(Node):

    def __init__(self):
        super().__init__("lidar_node")
        self.p = declare_params(self, PARAMS, dynamic=DYNAMIC, ranges=RANGES)
        self.cp = core.params_from(self.p)
        self.wp = wf.params_from({k[len("wall_"):]: v for k, v in self.p.items()
                                  if k.startswith("wall_")})
        self.nav_leaf = self.p["nav_cloud_leaf_m"]

        self._att = StreamCache(self.p["attitude_timeout_s"])    # (roll, pitch)
        self._pose = StreamCache(self.p["pose_timeout_s"])       # (e, n, heading_rad)
        self._clusters = StreamCache(self.p["prox_timeout_s"])   # [(x, y, extent)]
        self._origin = None                 # first fix; only pose deltas matter here
        self._window = []                   # [(pts_body, pose)]
        self._last_stats = None
        self._n_windows = 0
        self._avoid = True
        self._prox = {"sent": 0, "suppressed": 0, "sectors_filled": 0,
                      "nearest_cm": None, "nearest_bearing_deg": None}
        self._wall = None
        self._wall_n = self._wall_valid = 0
        self._preview_t = 0.0
        self._rng = np.random.default_rng(0)

        self.cluster_pub = self.create_publisher(Cluster3DArray, "crsd/lidar_clusters", 10)
        self.nav_pub = self.create_publisher(PointCloud2, NAV_CLOUD_TOPIC,
                                             qos_profile_sensor_data)
        self.prox_pub = self.create_publisher(ObstacleDistance, "crsd/obstacle_distance", 10)
        self.wall_pub = (self.create_publisher(WallRange, "/crsd/wall_range", 10)
                         if self.p["wall_enable"] else None)
        self.preview_pub = self.create_publisher(PointCloud2, "/crsd/lidar/preview",
                                                 qos_profile_sensor_data)
        self.health_pub = self.create_publisher(String, "/crsd/lidar/health", 10)

        self.create_subscription(PointCloud2, self.p["cloud_topic"], self._on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
        self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
        # Latched: a mission that turned avoidance off and then died keeps it off when
        # this node restarts.
        self.create_subscription(
            Bool, "crsd/avoidance_enable", self._on_avoid,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0 / self.p["prox_rate_hz"], self._send_proximity)
        self.create_timer(self.p["health_period_s"], self._health)
        on_param_set(self, self._apply)

        self.get_logger().info(
            f"{self.p['cloud_topic']}: sign_y {self.cp.sign_y:+.0f} sign_z {self.cp.sign_z:+.0f}, "
            f"fov {self.cp.fov_deg:.0f}, r {self.cp.r_min:.1f}-{self.cp.r_max:.0f} m, "
            f"{self.p['accumulate_sweeps']} sweeps; nav cloud "
            f"{'on' if self.p['nav_cloud_enable'] else 'OFF (empty clouds)'}; wall range "
            f"{'on' if self.wall_pub else 'off'}")

    def _apply(self, changes):
        self.p.update(changes)
        self.cp = core.params_from(self.p)
        self.nav_leaf = self.p["nav_cloud_leaf_m"]
        self._att.timeout_s = self.p["attitude_timeout_s"]
        self._pose.timeout_s = self.p["pose_timeout_s"]
        if "accumulate_sweeps" in changes:
            del self._window[:-1]            # shrink now rather than wait it out

    # ------------------------------------------------------------------ inputs

    def _on_att(self, msg: Attitude):
        self._att.set((msg.roll, msg.pitch), time.monotonic())

    def _on_pose(self, msg: LatLonHead):
        if math.isnan(msg.heading):
            return                           # GPS yaw unresolved: not a usable fix
        if self._origin is None:
            self._origin = (msg.latitude, msg.longitude)
        e, n = geo.latlon_to_xy(msg.latitude, msg.longitude, self._origin)
        self._pose.set((e, n, math.radians(msg.heading)), time.monotonic())

    def _on_avoid(self, msg: Bool):
        want = bool(msg.data)
        if want != self._avoid:
            self.get_logger().warn(
                f"avoidance {'ENABLED' if want else 'DISABLED'} by crsd/avoidance_enable"
                + ("" if want else ": the autopilot's proximity data will age out"))
        self._avoid = want

    # ------------------------------------------------------------------ each sweep

    def _on_cloud(self, msg: PointCloud2):
        t_rx = self.get_clock().now()        # first: the nav cloud's stamp
        now = time.monotonic()
        pts = pointcloud2_to_xyz(msg)
        sweep_n = pts.shape[0]
        att = self._att.get(now)
        roll, pitch = att if att is not None else (0.0, 0.0)
        pose = self._pose.get(now)

        # Filters and the body transform run per sweep, before storage, so the window holds
        # body points that need only a pose delta to combine.
        near = core.near_mask(pts, self.cp.r_min)
        n_near = int((~near).sum())
        pts = pts[near]
        infov = core.fov_mask(pts, self.cp.fov_deg)
        n_fov = int((~infov).sum())
        body = core.to_body(pts[infov], self.cp)

        if self.wall_pub is not None:
            self._fit_wall(body, att, roll, pitch, msg.header.stamp)
        self._publish_preview(body, t_rx)

        self._window.append((body, pose))
        del self._window[:-(int(self.p["accumulate_sweeps"]) if pose is not None else 1)]
        if pose is not None and len(self._window) > 1:
            allpts = np.vstack([core.compensate(w_pts, w_pose, pose) if w_pose is not None
                                else w_pts for w_pts, w_pose in self._window])
        else:
            allpts = self._window[-1][0]

        # near/fov count THIS sweep (they run before accumulation); the rest count the window.
        st = core.ClusterStats(n_in=int(allpts.shape[0]))
        st.n_near, st.n_fov = n_near, n_fov
        nav_leaf = self.nav_leaf if self.p["nav_cloud_enable"] and att is not None else None
        clusters, st, nav_pts = core.process_body_nav(
            allpts, self.cp, roll, pitch, levelled=att is not None, st=st, nav_leaf=nav_leaf)
        st.dropped = {"sweep_points": int(sweep_n), "sweeps": len(self._window),
                      "compensated": pose is not None, "attitude": att is not None}

        out = Cluster3DArray()
        out.header.stamp = msg.header.stamp  # the newest sweep: when this is current
        out.header.frame_id = BODY_FRAME
        for c in clusters:
            m = Cluster3D()
            m.x, m.y, m.z = c.x, c.y, c.z
            m.extent_x, m.extent_y, m.extent_z = c.ex, c.ey, c.ez
            m.n_points = c.n_points
            m.range = c.range_m
            out.clusters.append(m)
        self.cluster_pub.publish(out)        # every window, empty or not
        # extent_x/y are box sizes, so the larger is the width the object can present
        self._clusters.set([(c.x, c.y, max(c.ex, c.ey)) for c in clusters], now)
        st.dropped["nav_cloud"] = self._publish_nav(nav_pts, att is not None, t_rx)
        self._last_stats = st
        self._n_windows += 1

    def _publish_nav(self, nav_pts, levelled, t_rx):
        """-> what was done, for the health JSON."""
        if not levelled:
            self.get_logger().warn("attitude stale: nav cloud withheld; the costmap goes "
                                   "non-current and planned legs hold",
                                   throttle_duration_sec=10.0)
            return "withheld"
        self.nav_pub.publish(xyz_to_pointcloud2(nav_pts, self.p["nav_cloud_frame"],
                                                t_rx.to_msg()))
        return "published" if self.p["nav_cloud_enable"] else "empty"

    def _publish_preview(self, body, t_rx):
        if self.preview_pub.get_subscription_count() == 0:
            return
        now = time.monotonic()
        if now - self._preview_t < PREVIEW_PERIOD_S:
            return
        self._preview_t = now
        self.preview_pub.publish(xyz_to_pointcloud2(
            thin(body, PREVIEW_MAX_POINTS, self._rng), BODY_FRAME, t_rx.to_msg()))

    def _fit_wall(self, body, att, roll, pitch, stamp):
        fit = wf.fit(body, self.cp.water_z, self.wp, roll, pitch, seed=self._wall_n)
        self._wall_n += 1
        self._wall_valid += fit.valid
        self._wall = (fit, att is not None)
        out = WallRange()
        out.header.stamp = stamp
        out.header.frame_id = BODY_FRAME
        out.valid = fit.valid
        out.range_m = fit.range_m
        out.angle_deg = fit.angle_deg
        out.n_inliers = fit.n_inliers
        out.rms_m = 0.0 if math.isnan(fit.rms_m) else fit.rms_m
        out.span_m = fit.span_m
        out.has_left = not math.isnan(fit.left_m)
        out.left_m = fit.left_m
        out.has_right = not math.isnan(fit.right_m)
        out.right_m = fit.right_m
        out.lat_m = fit.lat_m
        self.wall_pub.publish(out)           # every sweep, valid or not

    # ------------------------------------------------------------------ proximity

    def _send_proximity(self):
        pts = self._clusters.get(time.monotonic())
        if not self._avoid or pts is None:
            self._prox["suppressed"] += 1
            return
        inc = self.p["prox_increment_deg"]
        sectors = proximity_core.build_sectors(
            pts, fov_deg=self.cp.fov_deg, increment_deg=inc,
            min_range_m=self.p["prox_min_range_m"], max_range_m=self.p["prox_max_range_m"],
            min_half_deg=self.p["prox_min_half_deg"])
        m = ObstacleDistance()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = BODY_FRAME
        m.distances = sectors
        m.increment_deg = float(inc)
        m.angle_offset_deg = 0.0
        m.min_distance_cm = int(self.p["prox_min_range_m"] * 100.0)
        m.max_distance_cm = int(self.p["prox_max_range_m"] * 100.0)
        self.prox_pub.publish(m)
        n, cm, deg = proximity_core.summarise(sectors, inc)
        self._prox.update(sent=self._prox["sent"] + 1, sectors_filled=n, nearest_cm=cm,
                          nearest_bearing_deg=deg)

    # ------------------------------------------------------------------ health

    def _health(self):
        if self._last_stats is None:
            self.get_logger().warn(
                "no clouds yet: is the livox container publishing, on --network host? A "
                "RELIABLE subscriber never matches a BEST_EFFORT publisher.",
                throttle_duration_sec=15.0)
            return
        st = self._last_stats
        payload = dict(st.as_dict(), **st.dropped)
        payload["windows"] = self._n_windows
        payload["proximity"] = dict(self._prox, enabled=self._avoid)
        if self._wall is not None:
            f, levelled = self._wall
            payload["wall"] = dict(
                sweeps=self._wall_n, valid_sweeps=self._wall_valid, valid=f.valid, why=f.why,
                n_band=f.n_band, n_inliers=f.n_inliers, range_m=finite_or_none(f.range_m, 3),
                angle_deg=finite_or_none(f.angle_deg, 1), lat_m=finite_or_none(f.lat_m, 3),
                skipped_m=finite_or_none(f.skipped_m, 3), levelled=levelled)
        self.health_pub.publish(String(data=json.dumps(payload)))
        # A gate eating the whole cloud looks exactly like a dead sensor from downstream.
        if st.n_in and st.n_clusters == 0:
            self.get_logger().warn(
                f"0 clusters from {st.n_in} points (near={st.n_near} fov={st.n_fov} "
                f"water={st.n_water} sky={st.n_sky} far={st.n_far} noise={st.n_noise}): if "
                "one of those is the whole cloud, that gate is misconfigured",
                throttle_duration_sec=10.0)
        if not st.levelled:
            self.get_logger().warn("/crsd/attitude stale: clustering unlevelled",
                                   throttle_duration_sec=10.0)
        if not st.dropped["compensated"] and self.p["accumulate_sweeps"] > 1:
            self.get_logger().warn("/crsd/pose stale: single sweeps only (about 10 m of range)",
                                   throttle_duration_sec=10.0)
        if self._wall is not None and not self._wall[0].valid:
            self.get_logger().warn(f"no wall: {self._wall[0].why}", throttle_duration_sec=10.0)


def main(args=None):
    run_node(LidarNode, args=args)


if __name__ == "__main__":
    main()
