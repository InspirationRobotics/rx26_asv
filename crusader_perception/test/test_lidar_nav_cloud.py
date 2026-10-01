"""The navigation obstacle cloud (docs/nav2_avoidance_spec.md section 6).

Core tests are numpy only, no ROS:   python crusader_perception/test/test_lidar_nav_cloud.py
The node tests need rclpy and a built crusader_msgs, and skip themselves without
them. Run those with a sourced workspace and an isolated domain:

    ROS_DOMAIN_ID=54 python3 crusader_perception/test/test_lidar_nav_cloud.py

Every cloud is built where the answer is DEFINED (the levelled frame, or BODY
axes) and pushed to the RAW SENSOR frame through the inverse of `to_body`, so the
upside-down mount (sign_y = sign_z = -1) is exercised rather than assumed: a sign
error in the extrinsic puts the buoy at the wrong y or z and fails here.
"""
import math
import os
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from crusader_perception import lidar_cluster_core as lcc   # noqa: E402

try:
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import ReliabilityPolicy
    from builtin_interfaces.msg import Time
    from crusader_msgs.msg import Attitude
    from crusader_perception import lidar_cluster_node as lcn
    HAVE_ROS = True
except Exception:                                   # no ROS, or msgs not built
    HAVE_ROS = False

WATER = 0.24                  # waterline above the hull datum (the YAML's water_z)
GATE = WATER + 0.15           # + water_margin: nothing below this is kept
P = lcc.ClusterParams(water_z=WATER, r_max=40.0)    # the other defaults ARE the YAML's
LEAF = 0.10


# ------------------------------------------------------------------ fixtures

def to_sensor(body, p=P):
    """Inverse of lidar_cluster_core.to_body: BODY -> raw (upside-down) sensor frame."""
    s = np.empty_like(body, dtype=np.float64)
    s[:, 0] = body[:, 0] - p.tx
    s[:, 1] = (body[:, 1] - p.ty) / p.sign_y
    s[:, 2] = (body[:, 2] - p.tz) / p.sign_z
    return s


def unlevel(lev, roll, pitch):
    """Inverse of lidar_cluster_core.level: levelled -> body."""
    R = lcc.level(np.eye(3), roll, pitch).T        # level(p) = R p, row-wise
    return lev @ R                                  # R^T p, row-wise


def buoy(x=8.0, y=0.4, z0=0.6, z1=1.2, r=0.15, n=90, seed=1):
    """A 0.3 m wide cylinder standing on the water, in the LEVELLED frame."""
    rng = np.random.default_rng(seed)
    a, z = rng.uniform(0, 2 * math.pi, n), rng.uniform(z0, z1, n)
    return np.column_stack([x + r * np.cos(a), y + r * np.sin(a), z])


def wall(x=8.0, half_width=10.0, z0=0.5, z1=2.0, step=0.1):
    """A flat 20 m wall across the bow: a dock face, a platform side, a shore."""
    ys, zs = np.meshgrid(np.arange(-half_width, half_width, step),
                         np.arange(z0, z1, step))
    return np.column_stack([np.full(ys.size, x), ys.ravel(), zs.ravel()])


def run(lev, roll=0.0, pitch=0.0, leaf=LEAF, p=P):
    """Levelled truth -> sensor frame -> the node's per-sweep path -> core."""
    sensor = to_sensor(unlevel(lev, roll, pitch), p)
    keep = lcc.near_mask(sensor, p.r_min)
    sensor = sensor[keep]
    keep = lcc.fov_mask(sensor, p.fov_deg)
    body = lcc.to_body(sensor[keep], p)
    return lcc.process_body_nav(body, p, roll, pitch, True, None, leaf)


# ----------------------------------------------------------------- core tests

class NavCloudCore(unittest.TestCase):

    def test_buoy_8m_ahead_lands_at_body_x_8(self):          # spec test 1, WP4 acceptance
        clusters, st, nav = run(buoy())
        self.assertEqual(len(clusters), 1)                   # the cluster output is unchanged
        self.assertGreater(nav.shape[0], 10)
        self.assertEqual(nav.dtype, np.float32)
        self.assertAlmostEqual(float(nav[:, 0].mean()), 8.0, delta=0.3)
        self.assertAlmostEqual(float(nav[:, 1].mean()), 0.4, delta=0.1)   # +y is PORT
        self.assertGreater(float(nav[:, 2].min()), GATE)     # above the water gate
        self.assertEqual(st.n_nav, nav.shape[0])
        self.assertIn("n_nav", st.as_dict())

    def test_wall_is_nav_but_not_a_cluster(self):             # spec test 2
        clusters, st, nav = run(wall())
        self.assertEqual(clusters, [])                       # > max_extent_m: not an object
        self.assertGreater(nav.shape[0], 1000)
        self.assertGreater(float(nav[:, 1].max() - nav[:, 1].min()), 19.0)
        self.assertEqual(st.n_clusters, 0)

    def test_wall_and_buoy_together(self):
        clusters, _, nav = run(np.vstack([wall(x=15.0), buoy()]))
        self.assertEqual(len(clusters), 1)                   # only the buoy is an object
        self.assertAlmostEqual(clusters[0].x, 8.0, delta=0.3)
        self.assertTrue((nav[:, 0] < 9).any() and (nav[:, 0] > 14.5).any())

    def test_sparse_noise_gives_nothing(self):                # spec test 3
        rng = np.random.default_rng(3)
        noise = rng.uniform([1, -15, 0.5], [30, 15, 3.5], (300, 3))
        clusters, st, nav = run(noise)
        self.assertEqual(clusters, [])
        self.assertEqual(nav.shape, (0, 3))
        self.assertEqual(st.n_nav, 0)

    def test_dense_water_sheet_gives_nothing(self):
        xs, ys = np.meshgrid(np.arange(1, 20, 0.1), np.arange(-5, 5, 0.1))
        sheet = np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, WATER)])
        _, st, nav = run(sheet)
        self.assertEqual(nav.shape[0], 0)
        self.assertEqual(st.n_water, sheet.shape[0])

    def test_pitched_and_rolled_points_land_at_levelled_z(self):  # spec test 4
        truth = buoy(z0=0.7, z1=1.3)
        for roll, pitch in ((0.0, math.radians(10)), (math.radians(-6), math.radians(10)),
                            (math.radians(8), math.radians(-7))):
            with self.subTest(roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch)):
                _, _, nav = run(truth, roll, pitch)
                self.assertGreater(nav.shape[0], 10)
                self.assertAlmostEqual(float(nav[:, 2].mean()), float(truth[:, 2].mean()),
                                       delta=0.08)
                self.assertGreater(float(nav[:, 2].min()), 0.7 - LEAF)
                self.assertLess(float(nav[:, 2].max()), 1.3 + LEAF)
        # the control: the same pitched cloud NOT levelled is ~1.4 m off at 8 m,
        # so the check above can fail
        body = to_sensor(unlevel(truth, 0.0, math.radians(10)))
        _, _, off = lcc.process_body_nav(lcc.to_body(body, P), P, 0.0, 0.0, True,
                                         None, LEAF)
        if off.shape[0]:
            self.assertGreater(abs(float(off[:, 2].mean()) - float(truth[:, 2].mean())), 1.0)

    def test_downsample_bounds_the_count(self):               # spec test 5
        w = wall()
        dense = np.vstack([w + np.random.default_rng(i).normal(0, 0.004, w.shape)
                           for i in range(30)])              # ~90k returns, one surface
        _, _, nav = run(dense)
        self.assertLess(nav.shape[0], 0.1 * dense.shape[0])
        # 20 m x 1.5 m at a 0.1 m leaf is ~3000 cells; x2 for returns straddling a voxel face
        self.assertLessEqual(nav.shape[0], 2 * 202 * 17)
        _, _, coarse = run(dense, leaf=0.5)
        self.assertLess(coarse.shape[0], nav.shape[0] / 4)

    def test_voxel_downsample_is_a_centroid_per_voxel(self):
        pts = np.array([[0.01, 0.01, 0.01], [0.03, 0.05, 0.07],      # one voxel
                        [0.31, 0.01, 0.01]])                        # another
        out = lcc.voxel_downsample(pts, 0.1)
        self.assertEqual(out.shape, (2, 3))
        self.assertEqual(out.dtype, np.float32)
        order = np.argsort(out[:, 0])
        np.testing.assert_allclose(out[order][0], [0.02, 0.03, 0.04], atol=1e-6)
        np.testing.assert_allclose(out[order][1], [0.31, 0.01, 0.01], atol=1e-6)
        self.assertEqual(lcc.voxel_downsample(np.empty((0, 3)), 0.1).shape, (0, 3))
        far = np.array([[5000.0, 0.0, 0.0]])                 # beyond the key space
        self.assertEqual(lcc.voxel_downsample(far, 0.1).shape, (0, 3))

    def test_dense_label_mask_applies_only_the_size_gate(self):
        labels = np.array([0, 0, 0, 0, 0, 1, 1, 1, -1, -1])
        np.testing.assert_array_equal(
            lcc.dense_label_mask(labels, 5),
            [True] * 5 + [False] * 5)
        self.assertFalse(lcc.dense_label_mask(np.full(4, -1), 1).any())

    def test_nav_leaf_none_skips_the_work_and_changes_nothing_else(self):
        sensor = to_sensor(np.vstack([buoy(), wall(x=15.0)]))
        body = lcc.to_body(sensor, P)
        c_old, st_old = lcc.process_body(body, P)            # the wrapper the viewer uses
        c_none, st_none, nav = lcc.process_body_nav(body, P)
        c_nav, st_nav, _ = lcc.process_body_nav(body, P, nav_leaf=LEAF)
        self.assertEqual(nav.shape, (0, 3))
        self.assertEqual(st_none.n_nav, 0)
        self.assertEqual(c_old, c_none)
        self.assertEqual(c_old, c_nav)
        for k in ("n_water", "n_sky", "n_far", "n_noise", "n_clustered", "n_clusters"):
            self.assertEqual(getattr(st_old, k), getattr(st_nav, k), k)


# ---------------------------------------------------------------- node tests

class _Rec:
    """Stands in for a publisher or a logger and keeps what it was handed."""

    def __init__(self):
        self.msgs, self.warns = [], []

    def publish(self, m):
        self.msgs.append(m)

    def warn(self, text, **kw):
        self.warns.append((text, kw))

    def info(self, *a, **k):
        pass

    debug = error = info


@unittest.skipUnless(HAVE_ROS, "needs rclpy and a built crusader_msgs")
class NavCloudNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.node = lcn.LidarClusterNode()
        # What the real publisher was built with, before it is swapped for a recorder
        self.qos = self.node.nav_pub.qos_profile
        self.topic = self.node.nav_pub.topic_name
        self.nav, self.clusters, self.log = _Rec(), _Rec(), _Rec()
        self.node.nav_pub, self.node.pub = self.nav, self.clusters
        self.node.get_logger = lambda: self.log
        self.node.params.r_max = 40.0

    def tearDown(self):
        self.node.destroy_node()

    def setp(self, name, kind, value):
        """`ros2 param set`, through the node's own validation callback."""
        return self.node.set_parameters([Parameter(name, kind, value)])[0]

    def attitude(self, roll=0.0, pitch=0.0):
        self.node._on_att(Attitude(roll=roll, pitch=pitch))

    def feed(self, lev, roll=0.0, pitch=0.0):
        sensor = to_sensor(unlevel(lev, roll, pitch))
        self.node._on_cloud(lcn.xyz_to_pointcloud2(sensor, "livox_frame",
                                                   Time(sec=1, nanosec=0)))

    def test_topic_qos_and_params(self):
        self.assertEqual(self.topic, "/crsd/nav/obstacle_cloud")
        self.assertEqual(self.qos.reliability, ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(self.qos.depth, 5)
        self.assertTrue(self.node.get_parameter("nav_cloud_enable").value)
        self.assertEqual(self.node.get_parameter("nav_cloud_frame").value, "base_footprint")
        self.assertAlmostEqual(self.node.get_parameter("nav_cloud_leaf_m").value, 0.10)

    def test_published_cloud_frame_stamp_and_position(self):
        self.attitude()
        before = time.time()
        self.feed(buoy())
        self.assertEqual(len(self.nav.msgs), 1)
        m = self.nav.msgs[0]
        self.assertEqual(m.header.frame_id, "base_footprint")
        stamp = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        self.assertGreaterEqual(stamp, before - 0.5)         # receipt time, NOT the driver's
        self.assertLessEqual(stamp, time.time() + 0.5)       # stamp (sec=1)
        self.assertTrue(m.is_dense)
        self.assertEqual((m.height, m.point_step), (1, 12))
        xyz = lcn.pointcloud2_to_xyz(m)
        self.assertEqual(xyz.shape[0], m.width)
        self.assertGreater(xyz.shape[0], 10)
        self.assertAlmostEqual(float(xyz[:, 0].mean()), 8.0, delta=0.3)
        self.assertGreater(float(xyz[:, 2].min()), GATE)
        self.assertEqual(len(self.clusters.msgs), 1)         # clusters still published
        self.assertEqual(len(self.clusters.msgs[0].clusters), 1)

    def test_attitude_is_applied(self):
        truth = buoy(z0=0.7, z1=1.3)
        self.attitude(math.radians(-6), math.radians(10))
        self.feed(truth, math.radians(-6), math.radians(10))
        xyz = lcn.pointcloud2_to_xyz(self.nav.msgs[0])
        self.assertAlmostEqual(float(xyz[:, 2].mean()), float(truth[:, 2].mean()), delta=0.08)

    def test_attitude_stale_withholds_cloud_and_warns(self):
        self.attitude()
        self.feed(buoy())
        self.assertEqual(len(self.nav.msgs), 1)
        # attitude stops: age the cache past attitude_timeout_s instead of sleeping
        self.node._att.set((0.0, 0.0), time.monotonic() - 100.0)
        for _ in range(3):
            self.feed(buoy())
        self.assertEqual(len(self.nav.msgs), 1)              # nothing more was published
        self.assertEqual(len(self.clusters.msgs), 4)         # clustering carried on
        self.assertEqual(len(self.log.warns), 3)             # call-site throttled, so one a call
        text, kw = self.log.warns[0]
        self.assertEqual(text, "attitude stale: nav cloud withheld; the costmap goes "
                               "non-current and planned legs hold")
        self.assertEqual(kw.get("throttle_duration_sec"), 10.0)
        self.assertEqual(self.node._last_stats.dropped["nav_cloud"], "withheld")
        self.attitude()                                      # attitude returns -> cloud returns
        self.feed(buoy())
        self.assertEqual(len(self.nav.msgs), 2)

    def test_pool_switch_publishes_empty_clouds(self):
        self.attitude()
        self.assertTrue(self.setp("nav_cloud_enable", Parameter.Type.BOOL, False).successful)
        self.feed(buoy())
        self.assertEqual(len(self.nav.msgs), 1)
        m = self.nav.msgs[0]
        self.assertEqual((m.width, len(m.data), m.header.frame_id), (0, 0, "base_footprint"))
        self.assertEqual(self.node._last_stats.dropped["nav_cloud"], "empty")
        self.assertEqual(len(self.clusters.msgs[0].clusters), 1)   # clusters unaffected
        self.setp("nav_cloud_enable", Parameter.Type.BOOL, True)
        self.feed(buoy())
        self.assertGreater(self.nav.msgs[1].width, 10)

    def test_leaf_is_range_checked_and_applied(self):
        self.assertFalse(self.setp("nav_cloud_leaf_m", Parameter.Type.DOUBLE, 0.01).successful)
        self.assertTrue(self.setp("nav_cloud_leaf_m", Parameter.Type.DOUBLE, 0.5).successful)
        self.assertEqual(self.node.nav_leaf, 0.5)
        self.assertFalse(self.setp("nav_cloud_topic", Parameter.Type.STRING, "/x").successful)

    def test_cloud_round_trip(self):
        pts = np.array([[1.0, -2.0, 0.5], [8.0, 0.25, 1.0]], dtype=np.float32)
        m = lcn.xyz_to_pointcloud2(pts, "base_footprint", Time(sec=5, nanosec=7))
        np.testing.assert_array_equal(lcn.pointcloud2_to_xyz(m), pts)
        self.assertEqual([f.name for f in m.fields], ["x", "y", "z"])
        empty = lcn.xyz_to_pointcloud2(np.empty((0, 3)), "base_footprint", Time())
        self.assertEqual((empty.width, empty.row_step, len(empty.data)), (0, 0, 0))
        self.assertEqual(lcn.pointcloud2_to_xyz(empty).shape, (0, 3))


if __name__ == "__main__":
    unittest.main()
