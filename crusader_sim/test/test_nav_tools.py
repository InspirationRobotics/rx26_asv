"""Offline tests for the sim nav test drivers: nav_goal, nav_checks (its pure parts) and sitl_param_set.

No ROS and no running sim. Needs python3 with PyYAML and pymavlink (the WSL host has both):

    cd ~/robotx_ws/src/rx26_asv/crusader_sim && python3 -m unittest discover -s test -v

The SITL parameter test talks to a fake autopilot of its own on a private loopback port (never
5760-5763, 14550 or 14551), so it cannot reach a running sim.
"""
import math
import os
import socket
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace as NS

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(os.path.dirname(PKG), "crusader_nav"))       # frames_core, pure

from crusader_sim import course as C                                           # noqa: E402
from crusader_sim import nav_checks as K                                       # noqa: E402
from crusader_sim import nav_goal as G                                         # noqa: E402
from crusader_sim.task1_judge import Task1Judge                                # noqa: E402


def run_path(judge, points, yaw=0.0, step=0.05):
    """Feed the judge a straight path a -> b (ground truth, with yaw), as odometry would."""
    (ax, ay), (bx, by) = points
    n = max(1, round(math.hypot(bx - ax, by - ay) / step))
    for i in range(n + 1):
        judge.update(ax + (bx - ax) * i / n, ay + (by - ay) * i / n, yaw, t=i * step)


class NavGoalTest(unittest.TestCase):
    def setUp(self):
        self.course = C.load("open_water_platform")
        self.judge = Task1Judge(self.course, echo=False)

    def test_goal_is_the_specs_40_m_east_point(self):
        lat, lon = G.goal_latlon(self.course, 40.0, 0.0)
        self.assertAlmostEqual(lat, 1.28060, places=9)
        self.assertAlmostEqual(lon, 103.8560594, places=7)       # docs/nav2_avoidance_spec.md 10.2 S6

    def test_goal_round_trips_through_course_scale(self):
        o = self.course["origin"]
        for east, north in ((40.0, 0.0), (0.0, 25.0), (-13.5, 7.25), (52.0, -9.0)):
            lat, lon = G.goal_latlon(self.course, east, north)
            e = (lon - o["lon"]) * C.EARTH_M_PER_DEG * math.cos(math.radians(o["lat"]))
            n = (lat - o["lat"]) * C.EARTH_M_PER_DEG
            self.assertAlmostEqual(e, east, places=6)
            self.assertAlmostEqual(n, north, places=6)

    def test_boats_own_projection_sees_the_same_point_0_11_percent_short(self):
        # bt_runner reads the goal with frames_core's R = 6371 km, SITL's scale is 111318.845 m/deg:
        # a known property of the sim (course.enu_to_latlon), here pinned so nobody "fixes" it by accident
        from crusader_nav import frames_core as fc
        o = self.course["origin"]
        lat, lon = G.goal_latlon(self.course, 40.0, 0.0)
        east_bt, north_bt = fc.to_local(lat, lon, o["lat"], o["lon"])
        self.assertAlmostEqual(east_bt / 40.0, 0.9989, places=4)
        self.assertAlmostEqual(north_bt, 0.0, places=6)

    def test_clearance_of_a_path_two_metres_beside_the_platform(self):
        run_path(self.judge, ((0.0, 2.0), (40.0, 2.0)))
        v = self.judge.verdict()
        cl = G.watched_clearance(v, ["plat"])
        self.assertEqual(cl["plat"], (1.0, 0.7))        # 2.0 off its centre - 1.0 half-width; minus the hull's 0.3
        self.assertEqual(G.clear_line("plat", *cl["plat"]), "[clear] plat centre-to-surface 1.00 m (hull 0.70 m)")
        self.assertEqual(G.touching(v, cl), [])
        self.assertAlmostEqual(G.final_distance(self.judge, 40.0, 0.0), 2.0, places=9)
        self.assertEqual(G.score(True, 2.0, 3.0, cl, 0.73, []), (True, []))

    def test_a_path_through_the_platform_is_contact_and_a_fail(self):
        run_path(self.judge, ((0.0, 0.0), (40.0, 0.0)))
        v = self.judge.verdict()
        cl = G.watched_clearance(v, ["plat"])
        self.assertEqual(cl["plat"][0], 0.0)
        self.assertEqual(G.touching(v, cl), ["plat"])
        ok, why = G.score(True, 0.5, 3.0, cl, 0.73, G.touching(v, cl))
        self.assertFalse(ok)
        self.assertTrue(any("touched plat" in w for w in why))
        self.assertTrue(any("below 0.73" in w for w in why))

    def test_score_criteria(self):
        cl = {"plat": (1.0, 0.7)}
        self.assertFalse(G.score(False, 1.0, 3.0, cl, None, [])[0])             # tree failed
        self.assertFalse(G.score(True, 3.5, 3.0, cl, None, [])[0])              # stopped short
        self.assertFalse(G.score(True, None, 3.0, cl, None, [])[0])             # no pose at all
        self.assertTrue(G.score(True, 1.0, 3.0, {"plat": (0.1, 0.0)}, None, [])[0])   # clearance only counts when asked
        self.assertFalse(G.score(True, 1.0, 3.0, {"plat": (0.72, 0.4)}, 0.73, [])[0])
        self.assertTrue(G.score(True, 1.0, 3.0, {"plat": (0.73, 0.4)}, 0.73, [])[0])
        self.assertFalse(G.score(True, 1.0, 3.0, {"plat": (None, None)}, 0.73, [])[0])

    def test_no_pose_prints_blanks_not_numbers(self):
        v = self.judge.verdict()
        cl = G.watched_clearance(v, ["plat"])
        self.assertEqual(G.clear_line("plat", *cl["plat"]), "[clear] plat centre-to-surface n/a m (hull n/a m)")
        self.assertIsNone(G.final_distance(self.judge, 40.0, 0.0))
        self.assertEqual(G.touching(v, cl), [])

    def test_watch_names_are_checked_against_the_judge(self):
        self.assertEqual(G.unknown_watch(self.judge, ["plat", "ref_red", "nope"]), ["nope"])

    def test_arguments(self):
        a = G.parse_args(["--course", "open_water_platform", "--east-m", "40", "--watch", "plat", "ref_red"])
        self.assertEqual((a.north_m, a.timeout_s, a.arrive_m, a.min_clear_m), (0.0, 300.0, 3.0, None))
        self.assertEqual(a.watch, ["plat", "ref_red"])


class FakeCloud:
    """The parts of sensor_msgs/PointCloud2 cloud_xyz reads."""
    def __init__(self, pts, big=False):
        import struct
        fmt = ">ffff" if big else "<ffff"
        self.data = b"".join(struct.pack(fmt, x, y, z, 1.0) for x, y, z in pts)
        self.fields = [NS(name=n, offset=4 * i, datatype=7) for i, n in enumerate("xyz")] + \
                      [NS(name="intensity", offset=12, datatype=7)]
        self.point_step, self.width, self.height, self.is_bigendian = 16, len(pts), 1, big


class NavChecksTest(unittest.TestCase):
    def test_cloud_xyz_skips_non_finite_points(self):
        pts = [(1.0, 2.0, 3.0), (float("nan"), 0.0, 0.0), (4.0, 5.0, 6.0), (0.0, float("inf"), 0.0)]
        self.assertEqual(K.cloud_xyz(FakeCloud(pts)), [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
        self.assertEqual(K.cloud_xyz(FakeCloud(pts[:1], big=True)), [(1.0, 2.0, 3.0)])

    def test_cloud_without_xyz_is_refused(self):
        bad = FakeCloud([(0.0, 0.0, 0.0)])
        bad.fields = [bad.fields[0], bad.fields[1]]
        with self.assertRaises(ValueError):
            K.cloud_xyz(bad)

    def test_body_to_world(self):
        x, y = K.body_to_world((2.0, 0.0), (10.0, 5.0, math.pi / 2))
        self.assertAlmostEqual(x, 10.0)
        self.assertAlmostEqual(y, 7.0)
        x, y = K.body_to_world((0.0, 1.0), (10.0, 5.0, math.pi / 2))      # 1 m to the left of a boat heading north
        self.assertAlmostEqual(x, 9.0)
        self.assertAlmostEqual(y, 5.0)

    def test_yaw_of_quat(self):
        self.assertAlmostEqual(K.yaw_of_quat(0.0, 0.0, math.sin(0.4), math.cos(0.4)), 0.8)

    def test_points_near_the_platform(self):
        plat = Task1Judge(C.load("open_water_platform"), echo=False).shapes["plat"]      # 2 m square at (20, 0)
        pts = [(19.5, 0.5), (21.5, 0.0), (23.0, 0.0), (20.0, -1.65), (20.0, 1.7)]
        self.assertEqual(K.count_near_rects(pts, plat, 0.6), 2)      # inside, and 0.5 m off; the others are out
        self.assertEqual(K.count_near_rects(pts, plat, 0.6 + 1e-9), 2)
        self.assertEqual(K.count_near_rects([], plat, 0.6), 0)

    def test_lethal_count(self):
        w = h = 10
        cells = [0] * (w * h)
        cells[5 * w + 5] = 100                  # centre (0.55, 0.55) at res 0.1 from the origin
        cells[5 * w + 6] = 99                   # inscribed, not lethal
        args = (cells, w, h, 0.1, 0.0, 0.0)
        self.assertEqual(K.lethal_count(*args, 0.5, 0.5, 0.3), 1)
        self.assertEqual(K.lethal_count(*args, 0.65, 0.55, 0.3), 1)      # still counts the 100, not the 99
        self.assertEqual(K.lethal_count(*args, 0.2, 0.2, 0.3), 0)
        self.assertEqual(K.lethal_count(*args, -5.0, -5.0, 1.0), 0)       # outside the grid: nothing, no error
        self.assertEqual(K.lethal_count(*args, 50.0, 0.5, 1.0), 0)
        self.assertEqual(K.lethal_count(*args, 0.5, 0.5, 5.0), 1)         # a window bigger than the grid

    def test_hazards_near(self):
        hz = [(10.0, 0.0, 0.3), (30.0, 5.0, 0.5)]
        self.assertEqual(K.hazards_near(hz, 11.0, 0.0, 0.8), 1)           # 1.0 <= 0.8 + 0.3
        self.assertEqual(K.hazards_near(hz, 11.2, 0.0, 0.8), 0)
        self.assertEqual(K.hazards_near([], 0.0, 0.0, 5.0), 0)

    def test_tf_error_against_frames_core(self):
        from crusader_nav import frames_core as fc
        datum = (1.2806, 103.8557)
        lat, lon = fc.to_latlon(10.0, 5.0, *datum)
        self.assertAlmostEqual(K.tf_error_m((10.0, 5.0), lat, lon, datum), 0.0, places=6)
        self.assertAlmostEqual(K.tf_error_m((10.03, 5.04), lat, lon, datum), 0.05, places=6)

    def test_n2_verdicts(self):
        self.assertEqual(K.score_n2([0.01, 0.02] * 20, 0, 0.05, 20)[0], "PASS")
        self.assertEqual(K.score_n2([0.01] * 39 + [0.0501], 0, 0.05, 20)[0], "FAIL")
        verdict, detail = K.score_n2([0.0] * 5, 12, 0.05, 20)
        self.assertEqual(verdict, "FAIL")
        self.assertIn("fewer than 20 samples", detail)
        self.assertEqual(K.score_n2([], 0, 0.05, 0)[0], "FAIL")           # never a TypeError on an empty list

    # ---- N3
    @staticmethod
    def series(t_rm, in_zero, out_zero, hz=0):
        """n3-watch lines: samples every 0.5 s from 6 s before the removal to 60 s after; entry
        reads 0 from in_zero s after the removal and g1_grn from out_zero s after it."""
        lines = []
        for i in range(-12, 121):
            t = t_rm + i * 0.5
            e = 14 if t < t_rm + in_zero else 0
            g = 9 if t < t_rm + out_zero else 0
            lines.append(f"[n3] t={t:.3f} entry={e} g1_grn={g} hz={hz}")
        return "\n".join(["[n3] targets entry@(12,3) g1_grn@(22,3) radius 1 m"] + lines + ["[n3] done"])

    def test_n3_twin_arguments(self):
        w = K.parse_args(["n3-watch", "--course", "task1_core", "--in", "entry", "--out", "g1_grn", "--ns", "n3"])
        self.assertEqual((w.ns, w.in_name, w.out_name), ("n3", "entry", "g1_grn"))
        self.assertEqual(K.parse_args(["n3-watch", "--course", "c", "--in", "a", "--out", "b"]).ns, "")
        t = K.parse_args(["twin-params", "--ns", "n3", "--out", "/tmp/x.yaml"])
        self.assertEqual((t.ns, t.out, t.run), ("n3", "/tmp/x.yaml", K.run_twin_params))
        # the twin's watcher line must not look like a sample to the scorer
        self.assertEqual(K.parse_n3("[n3] costmap /n3/global_costmap/costmap (STVL only)"), [])

    def test_n3_parse_and_decay_times(self):
        s = K.parse_n3(self.series(1000.0, 2.5, 30.0))
        self.assertEqual(s[0], (994.0, {"entry": 14, "g1_grn": 9, "hz": 0}))
        persist, gone = K.decay_times(s, "entry", 1000.0)
        self.assertAlmostEqual(persist, 2.0)             # last sample with it: t = 1002.0
        self.assertAlmostEqual(gone, 2.5)                # first without it: 1002.5
        persist, gone = K.decay_times(s, "g1_grn", 1000.0)
        self.assertAlmostEqual(persist, 29.5)            # last sample with it: 1029.5
        self.assertAlmostEqual(gone, 30.0)
        self.assertEqual(K.decay_times(s, "entry", 5000.0), (None, None))
        self.assertEqual(K.decay_times(s, "nope", 1000.0), (None, None))

    def test_n3_passes_when_stvl_decays_as_specified(self):
        removed = {"entry": 1000.0, "g1_grn": 1000.0}
        v, d = K.score_n3(K.parse_n3(self.series(1000.0, 2.5, 30.0)), removed, "entry", "g1_grn")
        self.assertEqual(v, "PASS", d)

    def test_n3_fails(self):
        removed = {"entry": 1000.0, "g1_grn": 1000.0}
        for what, text in (("in-view too slow", self.series(1000.0, 7.0, 30.0)),
                           ("out-of-view cleared early", self.series(1000.0, 2.5, 9.0)),
                           ("out-of-view too late", self.series(1000.0, 2.5, 39.0)),
                           ("out-of-view never gone", self.series(1000.0, 2.5, 999.0))):
            v, d = K.score_n3(K.parse_n3(text), removed, "entry", "g1_grn")
            self.assertEqual(v, "FAIL", f"{what}: {d}")

    def test_n3_limits_are_inclusive(self):
        both = {"entry": 1000.0, "g1_grn": 1000.0}
        score = lambda i, o: K.score_n3(K.parse_n3(self.series(1000.0, i, o)), both, "entry", "g1_grn")[0]
        self.assertEqual(score(5.0, 30.0), "PASS")       # gone at exactly 5 s
        self.assertEqual(score(5.5, 30.0), "FAIL")
        self.assertEqual(score(2.5, 20.5), "PASS")       # persisted 20.0 s (last sample 1020.0), gone at 20.5
        self.assertEqual(score(2.5, 20.0), "FAIL")       # persisted 19.5 s
        self.assertEqual(score(2.5, 35.0), "PASS")
        self.assertEqual(score(2.5, 35.5), "FAIL")

    def test_n3_partial_invalid_and_not_testable(self):
        s = K.parse_n3(self.series(1000.0, 2.5, 30.0))
        self.assertEqual(K.score_n3(s, {"entry": 1000.0}, "entry", "g1_grn")[0], "PARTIAL")     # out never marked
        self.assertEqual(K.score_n3(s, {}, "entry", "g1_grn")[0], "NOT TESTABLE")
        hz = K.parse_n3(self.series(1000.0, 2.5, 30.0, hz=1))
        self.assertEqual(K.score_n3(hz, {"entry": 1000.0, "g1_grn": 1000.0}, "entry", "g1_grn")[0], "INVALID")
        # a hazard that was gone well before the removal does not invalidate it
        early = K.parse_n3(self.series(1000.0, 2.5, 30.0))
        early = [(t, dict(c, hz=1 if t < 995.0 else 0)) for t, c in early]
        self.assertEqual(K.score_n3(early, {"entry": 1000.0, "g1_grn": 1000.0}, "entry", "g1_grn")[0], "PASS")

    def test_score_n3_cli_reads_a_log(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "n3.log")
            with open(path, "w") as f:
                f.write(self.series(1000.0, 2.5, 30.0))
            args = K.parse_args(["score-n3", "--log", path, "--in", "entry", "--out", "g1_grn",
                                 "--removed", "entry=1000.0", "g1_grn=1000.0"])
            self.assertEqual(K.main(["score-n3", "--log", path, "--in", "entry", "--out", "g1_grn",
                                     "--removed", "entry=1000.0", "g1_grn=1000.0"]), 0)
            self.assertEqual(args.removed, ["entry=1000.0", "g1_grn=1000.0"])
            self.assertEqual(K.main(["score-n3", "--log", path, "--in", "entry", "--out", "g1_grn"]), 1)

    # ---- S9
    WATCH = ("[watch] 100.000 heading finite\n[watch] 100.100 leg FOLLOWING \n"
             "[watch] 112.000 heading NaN\n[watch] 112.200 leg DEGRADED heading unknown (NaN)\n"
             "[watch] 133.000 heading finite\n[watch] 135.000 leg PLANNING \n[watch] 136.000 leg FOLLOWING \n")
    DONE = "  200 [result] outcome 0  Safe passage complete\n"

    def test_s9_parse_watch(self):
        ev = K.parse_watch(self.WATCH)
        self.assertEqual(ev[0], (100.0, "heading", "finite", ""))
        self.assertEqual(ev[3], (112.2, "leg", "DEGRADED", "heading unknown (NaN)"))

    def test_s9_passes(self):
        v, d = K.score_s9(K.parse_watch(self.WATCH), 110.0, 130.0, self.DONE, "")
        self.assertEqual(v, "PASS", d)
        self.assertIn("NaN at +2.0 s -> DEGRADED +0.20 s", d)
        self.assertIn("resumed +5.0 s from HDG 1", d)

    def test_s9_fails(self):
        ev = K.parse_watch(self.WATCH)
        self.assertEqual(K.score_s9([e for e in ev if e[2] != "NaN"], 110.0, 130.0, self.DONE, "")[0], "FAIL")
        self.assertEqual(K.score_s9([e for e in ev if e[2] != "DEGRADED"], 110.0, 130.0, self.DONE, "")[0], "FAIL")
        slow = [(t + 2.0 if v == "DEGRADED" else t, k, v, w) for t, k, v, w in ev]
        self.assertEqual(K.score_s9(slow, 110.0, 130.0, self.DONE, "")[0], "FAIL")
        self.assertEqual(K.score_s9(ev, 110.0, 130.0, self.DONE, "[WARN] nav: leg FAILED: blocked\n")[0], "FAIL")
        self.assertEqual(K.score_s9(ev, 110.0, 130.0, "  200 [result] outcome 1  timeout\n", "")[0], "FAIL")
        self.assertEqual(K.score_s9(ev, 110.0, 130.0, "", "")[0], "FAIL")
        stuck = [e for e in ev if e[2] not in ("PLANNING", "FOLLOWING") or e[0] < 112.0]
        self.assertEqual(K.score_s9(stuck, 110.0, 130.0, self.DONE, "")[0], "FAIL")
        failed = ev + [(140.0, "leg", "FAILED", "blocked")]
        self.assertEqual(K.score_s9(failed, 110.0, 130.0, self.DONE, "")[0], "FAIL")

    def test_s9_without_a_nan_says_what_the_legs_did_instead(self):
        ev = [e for e in K.parse_watch(self.WATCH) if e[2] != "NaN"]
        v, d = K.score_s9(ev, 110.0, 130.0, self.DONE, "")
        self.assertEqual(v, "FAIL")
        self.assertIn("heading never went NaN", d)
        self.assertIn("DEGRADED(heading unknown (NaN))@+2.2 PLANNING@+25.0 FOLLOWING@+26.0", d)
        self.assertEqual(K.legs_after([], 0.0), "none")
        long = [(float(i), "leg", "STATE", "") for i in range(12)]
        self.assertTrue(K.legs_after(long, 0.0).endswith("..."))

    def test_s9_a_nan_before_the_injection_is_not_the_injection(self):
        ev = K.parse_watch("[watch] 50.0 heading NaN\n[watch] 51.0 heading finite\n" + self.WATCH)
        v, d = K.score_s9(ev, 110.0, 130.0, self.DONE, "")
        self.assertEqual(v, "PASS", d)


class FakeSitl(threading.Thread):
    """A one-connection MAVLink 2 autopilot on a private loopback TCP port: HEARTBEAT, and a
    PARAM_VALUE echo for each PARAM_SET of a parameter it knows (the way ArduPilot ignores others)."""
    def __init__(self, known):
        super().__init__(daemon=True)
        self.known, self.stop, self.sets = dict(known), False, []
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]

    def run(self):
        conn, _ = self.sock.accept()
        conn.settimeout(0.1)
        try:
            self.serve(conn)
        except OSError:                              # the client closed its end
            pass
        finally:
            conn.close()
            self.sock.close()

    def serve(self, conn):
        from pymavlink.dialects.v20 import ardupilotmega as mav2
        mav = mav2.MAVLink(None, srcSystem=1, srcComponent=1)
        mav.robust_parsing = True
        next_hb = 0.0
        while not self.stop:
            if time.time() >= next_hb:
                conn.sendall(mav2.MAVLink_heartbeat_message(10, 3, 0, 0, 4, 3).pack(mav))
                next_hb = time.time() + 0.2
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue
            if not data:
                break
            for byte in data:
                m = mav.parse_char(bytes([byte]))
                if m is not None and m.get_type() == "PARAM_SET":
                    name = m.param_id if isinstance(m.param_id, str) else m.param_id.decode()
                    name = name.strip("\x00")
                    self.sets.append((name, m.param_value))
                    if name in self.known:
                        self.known[name] = m.param_value
                        conn.sendall(mav2.MAVLink_param_value_message(
                            name.encode(), m.param_value, 9, len(self.known), 0).pack(mav))


class SitlParamSetTest(unittest.TestCase):
    def test_set_and_ack(self):
        from crusader_sim import sitl_param_set as S
        sitl = FakeSitl({"SIM_GPS_HDG": 1.0})
        sitl.start()
        try:
            ok, why, t = S.set_param("SIM_GPS_HDG", 0, f"tcp:127.0.0.1:{sitl.port}", wait_s=3.0)
            self.assertTrue(ok, why)
            self.assertLess(abs(time.time() - t), 5.0)
            self.assertEqual(sitl.sets[-1], ("SIM_GPS_HDG", 0.0))
            self.assertEqual(sitl.known["SIM_GPS_HDG"], 0.0)
        finally:
            sitl.stop = True

    def test_unknown_parameter_is_not_acked(self):
        from crusader_sim import sitl_param_set as S
        sitl = FakeSitl({"SIM_GPS_HDG": 1.0})
        sitl.start()
        try:
            ok, why, t = S.set_param("SIM_NO_SUCH", 1, f"tcp:127.0.0.1:{sitl.port}", wait_s=0.5, tries=2)
            self.assertFalse(ok)
            self.assertIsNone(t)
            self.assertIn("is SIM_NO_SUCH a parameter", why)
            self.assertEqual(len(sitl.sets), 2)          # it did try twice
        finally:
            sitl.stop = True

    def test_nothing_listening(self):
        from crusader_sim import sitl_param_set as S
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()                                      # a port nobody listens on
        ok, why, t = S.set_param("SIM_GPS_HDG", 0, f"tcp:127.0.0.1:{port}", wait_s=0.3, tries=1)
        self.assertFalse(ok)
        self.assertIsNone(t)
        self.assertIn("127.0.0.1", why)

    def test_ack_matching(self):
        from crusader_sim import sitl_param_set as S
        self.assertTrue(S.acks(NS(param_id="SIM_GPS_HDG\x00\x00", param_value=0.0), "SIM_GPS_HDG", 0.0))
        self.assertTrue(S.acks(NS(param_id=b"SIM_GPS_HDG\x00", param_value=1.0), "SIM_GPS_HDG", 1.0))
        self.assertFalse(S.acks(NS(param_id="SIM_GPS_HDG", param_value=1.0), "SIM_GPS_HDG", 0.0))
        self.assertFalse(S.acks(NS(param_id="SIM_GPS_TYPE", param_value=0.0), "SIM_GPS_HDG", 0.0))


if __name__ == "__main__":
    unittest.main()
