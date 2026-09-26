#!/usr/bin/env python3
"""test_world.py — the simulated course, on its own. No runner, no browser.

    python tools/task3_sim/test_world.py

A simulator that is wrong makes the tree look wrong (or, worse, right), so the
world is held to the same standard as the tree: every piece a test, and the
ones that would mislead quietly - which bay is "1", which colour the code
starts with, what the camera can actually see from the berth - against
numbers worked out by hand from the build guide.
"""
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import world as W                                         # noqa: E402

# The default course: dock centre (the deck edge, mid-dock) at (0, 20), bays
# opening SOUTH, 2 m apart - bay 1 at x=-2, bay 2 at 0, bay 3 at +2. A berth
# (body origin 1.25 m out, bow in) is at y = 18.75, heading north.
BERTH_N = 20.0 - 1.25


def range_to_plane(nx, ny, d, bearing_deg):
    """dock_math.hpp's rangeToPlane, restated: what the runner will compute."""
    b = math.radians(bearing_deg)
    den = nx * math.cos(b) + ny * math.sin(b)
    return -d / den


def run_to(boat, target, seconds, current=(0.0, 0.0), dt=0.05):
    boat.set_point(*target)
    for _ in range(int(seconds / dt)):
        boat.step(dt, current)


class Projection(unittest.TestCase):
    def test_round_trip(self):
        o = (27.7745, -82.6320)
        lat, lon = W.to_latlon(12.5, -40.25, o)
        e, n = W.to_local(lat, lon, o)
        self.assertAlmostEqual(e, 12.5, places=6)
        self.assertAlmostEqual(n, -40.25, places=6)

    def test_north_is_north(self):
        o = (27.7745, -82.6320)
        lat, _ = W.to_latlon(0.0, 100.0, o)
        self.assertGreater(lat, o[0])


class DockGeometry(unittest.TestCase):
    """The build guide's dock, and bay 1 on the LEFT for someone FACING it."""

    def test_build_guide_dimensions(self):
        d = W.Dock(W.Scenario())
        self.assertAlmostEqual(d.pitch, 2.0)                       # 1.5 slip + 0.5 finger
        self.assertAlmostEqual(d.faces[3][0] - d.faces[1][0], 4.0)
        u0, u1, _v0, v1 = d.finger_rects[0]
        self.assertAlmostEqual(u1 - u0, 0.5)
        self.assertAlmostEqual(v1, 2.0)
        lo, hi = d.slip(2)
        self.assertAlmostEqual(hi - lo, 1.5)
        u0, u1, _, _ = d.deck_rect
        self.assertAlmostEqual(u1 - u0, 6.5)                        # 13 cubes

    def test_open_to_the_south(self):
        # Faces point south; you face them looking NORTH; your left is WEST.
        d = W.Dock(W.Scenario(facing_deg=180.0))
        self.assertLess(d.faces[1][0], d.faces[3][0])

    def test_open_to_the_north(self):
        # Faces point north; you face them looking SOUTH; your left is EAST.
        d = W.Dock(W.Scenario(facing_deg=0.0))
        self.assertGreater(d.faces[1][0], d.faces[3][0])

    def test_windows_where_the_drawing_puts_them(self):
        d = W.Dock(W.Scenario(facing_deg=180.0))
        (i0, s0, p0, _h0), (i1, s1, p1, _h1) = d.windows(2)
        self.assertEqual((i0, s0, i1, s1), (0, "UL", 1, "LR"))
        self.assertLess(p0[0], p1[0])                 # UL west of LR, facing north
        # deck 0.3 + panel 0.605 + half of 0.29: the upper window's centre
        self.assertAlmostEqual(p0[2], 0.3 + 0.605 + 0.145, places=2)
        self.assertGreater(p0[2], p1[2])
        self.assertAlmostEqual(d.indicator(2)[2], 0.3 + 0.08, places=2)

    def test_berth_contact_and_bay_of(self):
        d = W.Dock(W.Scenario())
        b = W.Boat(0.0, BERTH_N, 0.0)                 # bay 2's berth, bow in
        self.assertEqual(d.bay_of(b.corners()), 2)
        self.assertFalse(d.contact(b.corners()))
        # A 1 x 0.6 m hull can turn right round in a 1.5 m slip.
        b.yaw = 90.0
        self.assertEqual(d.bay_of(b.corners()), 2)
        # 0.6 m off the centreline, a corner is 0.9 m out: in the finger.
        b = W.Boat(0.6, BERTH_N, 0.0)
        self.assertEqual(d.bay_of(b.corners()), 0)
        self.assertTrue(d.contact(b.corners()))
        # Nosed into the deck.
        b = W.Boat(0.0, 20.0 - 0.3, 0.0)
        self.assertTrue(d.contact(b.corners()))
        self.assertEqual(d.bay_of(W.Boat(-2.0, BERTH_N, 0.0).corners()), 1)
        # Stern out past the finger ends: not fully docked.
        self.assertEqual(d.bay_of(W.Boat(0.0, 20.0 - 1.8, 0.0).corners()), 0)


class BoatModel(unittest.TestCase):
    def test_reaches_a_point_with_a_small_wp_radius(self):
        b = W.Boat(0, 0, 0, wp_radius=0.2)
        run_to(b, (0.0, 20.0), 40)
        self.assertLess(W.norm(W.sub(b.p, (0, 20))), 0.25)
        self.assertTrue(b.loitering)

    def test_wp_radius_two_parks_it_short(self):
        # The boat's own WP_RADIUS. It is reached - and stops - well short.
        b = W.Boat(0, 0, 0, wp_radius=2.0)
        run_to(b, (0.0, 20.0), 40)
        self.assertTrue(b.loitering)
        self.assertGreater(W.norm(W.sub(b.p, (0, 20))), 1.0)

    def test_it_turns_rather_than_strafes(self):
        b = W.Boat(0, 0, 0)                           # facing north
        b.set_point(-10.0, 0.0)                       # due west
        for _ in range(20):                           # one second
            b.step(0.05)
        self.assertLess(abs(b.e), 0.2)                # has not slid west
        self.assertGreater(abs(W.wrap180(b.yaw)), 20.0)   # has started turning

    def test_arrival_heading_is_the_direction_of_travel(self):
        b = W.Boat(0, -10, 90)                        # facing EAST
        run_to(b, (0.0, 8.0), 60)                     # a point to the north
        self.assertLess(abs(W.wrap180(b.yaw)), 12.0)  # ends facing north-ish

    def test_it_holds_the_leg_line_across_a_current(self):
        # AR_WPNav follows the origin->destination LINE. 20 m north through a
        # 0.1 m/s eastward set: the track must stay near x=0 all the way.
        b = W.Boat(0, 0, 0)
        b.set_point(0.0, 20.0)
        worst = 0.0
        for _ in range(int(40 / 0.05)):
            b.step(0.05, (0.1, 0.0))
            if b.loitering:
                break
            worst = max(worst, abs(b.e))
        self.assertTrue(b.loitering)
        self.assertLess(worst, 0.4)
        self.assertLess(W.wrap180(b.yaw), 0.0, "it crabs INTO the current (west of north)")

    def test_loiter_ignores_small_drift_and_returns_from_large(self):
        b = W.Boat(0, 0, 0)
        run_to(b, (0.0, 10.0), 40)
        start = b.loiter_pt
        # 0.1 m/s eastward current: 1.5 m in 15 s is left alone ...
        run_to(b, (0.0, 10.0), 15, current=(0.1, 0.0))
        self.assertGreater(b.e - start[0], 1.2)
        self.assertFalse(b.returning)
        # ... and past LOIT_RADIUS it goes back.
        for _ in range(int(20 / 0.05)):
            b.step(0.05, (0.1, 0.0))
        self.assertLess(W.norm(W.sub(b.p, start)), W.Boat(0, 0, 0).loit_radius + 0.3)


class LightSequence(unittest.TestCase):
    def test_disruptive_code(self):
        sc = W.Scenario(tier=2, code=("green", "blue"), target_window=1,
                        activation_delay_s=1.0, extinguish_s=2.0)
        L = W.Lights(sc)
        t, dt = 0.0, 0.05

        def advance(sec, spraying=False):
            nonlocal t
            for _ in range(int(round(sec / dt))):
                t += dt
                L.step(t, spraying, dt)

        self.assertEqual(L.window_colour(t), (1, W.OFF))
        L.activate(t)
        advance(1.1)
        self.assertEqual(L.window_colour(t), (1, W.RED))
        advance(1.0, spraying=True)
        self.assertEqual(L.state, "FIRE")            # 1 s is not enough
        advance(1.1, spraying=True)
        self.assertEqual(L.window_colour(t), (1, W.GREEN))
        advance(5.05)
        self.assertEqual(L.window_colour(t), (1, W.OFF))   # the 1 s gap
        advance(1.0)
        self.assertEqual(L.state, "CODE")
        seq = []
        for _ in range(10):                         # sample mid-second
            advance(0.5)
            seq.append(L.window_colour(t)[1])
            advance(0.5)
        # c1 (GREEN) 1 s, off 1 s, c2 (BLUE) 1 s, off 2 s, repeat
        self.assertEqual(seq, [W.GREEN, W.OFF, W.BLUE, W.OFF, W.OFF] * 2)
        advance(52.0)
        self.assertEqual(L.state, "DONE")

    def test_core_stops_after_green(self):
        sc = W.Scenario(tier=0, activation_delay_s=0.0, extinguish_s=0.1)
        L = W.Lights(sc)
        L.activate(0.0)
        t = 0.0
        for _ in range(200):
            t += 0.05
            L.step(t, True, 0.05)
        self.assertEqual(L.state, "DONE")


class CameraModel(unittest.TestCase):
    def setUp(self):
        self.sc = W.Scenario(green_bay=3, unknown_rate=0.0, miscolour=0.0, cam_pitch_deg=0.0)
        self.dock = W.Dock(self.sc)
        self.lights = W.Lights(self.sc)

    def test_left_to_right_and_plane_range(self):
        cam = W.Camera(self.sc, random.Random(3))
        boat = W.Boat(0.0, 20.0 - 5.37, 0.0)        # camera 5 m from the faces
        obs = cam.frame(0.0, boat, self.dock, self.lights)
        self.assertEqual(len(obs["bays"]), 3)
        truth = [b["_truth_bay"] for b in obs["bays"]]
        self.assertEqual(truth, [1, 2, 3], "bay_index 0 must be the LEFTMOST in the frame")
        self.assertGreater(obs["bays"][0]["bearing_deg"], 0.0, "left is + bearing")
        for b in obs["bays"]:
            r = range_to_plane(b["plane_normal"][0], b["plane_normal"][1],
                               b["plane_offset"], b["bearing_deg"])
            face = self.dock.faces[b["_truth_bay"]]
            self.assertAlmostEqual(r, W.norm(W.sub(face, (0.0, 15.0))), delta=0.3)

    def test_indicator_colours(self):
        cam = W.Camera(self.sc, random.Random(4))
        boat = W.Boat(0.0, 13.0, 0.0)                 # 7 m out
        got = {1: set(), 2: set(), 3: set()}
        for i in range(30):
            for b in cam.frame(i / 15.0, boat, self.dock, self.lights)["bays"]:
                if b["indicator_present"]:
                    got[b["_truth_bay"]].add(b["indicator_colour"])
        self.assertEqual(got[3], {W.GREEN})
        self.assertEqual(got[1], {W.RED})

    def test_nothing_behind_or_beyond(self):
        cam = W.Camera(self.sc, random.Random(5))
        self.assertEqual(cam.frame(0, W.Boat(0.0, 13.0, 180.0), self.dock, self.lights)["bays"], [])
        self.assertEqual(cam.frame(0, W.Boat(0.0, -10.0, 0.0), self.dock, self.lights)["bays"], [])

    def test_what_a_level_camera_sees_from_the_berth(self):
        """THE finding: level at 0.41 m with the hull band masked, from the
        berth the camera sees NEITHER window whole. -25 deg of pitch sees both."""
        for win in (0, 1):
            level = W.World(W.Scenario(target_window=win, cam_pitch_deg=0.0))
            up = W.World(W.Scenario(target_window=win, cam_pitch_deg=-25.0))
            self.assertFalse(level.target_visible_from_berth(), "level, window %d" % win)
            self.assertTrue(up.target_visible_from_berth(), "pitched up, window %d" % win)
        # Without the hull band the lower window is still out of view level.
        self.assertFalse(W.World(W.Scenario(target_window=1, cam_pitch_deg=0.0,
                                            hull_band_rows=0)).target_visible_from_berth())

    def test_pitched_frames_still_place_the_face(self):
        """A pitched camera reports in its tilted frame; levelling it with the
        pitch (as dock_math does) must give the true horizontal range."""
        sc = W.Scenario(unknown_rate=0.0, cam_pitch_deg=-25.0)
        dock, lights = W.Dock(sc), W.Lights(sc)
        cam = W.Camera(sc, random.Random(7))
        boat = W.Boat(0.0, 15.0, 0.0)                 # dead ahead of bay 2
        b = next(x for x in cam.frame(0.0, boat, dock, lights)["bays"] if x["_truth_bay"] == 2)
        n = b["plane_normal"]
        # Tilted UP, the camera's own up axis leans back toward the boat - and
        # so does the face normal, so its camera-frame z is +sin(25): the
        # same sign test_dock_math builds its pitched sighting with.
        self.assertAlmostEqual(n[2], math.sin(math.radians(25.0)), delta=0.1)
        t = range_to_plane(n[0], n[1], b["plane_offset"], b["bearing_deg"])
        horiz = t * math.cos(math.radians(-25.0))     # levelled, at bearing ~0
        self.assertAlmostEqual(horiz, 20.0 - 15.0 - 0.37, delta=0.25)

    def test_the_timing_layer_names_the_code_in_order(self):
        """c1 must be the colour AFTER the 2 s off - the handbook's first colour."""
        sc = W.Scenario(green_bay=2, tier=2, code=("red", "blue"), target_window=0,
                        unknown_rate=0.02, activation_delay_s=0.0, extinguish_s=0.5)
        dock, lights = W.Dock(sc), W.Lights(sc)
        cam = W.Camera(sc, random.Random(6))
        boat = W.Boat(0.0, BERTH_N, 0.0)              # in the berth, facing in
        lights.activate(0.0)
        t, found, code_start = 0.0, None, None
        while t < 40.0 and found is None:
            t += 1.0 / 15.0
            lights.step(t, True, 1.0 / 15.0)
            if lights.state == "CODE" and code_start is None:
                code_start = t
            obs = cam.frame(t, boat, dock, lights)
            if obs["target_pattern"] == "code":
                found = (obs["target_colours"], obs["target_window_index"], t)
        self.assertIsNotNone(found, "the timing layer never called the code")
        self.assertEqual(found[0], ["red", "blue"])
        self.assertEqual(found[1], 0)
        # two full 5 s cycles, give or take the alignment
        self.assertLess(found[2] - code_start, 13.0)


class Judging(unittest.TestCase):
    def test_docking_report_is_checked_against_the_truth(self):
        sc = W.Scenario(green_bay=2)
        j, L = W.Judge(sc), W.Lights(sc)
        j.on_docking(1.0, 2, 0, L)                    # says 2, is not docked
        self.assertIsNone(j.docking)
        self.assertEqual(L.state, "IDLE")
        j.on_docking(2.0, 3, 2, L)                    # docked in 2, says 3
        self.assertIsNone(j.docking)
        j.on_docking(3.0, 2, 2, L)                    # right
        self.assertIsNotNone(j.docking)
        self.assertEqual(L.state, "ARMING")

    def test_request_marking(self):
        sc = W.Scenario(tier=2, code=("red", "blue"))
        j = W.Judge(sc)
        j.on_request(1.0, {"resource_color": "COLOR_RED", "delivery_circle_color": "COLOR_BLUE"})
        j.on_uav(1.0, {"seq": 1, "resource_color": 1, "delivery_color": 3})
        self.assertTrue(j.request["ok"])
        self.assertTrue(j.uav["ok"])
        j2 = W.Judge(sc)
        j2.on_request(1.0, {"resource_color": "COLOR_BLUE", "delivery_circle_color": "COLOR_RED"})
        self.assertFalse(j2.request["ok"], "the reversed order is wrong")

    def test_spray_lands_on_the_window_it_is_aimed_at(self):
        sc = W.Scenario(green_bay=2, cam_pitch_deg=-25.0)
        w = W.World(sc)
        w.boat = W.Boat(0.0, BERTH_N, 0.0)
        cam, look = w.boat.camera()
        f, l = W.hvec(look), W.port(W.hvec(look))
        for idx, _slot, wc, _half in w.dock.windows(2):
            c = w.camera.to_cam(cam, f, l, wc)        # the tilted camera frame
            w.on_cannon({"fire": True, "x": c[0], "y": c[1], "z": c[2]})
            self.assertEqual(w._spray_hits(), idx)
        c = w.camera.to_cam(cam, f, l, (0.0, 20.0, 0.8))   # the face, between windows
        w.on_cannon({"fire": True, "x": c[0], "y": c[1], "z": c[2]})
        self.assertIsNone(w._spray_hits())
        w.on_cannon({"fire": False, "x": 0, "y": 0, "z": 0})
        self.assertIsNone(w._spray_hits())



def fire_world(**kw):
    """sim.py --fire's course, with overrides. The boat's body origin at
    `rng` m in front of bay 2's deck edge, square on, u = `u` (+ right)."""
    rng, u = kw.pop("rng", 3.22), kw.pop("u", 0.0)
    sc = W.Scenario(fire_lit=True, target_window=0, green_bay=2, tier=0,
                    start_e=u, start_n=20.0 - rng, start_heading=0.0, **kw)
    return W.World(sc)


class FixedNozzle(unittest.TestCase):
    """The truth the fire tree is aimed against."""

    def test_fitted_to_the_pool_number(self):
        # level and square on at nozzle_hit_range_m, in line with the window:
        # the stream crosses the face ON the upper-left window's top edge
        w = fire_world(u=-0.22)
        u, z, run = w._crossing()
        self.assertAlmostEqual(run, 3.22 - 0.45, places=6)
        self.assertAlmostEqual(z, 0.3 + 0.895, places=6)                # deck + 895 mm
        self.assertAlmostEqual(u, -0.22, places=6)
        # a drag-free 45 deg arc that does that reaches ~3.9 m on the level
        self.assertAlmostEqual(w.nozzle.v ** 2 / W.G, 3.885, delta=0.01)

    def test_the_arc_is_falling_there_so_closer_is_higher(self):
        near = fire_world(rng=3.0, u=-0.22)._crossing()[1]
        far = fire_world(rng=3.4, u=-0.22)._crossing()[1]
        self.assertGreater(near, far)
        # ~0.43 m of height per metre of range on this arc
        self.assertAlmostEqual((near - far) / 0.4, 0.43, delta=0.05)

    def test_bow_up_lifts_the_stream_and_roll_moves_it_sideways(self):
        w = fire_world(u=-0.22)
        z0 = w._crossing()[1]
        w.att = (0.0, 1.0, 0.0, 0.0)                     # 1 deg bow up
        self.assertGreater(w._crossing()[1] - z0, 0.02)
        w.att = (2.0, 0.0, 0.0, 0.0)                     # rolled right
        self.assertGreater(w._crossing()[0], -0.22 + 0.02)

    def test_turning_left_moves_the_stream_left(self):
        w = fire_world()
        u0 = w._crossing()[0]
        w.boat.yaw = 356.0                               # 4 deg left of square
        self.assertAlmostEqual(w._crossing()[0] - u0,
                               -(3.22 - 0.45) * math.tan(math.radians(4.0))
                               - 0.45 * math.sin(math.radians(4.0)), delta=0.01)

    def test_a_left_skewed_nozzle_lands_left(self):
        straight = fire_world()._crossing()[0]
        skewed = fire_world(nozzle_yaw_bias_deg=3.0)._crossing()[0]
        self.assertAlmostEqual(skewed - straight, -2.77 * math.tan(math.radians(3.0)), delta=0.01)

    def test_the_cal_error_scenario_misses_high(self):
        # the truth is 3.6 m, the tree fires from 3.22 m: well over the top edge
        w = fire_world(u=-0.22, nozzle_hit_range_m=3.6)
        self.assertGreater(w._crossing()[1] - w.target_edge()[1], W.World.STREAM_R + 0.1)


class WallRangeSensor(unittest.TestCase):

    def test_range_bearing_and_offset_signs(self):
        w = fire_world(rng=3.2, u=-0.2)                  # 0.2 m LEFT of the slip centre
        w.boat.yaw = 5.0                                 # bow 5 deg right of square
        m = w.wall_range()
        self.assertTrue(m["valid"])
        self.assertAlmostEqual(m["range_m"], 3.2, delta=0.03)
        # the wall's nearest point is 5 deg to the LEFT: + (turn left to square)
        self.assertAlmostEqual(m["angle_deg"], 5.0, delta=1.5)
        self.assertAlmostEqual(m["lat_m"], 0.2, delta=0.03)        # + = LEFT

    def test_finger_tip_lock_reads_short_and_sees_no_fingers(self):
        w = fire_world(rng=3.2, wall_on_fingers=True)
        m = w.wall_range()
        self.assertAlmostEqual(m["range_m"], 1.2, delta=0.03)
        self.assertIsNone(m["lat_m"])

    def test_no_wall_and_out_of_view(self):
        self.assertFalse(fire_world(wall_ok=False).wall_range()["valid"])
        w = fire_world()
        w.boat.yaw = 40.0                                # more oblique than max_angle_deg
        self.assertFalse(w.wall_range()["valid"])
        # wall_range_node's r_max (4 m): blind from 4.5 m unless it is raised
        self.assertFalse(fire_world(rng=4.5).wall_range()["valid"])
        self.assertTrue(fire_world(rng=4.5, wall_r_max=9.0).wall_range()["valid"])
        self.assertIsNone(fire_world(rng=6.0, wall_r_max=9.0).wall_range()["lat_m"])  # no fingers


class HeadingSpeed(unittest.TestCase):
    """The autopilot side (as remembered) and the bridge's gates in front of it."""

    def test_drives_the_heading_and_speed_then_times_out_into_loiter(self):
        b = W.Boat(0.0, 0.0, 0.0)
        self.assertTrue(b.set_heading_speed(10.0, 0.2))
        for _ in range(40):
            b.step(0.05)
        self.assertAlmostEqual(b.yaw, 10.0, delta=0.5)
        self.assertAlmostEqual(b.v, 0.2, delta=0.01)
        for _ in range(int(3.5 / 0.05)):
            b.step(0.05)
        self.assertIsNone(b.hs)
        self.assertEqual(b.hs_timeouts, 1)
        self.assertTrue(b.loitering)

    def test_astern_is_a_negative_speed(self):
        b = W.Boat(0.0, 0.0, 0.0)
        b.set_heading_speed(0.0, -0.2)
        for _ in range(40):
            b.step(0.05)
        self.assertLess(b.n, -0.1)
        self.assertAlmostEqual(b.yaw, 0.0, delta=0.5)

    def test_ignored_outside_guided(self):
        b = W.Boat(0.0, 0.0, 0.0)
        b.mode = "MANUAL"
        self.assertFalse(b.set_heading_speed(10.0, 0.2))

    def test_bridge_gates_clamp_and_deadman(self):
        w = fire_world()
        br = w.bridge
        ok, why = br.on_heading_speed(0.0, 350.0, 0.9)
        self.assertTrue(ok)
        self.assertIn("clamped", why)
        self.assertAlmostEqual(w.boat.hs[1], 0.4, places=6)
        self.assertAlmostEqual(w.boat.hs[0], 350.0, places=6)     # through the quaternion
        br.tick(0.4)
        self.assertAlmostEqual(w.boat.hs[1], 0.4, places=6)       # not yet
        br.tick(0.6)
        self.assertEqual(w.boat.hs[1], 0.0)                       # the dead-man's stop
        w.boat.mode = "MANUAL"
        self.assertFalse(br.on_heading_speed(1.0, 0.0, 0.2)[0])
        w.boat.mode = "GUIDED"
        w.dropped = True
        self.assertFalse(br.on_heading_speed(1.0, 0.0, 0.2)[0])

    def test_a_drop_stops_the_boat_from_the_bridge_itself(self):
        w = fire_world()
        w.bridge.on_heading_speed(0.0, 0.0, 0.25)
        w.dropped = True
        w.bridge.tick(0.1)
        self.assertEqual(w.boat.hs[1], 0.0)

    def test_avoidance_holds_the_boat_off_only_while_enabled(self):
        w = fire_world(rng=3.2, autopilot_avoidance=True)
        w.bridge.on_heading_speed(0.0, 0.0, 0.25)
        w.step()
        w.step()
        self.assertEqual(w.boat.v, 0.0)          # the finger tips are inside 2 m of the bow
        w.on_avoidance(False)
        w.bridge.on_heading_speed(w.t, 0.0, 0.25)
        for _ in range(5):
            w.step()
        self.assertGreater(w.boat.v, 0.05)


class Pump(unittest.TestCase):

    def test_a_burst_is_water_for_its_length_after_the_ack(self):
        w = fire_world()
        br = w.bridge
        br.on_pump(1.0, 0.5, 7, "test")
        self.assertEqual(br.result, W.pump_core.RESULT_SENT)
        self.assertFalse(br.water(1.0 + br.ACK_S))
        self.assertTrue(br.water(1.0 + br.ACK_S + br.LATENCY_S + 0.01))
        self.assertTrue(br.water(1.0 + br.ACK_S + br.LATENCY_S + 0.49))
        self.assertFalse(br.water(1.0 + br.ACK_S + br.LATENCY_S + 0.51))
        br.tick(1.0 + br.ACK_S)
        self.assertEqual(br.pump_state(1.2)["last_result"], W.pump_core.RESULT_ACCEPTED)
        self.assertEqual(br.pump_state(1.2)["last_seq"], 7)

    def test_the_bridges_refusals(self):
        w = fire_world()
        br = w.bridge
        br.on_pump(1.0, 0.5, 1)
        br.on_pump(1.8, 0.5, 2)                       # 0.2 s after it ended
        self.assertEqual(br.result, W.pump_core.RESULT_REFUSED)
        self.assertIn("too soon", br.reason)
        br.on_pump(5.0, 1.5, 3)                       # longer than pump_max_burst_s
        self.assertIn("outside", br.reason)
        w.boat.armed = False
        br.on_pump(9.0, 0.5, 4)
        self.assertEqual(br.reason, "disarmed")
        off = fire_world(pump_path=False).bridge
        self.assertFalse(off.pump_state(0.0)["enabled"])
        off.on_pump(0.0, 0.5, 5)
        self.assertEqual(off.result, W.pump_core.RESULT_REFUSED)

    def test_a_good_burst_puts_the_fire_out_and_is_recorded(self):
        w = fire_world(u=-0.22, extinguish_s=0.3)
        w.bridge.on_pump(w.t, 0.5, 1)
        for _ in range(30):
            w.step()
        self.assertEqual(len(w.shots), 1)
        self.assertTrue(w.shots[0]["hit"])
        self.assertIsNotNone(w.lights.hit_t)
        self.assertEqual(w.lights.state, "GREEN")

    def test_spray_returns_only_while_the_water_flies(self):
        w = fire_world()
        clean = [w.wall_range()["range_m"] for _ in range(50)]
        self.assertLess(max(clean) - min(clean), 0.05)
        w.bridge.on_pump(w.t, 0.5, 1)
        w.t += 0.3
        wet = [w.wall_range()["range_m"] for _ in range(100)]
        self.assertLess(min(wet), 3.22 - 0.09)


class Sea(unittest.TestCase):

    def test_flat_and_still_is_level(self):
        w = fire_world(sea=0.0)
        for _ in range(40):
            w.step()
        self.assertTrue(all(abs(a) < 1e-6 for a in w.att))

    def test_accelerating_kicks_the_pitch(self):
        w = fire_world(sea=0.0, rng=6.0)
        w.bridge.on_heading_speed(0.0, 0.0, 0.3)
        peak = 0.0
        for _ in range(20):
            w.bridge.on_heading_speed(w.t, 0.0, 0.3)
            w.step()
            peak = max(peak, abs(w.att[1]))
        self.assertGreater(peak, 0.5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
