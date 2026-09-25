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


if __name__ == "__main__":
    unittest.main(verbosity=2)
