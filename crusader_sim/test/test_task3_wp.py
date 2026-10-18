"""Offline tests for the Task 3 waypoint run's sim tools: course_random (the GREEN bay picked
once per bring-up) and upload_wp (the waypoint off a bay's face).

No ROS and no running sim. Needs python3 with PyYAML (the WSL host has it):

    cd ~/robotx_ws/src/rx26_asv/crusader_sim && python3 -m unittest test.test_task3_wp -v
"""
import io
import math
import os
import random
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest import mock

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from crusader_sim import course as C                                           # noqa: E402
from crusader_sim import course_random as R                                    # noqa: E402
from crusader_sim import gen_world                                             # noqa: E402
from crusader_sim import upload_wp as U                                        # noqa: E402


def dock_course(green="random", facing=-90.0, x=0.0, y=20.0):
    return {"name": "t", "origin": {"lat": 1.2806, "lon": 103.8557}, "elements": [
        {"type": "dock", "name": "dock", "x": x, "y": y, "facing_deg": facing, "green_bay": green,
         "lit_window": {"bay": 2, "slot": 1, "colour": "red"}},
        {"type": "platform", "name": "p", "x": 1.0, "y": 2.0, "circle": "red"}]}


def dock_of(course):
    return next(e for e in course["elements"] if e["type"] == "dock")


class CourseRandomTest(unittest.TestCase):
    def setUp(self):
        p = mock.patch.dict(os.environ)
        p.start()
        self.addCleanup(p.stop)
        os.environ.pop("SIM_GREEN_BAY", None)

    def test_random_picks_a_bay_and_the_fire_follows_it(self):
        seen = set()
        for seed in range(40):
            c, picked = R.resolve(dock_course(), random.Random(seed))
            d = dock_of(c)
            self.assertIn(d["green_bay"], (1, 2, 3))
            self.assertEqual(d["lit_window"]["bay"], d["green_bay"])     # the fire is in the GREEN bay
            self.assertEqual(d["lit_window"]["slot"], 1)                  # the rest of it kept
            self.assertEqual(picked, [("green_bay", d["green_bay"])])
            seen.add(d["green_bay"])
        self.assertEqual(seen, {1, 2, 3}, "not always the middle")

    def test_case_does_not_matter(self):
        c, picked = R.resolve(dock_course("Random"), random.Random(1))
        self.assertIn(dock_of(c)["green_bay"], (1, 2, 3))
        self.assertEqual(len(picked), 1)

    def test_sim_green_bay_forces_one(self):
        os.environ["SIM_GREEN_BAY"] = "3"
        for seed in range(10):
            c, _ = R.resolve(dock_course(), random.Random(seed))
            self.assertEqual(dock_of(c)["green_bay"], 3)
            self.assertEqual(dock_of(c)["lit_window"]["bay"], 3)

    def test_a_bad_sim_green_bay_is_ignored(self):
        os.environ["SIM_GREEN_BAY"] = "7"
        c, _ = R.resolve(dock_course(), random.Random(0))
        self.assertIn(dock_of(c)["green_bay"], (1, 2, 3))

    def test_a_fixed_bay_is_left_alone(self):
        os.environ["SIM_GREEN_BAY"] = "3"
        c, picked = R.resolve(dock_course(green=1), random.Random(0))
        self.assertEqual(dock_of(c)["green_bay"], 1)
        self.assertEqual(dock_of(c)["lit_window"]["bay"], 2)
        self.assertEqual(picked, [])

    def test_no_lit_window_gets_one_in_the_green_bay(self):
        course = dock_course()
        del dock_of(course)["lit_window"]
        c, _ = R.resolve(course, random.Random(0))
        d = dock_of(c)
        self.assertEqual(d["lit_window"], {"slot": 1, "colour": "red", "bay": d["green_bay"]})

    def test_main_rewrites_the_runs_copy_once(self):
        os.environ["SIM_GREEN_BAY"] = "1"
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "c.yaml")
            with open(p, "w", encoding="utf-8") as f:
                yaml.safe_dump(dock_course(), f)
            out = io.StringIO()
            with redirect_stdout(out):
                self.assertEqual(R.main([p]), 0)
            self.assertEqual(out.getvalue().strip(), "green_bay 1")
            with open(p, encoding="utf-8") as f:
                text = f.read()
            self.assertTrue(text.startswith("# THIS RUN'S COPY"))
            again = yaml.safe_load(text)
            self.assertEqual(dock_of(again)["green_bay"], 1)
            self.assertEqual(again["elements"][1]["name"], "p")             # the rest of the course kept
            out = io.StringIO()
            with redirect_stdout(out):                                     # nothing random left
                self.assertEqual(R.main([p]), 0)
            self.assertEqual(out.getvalue(), "")

    def test_the_shipped_courses_still_say_random(self):
        for name in ("task3", "task3_wp"):
            self.assertEqual(str(dock_of(C.load(name))["green_bay"]).lower(), "random", name)

    def test_the_world_will_not_build_an_unpicked_bay(self):
        d = dock_of(dock_course())
        with self.assertRaises(SystemExit):
            gen_world.dock(d)


class BayWaypointTest(unittest.TestCase):
    def test_task3_wp_bays_left_to_right(self):
        course = C.load("task3_wp")                  # dock at (0, 20), faces out to the south
        for bay, x in ((1, -2.0), (2, 0.0), (3, 2.0)):
            e, n = U.bay_waypoint(course, bay, 5.0)
            self.assertAlmostEqual(e, x, places=6)
            self.assertAlmostEqual(n, 20.0 - U.FACE_U - 5.0, places=6)

    def test_closer_is_closer(self):
        course = C.load("task3_wp")
        e5, n5 = U.bay_waypoint(course, 3, 5.0)
        e3, n3 = U.bay_waypoint(course, 3, 3.0)
        self.assertAlmostEqual(e3, e5, places=6)
        self.assertAlmostEqual(n3 - n5, 2.0, places=6)

    def test_left_is_on_the_left_from_the_water_whatever_way_the_dock_faces(self):
        for facing in (-90.0, 0.0, 90.0, 180.0, 37.0):
            course = dock_course(green=1, facing=facing, x=5.0, y=-3.0)
            psi = math.radians(facing)
            out = (math.cos(psi), math.sin(psi))
            look = (-out[0], -out[1])                 # looking in from the water
            pts = {b: U.bay_waypoint(course, b, 4.0) for b in (1, 2, 3)}
            for b, (e, n) in pts.items():
                # 4 m + FACE_U out from the dock along its facing
                along = (e - 5.0) * out[0] + (n + 3.0) * out[1]
                self.assertAlmostEqual(along, U.FACE_U + 4.0, places=6, msg=f"{facing} bay {b}")
            # bay 1 is to the LEFT of bay 3 for a viewer looking in: cross(look, 3 -> 1) > 0
            d = (pts[1][0] - pts[3][0], pts[1][1] - pts[3][1])
            self.assertGreater(look[0] * d[1] - look[1] * d[0], 0.0, f"facing {facing}")
            self.assertAlmostEqual(math.hypot(*d), 2 * U.PITCH_M, places=6)

    def test_bays_by_name(self):
        self.assertEqual([U.BAYS[k] for k in ("left", "middle", "right")], [1, 2, 3])


if __name__ == "__main__":
    unittest.main()
