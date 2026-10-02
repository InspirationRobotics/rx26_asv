"""Offline tests for the Task 1 referee's per-buoy side scoring (crusader_sim.task1_judge).

No ROS and no running sim. Needs python3 with PyYAML (the WSL host has it):

    cd ~/robotx_ws/src/rx26_asv/crusader_sim && python3 -m unittest test.test_task1_judge -v

Frame: ENU metres, the boat drives east, so STARBOARD is SOUTH (-y) and PORT is north.
handbook 3.3.2: every RED is kept to starboard, every GREEN to port, whether or not it has
a partner. The referee scores each one on its own.
"""
import math
import os
import sys
import unittest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from crusader_sim import course as C                                           # noqa: E402
from crusader_sim.task1_judge import Task1Judge, format_verdict                # noqa: E402


def buoy(name, x, y, beacon):
    return {"type": "robobuoy", "name": name, "x": x, "y": y, "beacon": beacon}


def judge(*buoys):
    return Task1Judge({"elements": list(buoys)}, echo=False)


def drive(j, points, step=0.05, yaw=None):
    """Feed the judge a polyline of ground-truth positions, resampled every `step` metres."""
    t = 0.0
    j.update(points[0][0], points[0][1], yaw, t=t)
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        n = max(1, round(math.hypot(bx - ax, by - ay) / step))
        for i in range(1, n + 1):
            t += step
            j.update(ax + (bx - ax) * i / n, ay + (by - ay) * i / n, yaw, t=t)


def arc(centre, radius, a0_deg, sweep_deg, step_deg=2.0):
    """Points round `centre` from bearing a0 (ENU degrees, ccw positive) by sweep (negative = cw)."""
    n = max(2, int(abs(sweep_deg) / step_deg))
    return [(centre[0] + radius * math.cos(math.radians(a0_deg + sweep_deg * i / n)),
             centre[1] + radius * math.sin(math.radians(a0_deg + sweep_deg * i / n)))
            for i in range(n + 1)]


class PerBuoySides(unittest.TestCase):
    def test_a_lone_red_is_judged_with_no_partner_at_all(self):
        # red at (10, -3): the eastbound track passes north of it, so it is to STARBOARD
        j = judge(buoy("r", 10, -3, "flash_red"))
        drive(j, [(0, 0), (20, 0)])
        v = j.verdict()
        self.assertEqual((v["buoys_correct"], v["buoys"]), (1, 1))
        self.assertTrue(v["pass"])
        self.assertEqual(v["gates"], 0)               # there is no gate, and nothing waits on one
        self.assertIn("correct (red kept to starboard", v["buoys_detail"]["r"])

    def test_a_lone_red_on_the_wrong_side_fails(self):
        j = judge(buoy("r", 10, 3, "flash_red"))        # north of the track: to port
        drive(j, [(0, 0), (20, 0)])
        v = j.verdict()
        self.assertFalse(v["pass"])
        self.assertEqual(v["buoys_correct"], 0)
        self.assertIn("WRONG SIDE (red kept to port", v["buoys_detail"]["r"])

    def test_a_lone_green_is_the_mirror_image(self):
        for y, ok in ((3.0, True), (-3.0, False)):       # green must be to PORT = north
            j = judge(buoy("g", 10, y, "flash_green"))
            drive(j, [(0, 0), (20, 0)])
            self.assertEqual(j.verdict()["pass"], ok, f"green at y={y}")

    def test_black_buoys_have_no_side(self):
        j = judge(buoy("b", 10, 1, "off"))
        drive(j, [(0, 0), (20, 0)])
        v = j.verdict()
        self.assertEqual((v["buoys"], v["buoys_detail"]), (0, {}))
        self.assertTrue(v["pass"])

    def test_a_buoy_never_reached_is_not_passed_and_fails(self):
        j = judge(buoy("g", 30, 3, "flash_green"))
        drive(j, [(0, 0), (20, 0)])                      # stops 10 m short
        v = j.verdict()
        self.assertEqual(v["buoys_detail"]["g"], "not passed")
        self.assertFalse(v["pass"])

    def test_the_direction_of_travel_decides_not_the_compass(self):
        # the same buoy, the same side of the map, passed westbound: now it is to port
        j = judge(buoy("r", 10, -3, "flash_red"))
        drive(j, [(20, 0), (0, 0)])
        self.assertFalse(j.verdict()["pass"])
        j = judge(buoy("g", 10, -3, "flash_green"))
        drive(j, [(20, 0), (0, 0)])
        self.assertTrue(j.verdict()["pass"])

    def test_a_diagonal_track_is_judged_against_its_own_heading(self):
        # heading north-east: starboard is south-east. A red at (10, 4) is north-west of the
        # track y = x: to PORT, wrong. A red at (10, 16): wrong too. A red at (14, 6) is
        # south-east of it: correct.
        j = judge(buoy("a", 6, 14, "flash_red"), buoy("b", 14, 6, "flash_red"))
        drive(j, [(0, 0), (20, 20)])
        d = j.verdict()["buoys_detail"]
        self.assertTrue(d["a"].startswith("WRONG SIDE"), d)
        self.assertTrue(d["b"].startswith("correct"), d)

    def test_the_closest_pass_is_the_one_that_counts(self):
        # correct at 2 m, then a wide loop that goes by again on the other side at 9 m
        j = judge(buoy("r", 10, -2, "flash_red"))
        drive(j, [(0, 0), (20, 0), (20, -11), (0, -11)])    # back west, south of the buoy
        v = j.verdict()
        self.assertTrue(v["pass"], v["buoys_detail"])
        self.assertIn("passed at 2.0 m", v["buoys_detail"]["r"])
        # ... and a far correct pass does not excuse a close wrong one
        j = judge(buoy("r", 10, 1, "flash_red"))             # 1 m to the north: wrong, close
        drive(j, [(0, 0), (20, 0), (20, -8), (0, -8)])       # back west 9 m south: red to starboard, but far
        self.assertFalse(j.verdict()["pass"])

    def test_a_pass_earned_survives_a_recolour(self):
        j = judge(buoy("r", 10, -3, "flash_red"))
        drive(j, [(0, 0), (20, 0)])
        j.set_states({"r": "off"})                           # the panel turns it black afterwards
        v = j.verdict()
        self.assertEqual(v["buoys"], 1)
        self.assertTrue(v["pass"])

    def test_a_far_pass_does_not_survive_a_recolour(self):
        # the entry orbit sweeps a red 40 m out "past" the boat; the UAV then turns it black
        j = judge(buoy("r", 10, 40, "flash_red"))
        drive(j, [(0, 0), (20, 0)])
        j.set_states({"r": "off"})
        v = j.verdict()
        self.assertEqual(v["buoys"], 0, v["buoys_detail"])
        self.assertTrue(v["pass"])

    def test_a_recolour_before_the_pass_sets_the_rule(self):
        j = judge(buoy("r", 10, -3, "flash_red"))
        j.set_states({"r": "flash_green"})                   # now it must be to port: it is not
        drive(j, [(0, 0), (20, 0)])
        self.assertFalse(j.verdict()["pass"])

    def test_far_passes_are_scored_but_not_announced(self):
        # an orbit-style loop 40 m out flips the buoy ahead -> behind: scored, silent
        j = judge(buoy("r", 10, 40, "flash_red"))
        drive(j, [(0, 0), (20, 0)])
        self.assertEqual(j.verdict()["buoys_detail"]["r"][:10], "WRONG SIDE")
        self.assertEqual(j.events, [])

    def test_events_say_what_happened(self):
        j = judge(buoy("r", 10, 3, "flash_red"))
        drive(j, [(0, 0), (20, 0)])
        self.assertTrue(any("passed r (red) with it to port" in e and "WRONG SIDE" in e
                            for e in j.events), j.events)


class WrongOrientationPair(unittest.TestCase):
    """A red on the LEFT of its green (facing down the course) is a pair nobody can cross
    forwards. The referee never looked at pairs for the verdict; it must still hold each
    buoy to its own side: north of the red AND south of the green, an S through the gap."""

    RED, GREEN = (56.0, 1.2), (66.0, -1.2)

    def field(self):
        return judge(buoy("red", *self.RED, "flash_red"), buoy("grn", *self.GREEN, "flash_green"))

    def test_straight_through_the_middle_fails_both(self):
        j = self.field()
        drive(j, [(40, 0), (80, 0)])
        v = j.verdict()
        self.assertEqual(v["buoys_correct"], 0, v["buoys_detail"])
        self.assertFalse(v["pass"])

    def test_the_old_gate_pairing_would_have_called_that_a_non_event(self):
        # the pair is one "gate" to the informational output, and a straight run between the
        # two never crosses the segment's line of sight the way a gate is crossed
        j = self.field()
        drive(j, [(40, 0), (80, 0)])
        self.assertEqual(j.verdict()["gates"], 1)

    def test_the_s_curve_passes_both(self):
        j = self.field()
        drive(j, [(40, 0), (52, 0), (56, 3.6), (60, 1.1), (64, -2.6), (68, -3.6), (80, 0)])
        v = j.verdict()
        self.assertEqual((v["buoys_correct"], v["buoys"]), (2, 2), v["buoys_detail"])
        self.assertTrue(v["pass"])

    def test_going_round_the_south_of_both_gets_the_red_wrong(self):
        j = self.field()
        drive(j, [(40, 0), (50, -9), (70, -4), (80, 0)])
        d = j.verdict()["buoys_detail"]
        self.assertTrue(d["red"].startswith("WRONG SIDE"), d)
        self.assertTrue(d["grn"].startswith("correct"), d)


class UnpairedCourse(unittest.TestCase):
    """courses/task1_unpaired.yaml: the side-fence course, with an idealised hand-drawn track."""

    def setUp(self):
        self.course = C.load("task1_unpaired")

    def test_the_course_is_what_the_rxl_field_can_carry(self):
        b = C.buoys(self.course)
        self.assertEqual(len(b), 10)                          # RXL_SAFE_PASSAGE: at most 10
        kinds = sorted(s for _, _, _, s, *_ in b)
        self.assertEqual(kinds, ["flash_blue", "flash_green", "flash_green", "flash_green",
                                 "flash_red", "flash_red", "off", "off", "off", "steady_blue"])
        self.assertEqual(len({n for n, *_ in b}), 10)

    def track(self, single_side=-1.0):
        """Entry orbit cw, the gate, the S through the wrong-way pair, round the lone green
        (south of it for single_side = -1: correct; north of it for +1: wrong), exit orbit ccw."""
        entry, exit_ = (10.0, 2.0), (88.0, 0.0)
        start = [(0.0, 0.0)]
        orbit_in = arc(entry, 6.0, 190.0, -400.0)               # clockwise, a bit over a lap
        # the reference planner's route (crusader_bt test_side_fences): north of black1, the
        # gate's approach -> through crossing, north of black2, then the S
        to_gate = [(13.0, 4.1), (17.0, 3.4), (21.0, 2.4), (22.0, 0.0), (36.0, 0.0)]
        s_curve = [(40.0, 1.1), (44.0, 2.4), (48.0, 2.6), (52.0, 2.6), (56.0, 3.6),
                   (60.0, 1.1), (64.0, -2.6), (68.0, -3.6)]
        lone_y = -4.0 + single_side * 2.4
        lone = [(72.0, lone_y), (76.0, lone_y), (80.0, lone_y * 0.5)]
        orbit_out = arc(exit_, 6.0, 180.0, 400.0)               # counter-clockwise
        return start + orbit_in + to_gate + s_curve + lone + orbit_out

    def test_the_correct_track_passes(self):
        j = Task1Judge(self.course, echo=False)
        drive(j, self.track(-1.0))
        v = j.verdict()
        self.assertTrue(v["pass"], format_verdict(v))
        self.assertEqual((v["buoys_correct"], v["buoys"]), (5, 5))
        self.assertEqual(v["circles"], {"entry": "correct", "exit": "correct"})

    def test_passing_the_lone_green_on_its_north_side_fails_the_run(self):
        j = Task1Judge(self.course, echo=False)
        drive(j, self.track(+1.0))
        v = j.verdict()
        self.assertFalse(v["pass"], format_verdict(v))
        self.assertTrue(v["buoys_detail"]["single_grn"].startswith("WRONG SIDE"))
        self.assertEqual(v["buoys_correct"], 4)

    def test_a_straight_line_down_the_course_fails_three_buoys(self):
        j = Task1Judge(self.course, echo=False)
        drive(j, [(0, 0)] + arc((10.0, 2.0), 6.0, 190.0, -400.0) + [(22, 0), (88, 0)])
        v = j.verdict()
        self.assertFalse(v["pass"])
        self.assertEqual(v["buoys_correct"], 2)                  # only the gate's two


class VerdictShape(unittest.TestCase):
    """The panel and gz_nav_test read these keys: they stay."""

    def test_the_old_keys_are_still_there(self):
        j = judge(buoy("r", 10.02, -3, "flash_red"), buoy("g", 10.02, 3, "flash_green"))
        drive(j, [(0, 0), (20, 0)])
        v = j.verdict()
        for key in ("pass", "gates_correct", "gates", "gates_detail", "circles", "contacts",
                    "min_clearance", "buoys_correct", "buoys", "buoys_detail"):
            self.assertIn(key, v)
        self.assertEqual(v["gates_detail"], {"r/g": "correct"})   # the informational gate line

    def test_the_verdict_line_the_nav_test_picks_up(self):
        j = judge(buoy("r", 10, -3, "flash_red"))
        drive(j, [(0, 0), (20, 0)])
        text = format_verdict(j.verdict())
        self.assertIn("VERDICT: PASS", text.splitlines()[0])
        self.assertIn("buoys 1/1 on their side", text.splitlines()[0])
        self.assertTrue(any("min clearance" in line for line in text.splitlines()))

    def test_contact_still_fails_a_run_with_every_side_right(self):
        j = judge(buoy("r", 10, -0.3, "flash_red"))             # 0.3 m abeam: touching
        drive(j, [(0, 0), (20, 0)])
        v = j.verdict()
        self.assertEqual(v["buoys_correct"], 1)
        self.assertEqual(v["contacts"], ["r"])
        self.assertFalse(v["pass"])


if __name__ == "__main__":
    unittest.main()
