"""sim_uav's field, through Ekko's Sender and the boat's Receiver: the plan the boat would
publish is the course's truth. No ROS, no pymavlink, no radio."""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [os.path.dirname(HERE), os.path.join(REPO, "crusader_link")]
from crusader_link import tunnel_link as tl  # noqa: E402
from crusader_link.ekko_map import EkkoMap  # noqa: E402
from crusader_sim import course as C  # noqa: E402
from crusader_sim.sim_uav import field_from_course  # noqa: E402


class SimUav(unittest.TestCase):

    def setUp(self):
        self.course = C.load("task1_core")
        self.field = field_from_course(self.course)

    def test_every_buoy_with_its_true_light(self):
        truth = C.buoys(self.course)
        self.assertEqual(len(self.field), len(truth))
        for (bid, _lat, _lon, label), (_n, _x, _y, state, _s, _u) in zip(self.field, truth):
            self.assertEqual(tl.CODE[label], C.BEACON_CODE[state])
        self.assertEqual([f[0] for f in self.field], list(range(1, len(truth) + 1)))

    def test_the_boat_hears_the_course_as_its_plan(self):
        sender, ekko = tl.Sender(first_epoch=7), EkkoMap()
        for t in (0.0, 1.0, 2.0):
            sender.feed(self.field, [], t, map_id="task1_core")
            for ptype, payload in sender.due(t):
                ekko.hear(ptype, payload)
        plan = ekko.plan()
        self.assertEqual(len(plan["buoys"]), len(self.field))
        entry = next(f for f in self.field if f[3] == "FLASHING_BLUE")
        exit_ = next(f for f in self.field if f[3] == "SOLID_BLUE")
        self.assertAlmostEqual(plan["entry"][0], entry[1], places=6)
        self.assertAlmostEqual(plan["exit"][1], exit_[2], places=6)


if __name__ == "__main__":
    unittest.main()
