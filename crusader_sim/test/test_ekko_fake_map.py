"""Offline tests for ekko_fake_map, the sim course as the BuoyMap Ekko would publish.

No ROS, no sim. Needs python3 with PyYAML:

    cd rx26_asv/crusader_sim && python3 -m unittest discover -s test -p "test_ekko_fake_map.py" -v

The label check also runs Ekko's own Sender (a copy of boat_link in crusader_link)
over the map, when that package is beside this one.
"""
import contextlib
import io
import json
import math
import os
import re
import sys
import unittest

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(os.path.dirname(PKG), "crusader_link"))

from crusader_sim import course as C                                           # noqa: E402
from crusader_sim import ekko_fake_map as F                                    # noqa: E402

try:
    from crusader_link import tunnel_link as tl                                # noqa: E402
except ImportError:                                                            # pragma: no cover
    tl = None

M_PER_DEG_LAT = C.EARTH_M_PER_DEG


def metres(a, b):
    """Flat-earth distance between two (lat, lon): fine at these ranges."""
    dn = (a[0] - b[0]) * M_PER_DEG_LAT
    de = (a[1] - b[1]) * M_PER_DEG_LAT * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def mini_course(tier, buoys):
    """buoys: [(beacon, side_beacon, up_beacon)] placed on a diagonal."""
    return {"name": "mini", "tier": tier, "origin": {"lat": 1.2806, "lon": 103.8557},
            "elements": [{"type": "robobuoy", "name": "b%d" % i, "x": 10.0 * i, "y": -4.0 * i,
                          "beacon": b, "side_beacon": s, "up_beacon": u}
                         for i, (b, s, u) in enumerate(buoys)]}


class PositionsAreTheWorlds(unittest.TestCase):
    def test_every_buoy_is_enu_to_latlon_exactly(self):
        course = C.load("task1_core")
        msg = F.buoy_map(course)
        robobuoys = C.buoys(course)
        self.assertEqual(len(msg["buoys"]), len(robobuoys))
        self.assertEqual(len(robobuoys), 10)
        for b, (_name, x, y, *_rest) in zip(msg["buoys"], robobuoys):
            lat, lon = C.enu_to_latlon(x, y, course["origin"])
            self.assertEqual((b["latitude"], b["longitude"]), (lat, lon))

    def test_ids_are_one_to_n_in_course_order(self):
        msg = F.buoy_map(C.load("task1_core"))
        self.assertEqual([b["id"] for b in msg["buoys"]], list(range(1, 11)))

    def test_the_entry_is_east_of_and_north_of_the_origin_as_the_course_says(self):
        course = C.load("task1_core")
        entry = F.buoy_map(course)["buoys"][0]                       # x 12, y 3
        o = course["origin"]
        self.assertAlmostEqual((entry["latitude"] - o["lat"]) * M_PER_DEG_LAT, 3.0, places=6)
        self.assertGreater(entry["longitude"], o["lon"])


class WhatTheAircraftSees(unittest.TestCase):
    def test_core_tier_reads_the_side_beacon(self):
        c = mini_course("core", [("flash_red", True, False), ("flash_blue", False, True)])
        labels = [b["label"] for b in F.buoy_map(c)["buoys"]]
        self.assertEqual(labels, ["FLASHING_RED", "OFF"])          # the second one's side light is dark

    def test_advanced_and_disruptive_read_the_up_beacon(self):
        for tier in ("advanced", "disruptive"):
            c = mini_course(tier, [("flash_red", True, False), ("steady_blue", False, True)])
            labels = [b["label"] for b in F.buoy_map(c)["buoys"]]
            self.assertEqual(labels, ["OFF", "SOLID_BLUE"], tier)

    def test_a_missing_tier_is_core_and_unset_lights_are_lit(self):
        c = mini_course("core", [("flash_green", True, True)])
        del c["tier"]
        c["elements"][0].pop("side_beacon"), c["elements"][0].pop("up_beacon")
        self.assertEqual(F.buoy_map(c)["buoys"][0]["label"], "FLASHING_GREEN")

    def test_a_disruptive_course_shape(self):
        # side beacons off, up beacons on
        c = mini_course("disruptive", [("flash_blue", False, True), ("steady_blue", False, True),
                                       ("flash_red", False, True), ("flash_green", False, True),
                                       ("off", False, True)])
        self.assertEqual([b["label"] for b in F.buoy_map(c)["buoys"]],
                         ["FLASHING_BLUE", "SOLID_BLUE", "FLASHING_RED", "FLASHING_GREEN", "OFF"])

    def test_state_and_colour_agree_with_the_label(self):
        c = mini_course("core", [(k, True, True) for k in C.BEACON_STATES])
        for b in F.buoy_map(c)["buoys"]:
            want = "%s_%s" % (b["state"], b["colour"]) if b["colour"] else b["state"]
            self.assertEqual(b["label"], want)
            self.assertTrue(b["locked"])
        self.assertEqual({b["label"] for b in F.buoy_map(c)["buoys"]},
                         {"OFF", "FLASHING_RED", "FLASHING_GREEN", "FLASHING_BLUE", "SOLID_BLUE"})

    def test_evidence_fields_are_left_at_zero_not_invented(self):
        b = F.buoy_map(C.load("task1_core"))["buoys"][0]
        for k in ("lit_fraction", "colour_agreement", "samples", "flash_transitions",
                  "observed_s", "sightings", "spread_m", "last_seen_age_s"):
            self.assertEqual(b[k], 0, k)


@unittest.skipIf(tl is None, "crusader_link not beside crusader_sim")
class EkkosOwnPipelineAcceptsIt(unittest.TestCase):
    """Run the map through boat_link.Sender -- what telemetry_bridge does with a
    BuoyMap -- and read it back with the Receiver the boat uses."""

    def test_labels_are_strings_boat_link_knows_and_codes_match_the_course(self):
        c = mini_course("core", [(k, True, True) for k in C.BEACON_STATES])
        for b, k in zip(F.buoy_map(c)["buoys"], C.BEACON_STATES):
            self.assertIn(b["label"], tl.STATES)
            self.assertEqual(tl.CODE[b["label"]], C.BEACON_CODE[k])

    def test_a_sender_puts_the_whole_course_on_the_air_and_a_receiver_reads_it(self):
        course = C.load("task1_core")
        msg = F.buoy_map(course)
        sender = tl.Sender(first_epoch=5)
        sender.feed([(b["id"], b["latitude"], b["longitude"], b["label"]) for b in msg["buoys"]],
                    [], 0.0, msg["export_stem"])
        rx = tl.Receiver()
        for ptype, payload in sender.due(0.0):
            rx.hear(ptype, payload)
        held = rx.buoys()
        self.assertEqual(len(held), 10)
        by_id = {b["id"]: b for b in held}
        for b in msg["buoys"]:
            self.assertEqual(by_id[b["id"]]["label"], b["label"])
            self.assertLess(metres((by_id[b["id"]]["lat"], by_id[b["id"]]["lon"]),
                                   (b["latitude"], b["longitude"])), 0.02)   # 1e-7 deg on the wire
        self.assertEqual(rx.epoch, 5)


class Noise(unittest.TestCase):
    def test_zero_noise_is_exact_and_noise_stays_inside_the_radius(self):
        course = C.load("task1_core")
        exact = F.buoy_map(course)["buoys"]
        noisy = F.buoy_map(course, 0.5)["buoys"]
        moved = 0
        for e, n in zip(exact, noisy):
            d = metres((e["latitude"], e["longitude"]), (n["latitude"], n["longitude"]))
            self.assertLess(d, 0.5)
            moved += d > 1e-6
        self.assertGreater(moved, 5)

    def test_it_is_the_same_every_run(self):
        course = C.load("task1_core")
        self.assertEqual(F.buoy_map(course, 1.0), F.buoy_map(course, 1.0))
        self.assertNotEqual(F.buoy_map(course, 1.0), F.buoy_map(course, 0.0))

    def test_noise_does_not_touch_labels_or_ids(self):
        course = C.load("task1_core")
        a, b = F.buoy_map(course)["buoys"], F.buoy_map(course, 2.0)["buoys"]
        self.assertEqual([(x["id"], x["label"]) for x in a], [(x["id"], x["label"]) for x in b])


class Output(unittest.TestCase):
    def run_main(self, *argv):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = F.main(list(argv))
        self.assertEqual(rc, 0)
        return out.getvalue(), err.getvalue()

    def test_plain_output_is_the_message_as_yaml(self):
        out, _ = self.run_main("task1_core")
        msg = yaml.safe_load(out)
        self.assertEqual(msg["export_stem"], "fake-task1_core")
        self.assertEqual(len(msg["buoys"]), 10)
        self.assertEqual(msg["buoys"][0]["label"], "FLASHING_BLUE")
        self.assertEqual(msg["buoys"][7]["label"], "SOLID_BLUE")        # the exit

    def test_command_is_one_single_quoted_ros2_topic_pub_line(self):
        out, _ = self.run_main("task1_core", "--command")
        line = out.rstrip("\n")
        self.assertEqual(len(out.splitlines()), 1)
        m = re.fullmatch(r"ros2 topic pub -r 1 /uav/perception/buoy_map "
                         r"uav_msgs/msg/BuoyMap '([^']*)'", line)
        self.assertIsNotNone(m, line[:120])
        body = m.group(1)
        self.assertEqual(yaml.safe_load(body), F.buoy_map(C.load("task1_core")))
        self.assertEqual(json.loads(body)["export_stem"], "fake-task1_core")

    def test_the_summary_goes_to_stderr_so_the_command_stays_pasteable(self):
        out, err = self.run_main("task1_core", "--command", "--noise-m", "0.3")
        self.assertNotIn("ekko_fake_map:", out)
        self.assertIn("10 buoys, noise < 0.3 m", err)

    def test_every_checked_in_course_makes_a_map_ekko_can_carry(self):
        names = [f[:-5] for f in sorted(os.listdir(os.path.join(PKG, "courses")))
                 if f.endswith(".yaml")]
        for name in names:
            course = C.load(name)
            if not C.buoys(course):
                continue
            msg = F.buoy_map(course)
            self.assertLessEqual(len(msg["buoys"]), 31, name)           # boat_link.MAX_SLOT
            F.as_command(msg)                                           # asserts no quote inside


if __name__ == "__main__":
    unittest.main()
