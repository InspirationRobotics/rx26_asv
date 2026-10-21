"""crusader_link off the boat: the tunnel_link copy, ekko_map's plan and BOAT packet, and
rfd_link_node over a UDP loopback (that part needs rclpy, so it runs in the ROS container).

    python3 -m pytest crusader_link/test
"""
import hashlib
import os
import pathlib
import time
import unittest

from crusader_link import ekko_map
from crusader_link import tunnel_link as tl

HERE = pathlib.Path(__file__).resolve()
COPY = HERE.parents[1] / "crusader_link" / "tunnel_link.py"
UAV_FILE = pathlib.Path("uav_common", "uav_common", "boat_link.py")

ENTRY = (1.28063, 103.85583)
EXIT_ = (1.28060, 103.85620)
RED = (1.28057, 103.85592)
GREEN = (1.28063, 103.85592)
BLACK = (1.28068, 103.85600)


def feed(em, positions, lights, confirmed=(), epoch=7):
    """What Ekko's Sender puts on the air: POSITIONS (11 to a packet), then LIGHTS."""
    recs = [(s, bid, lat, lon) for s, (bid, lat, lon) in sorted(positions.items())]
    for p in tl.pack_positions(recs, epoch):
        em.hear(tl.PAYLOAD_POSITIONS, p)
    return em.hear(tl.PAYLOAD_LIGHTS, tl.pack_lights(lights, confirmed, epoch=epoch))


def field():
    pos = {1: (11, *ENTRY), 2: (12, *RED), 3: (13, *GREEN), 4: (14, *EXIT_), 5: (15, *BLACK)}
    lights = {1: "FLASHING_BLUE", 2: "FLASHING_RED", 3: "FLASHING_GREEN", 4: "SOLID_BLUE", 5: "OFF"}
    return pos, lights


def body_below_header(text):
    return text[text.index('"""boat_link'):]


class TunnelLinkCopy(unittest.TestCase):
    # sha256 of the copy below its header: an edit here fails this, a new upstream fails the next
    BODY_SHA256 = "e2abf1c6c400dd0a1b8d1da66ba6418f4d1b14bba40e38c7f4d90391183dcf16"

    def test_the_copy_is_unedited(self):
        body = body_below_header(COPY.read_text(encoding="utf-8"))
        self.assertEqual(hashlib.sha256(body.encode()).hexdigest(), self.BODY_SHA256,
                         "tunnel_link.py was edited below its header: re-copy boat_link.py")

    def test_same_bytes_as_rx26_uav(self):
        # $RX26_UAV, a checkout beside this repo, or the team's RobotX_2026/Drone/rx26_uav layout
        roots = [os.environ.get("RX26_UAV", ""), HERE.parents[3] / "rx26_uav",
                 HERE.parents[4] / "Drone" / "rx26_uav"]
        theirs = next((pathlib.Path(r) / UAV_FILE for r in roots
                       if r and (pathlib.Path(r) / UAV_FILE).is_file()), None)
        if theirs is None:
            self.skipTest("no rx26_uav checkout beside this one (or set RX26_UAV)")
        self.assertEqual(body_below_header(COPY.read_text(encoding="utf-8")),
                         theirs.read_text(encoding="utf-8"),
                         "tunnel_link.py has drifted from rx26_uav's boat_link.py: re-copy it")

    def test_the_numbers_the_boat_relies_on(self):
        self.assertEqual((tl.PAYLOAD_BOAT, tl.PAYLOAD_POSITIONS, tl.PAYLOAD_LIGHTS, tl.PAYLOAD_TEST),
                         (32768, 32772, 32773, 33022))
        self.assertEqual([tl.CODE[n] for n in ("UNKNOWN", "OFF", "FLASHING_RED", "FLASHING_GREEN",
                                               "FLASHING_BLUE", "SOLID_BLUE")], [0, 1, 2, 3, 4, 5])


class EkkoMapPlan(unittest.TestCase):
    def setUp(self):
        self.em = ekko_map.EkkoMap()

    def plan_after(self, positions, lights, **kw):
        feed(self.em, positions, lights, **kw)
        return self.em.plan()

    def test_positions_and_lights_become_the_plan(self):
        pos, lights = field()
        plan = self.plan_after(pos, lights)
        by_id = {b["id"]: b for b in plan["buoys"]}
        self.assertEqual(sorted(by_id), [1, 2, 3, 4, 5])
        self.assertEqual((by_id[2]["beacon"], by_id[3]["beacon"], by_id[5]["beacon"]), (2, 3, 1))
        self.assertAlmostEqual(plan["entry"][0], ENTRY[0], places=7)
        self.assertAlmostEqual(plan["exit"][1], EXIT_[1], places=7)

    def test_absent_exit_is_zero_zero_and_two_entries_lowest_slot_wins(self):
        pos, lights = field()
        del pos[4], lights[4]
        pos[6], lights[6] = (16, 1.2, 103.0), "FLASHING_BLUE"
        plan = self.plan_after(pos, lights)
        self.assertEqual(plan["exit"], (0.0, 0.0))
        self.assertAlmostEqual(plan["entry"][0], ENTRY[0], places=7)

    def test_unknown_lights_and_unplaced_lights_are_not_buoys(self):
        pos, lights = field()
        lights[3] = "UNKNOWN"
        lights[9] = "FLASHING_RED"
        self.assertEqual([b["id"] for b in self.plan_after(pos, lights)["buoys"]], [1, 2, 4, 5])

    def test_no_usable_buoy_is_no_plan(self):
        self.assertIsNone(self.em.plan())
        pos, lights = field()
        feed(self.em, pos, {s: "UNKNOWN" for s in lights})
        self.assertIsNone(self.em.plan())
        feed(self.em, {}, {}, epoch=8)                 # Ekko's "cleared"
        self.assertIsNone(self.em.plan())

    def test_a_new_map_number_discards_the_old_map(self):
        pos, lights = field()
        feed(self.em, pos, lights, epoch=7)
        feed(self.em, {1: (3, 1.5, 103.5)}, {1: "SOLID_BLUE"}, epoch=8)
        plan = self.em.plan()
        self.assertEqual(([b["id"] for b in plan["buoys"]], plan["entry"]), ([1], (0.0, 0.0)))

    def test_more_than_eleven_buoys_span_two_packets(self):
        pos = {s: (100 + s, 1.28 + s * 1e-5, 103.85) for s in range(1, 16)}
        self.assertEqual(len(self.plan_after(pos, {s: "FLASHING_RED" for s in pos})["buoys"]), 15)

    def test_malformed_payloads_do_not_raise(self):
        for ptype, payload in ((tl.PAYLOAD_LIGHTS, b""), (tl.PAYLOAD_POSITIONS, b"\x07\x05\x01"),
                               (32770, b"\x01\x02\x03"), (tl.PAYLOAD_TEST, b"hello")):
            self.assertIn(self.em.hear(ptype, payload), (None, "positions", "lights"))
        self.assertTrue(self.em.heard())


class PlanVersionTest(unittest.TestCase):
    def test_moves_on_change_only(self):
        v = ekko_map.PlanVersion()
        plan = {"entry": ENTRY, "exit": EXIT_, "buoys": [{"id": 1, "lat": 1.0, "lon": 2.0, "beacon": 2}]}
        self.assertTrue(v.update(plan))
        self.assertFalse(v.update(dict(plan)))
        self.assertEqual(v.version, 1)
        self.assertTrue(v.update(dict(plan, buoys=[{"id": 1, "lat": 1.0, "lon": 2.0, "beacon": 3}])))
        self.assertEqual(v.version, 2)


class BoatPacket(unittest.TestCase):
    def test_position_map_and_slots_held(self):
        em = ekko_map.EkkoMap()
        pos, lights = field()
        feed(em, pos, lights, epoch=7)
        got = tl.unpack_boat(em.boat_payload(1.2806, 103.8557))
        self.assertAlmostEqual(got["lat"], 1.2806, places=7)
        self.assertEqual((got["epoch"], got["acked"], got["activity"]), (7, {1, 2, 3, 4, 5}, 0))

    def test_before_any_map_it_acks_nothing(self):
        got = tl.unpack_boat(ekko_map.EkkoMap().boat_payload(1.0, 103.0))
        self.assertEqual((got["acked"], got["epoch"]), (set(), None))

    def test_ekkos_sender_accepts_it(self):
        s = tl.Sender(first_epoch=7)
        s.feed([(11, 1.0, 103.0, "FLASHING_RED"), (12, 1.1, 103.1, "SOLID_BLUE")], [], 0.0, "map-a")
        rx = ekko_map.EkkoMap()
        for ptype, payload in s.due(0.0):
            rx.hear(ptype, payload)
        got = tl.unpack_boat(rx.boat_payload(1.0, 103.0))
        s.boat_heard(got["acked"], 10.0, got["epoch"])
        self.assertEqual(s.acked, {1, 2})


try:
    import rclpy  # noqa: F401
    from crusader_msgs.msg import PassagePlan  # noqa: F401
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


@unittest.skipUnless(HAVE_ROS, "needs rclpy and crusader_msgs (the ROS container)")
class NodeOverLoopback(unittest.TestCase):
    """The real node on udpin; a fake Ekko (system 200) sends its v3 frames over UDP."""

    def test_ekko_frames_become_a_plan_and_the_boat_answers(self):
        os.environ["MAVLINK20"] = "1"
        os.environ.setdefault("ROS_DOMAIN_ID", "71")
        import rclpy
        from pymavlink import mavutil
        from crusader_msgs.msg import LatLonHead, PassagePlan, RadioFrame
        from crusader_link.rfd_link_node import RfdLinkNode

        rclpy.init(args=["--ros-args", "-p", "rfd_endpoint:=udpin:127.0.0.1:14599"])
        node = RfdLinkNode()
        probe = rclpy.create_node("probe")
        plans, frames = [], []
        probe.create_subscription(PassagePlan, "/crsd/passage_plan", plans.append, 10)
        probe.create_subscription(RadioFrame, "/crsd/radio/traffic", frames.append, 50)
        pose = probe.create_publisher(LatLonHead, "/crsd/pose", 10)
        ekko = mavutil.mavlink_connection("udpout:127.0.0.1:14599", source_system=200)
        sender = tl.Sender(first_epoch=3)
        sender.feed([(11, *ENTRY, "FLASHING_BLUE"), (12, *EXIT_, "SOLID_BLUE")], [], 0.0, "m")
        execs = rclpy.executors.SingleThreadedExecutor()
        execs.add_node(node)
        execs.add_node(probe)
        boat = None
        try:
            end = time.time() + 8.0
            while time.time() < end and (not plans or boat is None):
                for ptype, payload in sender.due(time.time()):
                    ekko.mav.send(ekko.mav.tunnel_encode(0, 0, ptype, len(payload), tl.pad(payload)))
                m = LatLonHead()
                m.latitude, m.longitude = 1.2806, 103.8557
                pose.publish(m)
                execs.spin_once(timeout_sec=0.05)
                reply = ekko.recv_match(type="TUNNEL", blocking=False)
                if reply is not None and reply.payload_type == tl.PAYLOAD_BOAT:
                    boat = tl.unpack_boat(tl.body(reply))
        finally:
            node.destroy_node()
            probe.destroy_node()
            rclpy.try_shutdown()
        self.assertTrue(plans, "no PassagePlan from Ekko's frames")
        self.assertAlmostEqual(plans[-1].entry_latitude, ENTRY[0], places=6)
        self.assertEqual(plans[-1].plan_version, 1)
        self.assertTrue(any(f.direction == RadioFrame.DIR_RX for f in frames))
        self.assertIsNotNone(boat, "no BOAT packet came back")
        self.assertEqual(boat["epoch"], 3)


if __name__ == "__main__":
    unittest.main()
