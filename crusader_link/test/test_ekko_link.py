"""The boat's end of Ekko's v3 TUNNEL link, off-boat: tunnel_link (the copy of
Ekko's boat_link.py), ekko_map (plan, self-ack clock, BOAT packet), the codec's
TUNNEL branch and the node's own rules.

    cd rx26_asv/crusader_link && python3 -m unittest discover -s test -v

No radio, no ROS, no boat. The parts that need more are SKIPPED, not faked:
  * the codec test needs pymavlink (the generated dialect is in the package);
  * the node-rule tests need rclpy + crusader_msgs, i.e. run them in the crsd-sim
    container (`source install/setup.bash` first). They call the node's real
    methods on a stand-in `self`, so the rules tested are the shipped ones.
  * the drift test needs the Drone checkout beside this one, and passes quietly
    when it is not there.
"""
import os
import sys
import types
import unittest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))     # .../crusader_link
sys.path.insert(0, PKG)

from crusader_link import ekko_map                                      # noqa: E402
from crusader_link import tunnel_link as tl                             # noqa: E402

EKKO_COPY = os.path.join(PKG, "..", "..", "..", "Drone", "rx26_uav",
                         "uav_common", "uav_common", "boat_link.py")

# A field in the shape the drone sees it: ids are tracker ids (not slots), the
# slots 1.. are handed out in the order the positions were first sent.
ENTRY = (1.28063, 103.85583)
EXIT_ = (1.28060, 103.85620)
RED = (1.28057, 103.85592)
GREEN = (1.28063, 103.85592)
BLACK = (1.28068, 103.85600)


def feed(em, positions, lights, confirmed=(), epoch=7, now=100.0):
    """Hand EkkoMap what Ekko's Sender would put on the air: one POSITIONS
    packet per 11 buoys, then one LIGHTS. positions: {slot: (id, lat, lon)}."""
    recs = [(s, bid, lat, lon) for s, (bid, lat, lon) in sorted(positions.items())]
    for p in tl.pack_positions(recs, epoch):
        em.hear(tl.PAYLOAD_POSITIONS, p, now)
    return em.hear(tl.PAYLOAD_LIGHTS, tl.pack_lights(lights, confirmed, epoch=epoch), now)


def field():
    pos = {1: (11, *ENTRY), 2: (12, *RED), 3: (13, *GREEN), 4: (14, *EXIT_), 5: (15, *BLACK)}
    lights = {1: "FLASHING_BLUE", 2: "FLASHING_RED", 3: "FLASHING_GREEN",
              4: "SOLID_BLUE", 5: "OFF"}
    return pos, lights


class TunnelLinkIsEkkosCopy(unittest.TestCase):
    def test_same_bytes_as_the_drone_file_below_the_header(self):
        if not os.path.isfile(EKKO_COPY):
            self.skipTest("no Drone checkout beside this one")
        with open(os.path.join(PKG, "crusader_link", "tunnel_link.py"), encoding="utf-8") as f:
            mine = f.read()
        with open(EKKO_COPY, encoding="utf-8") as f:
            theirs = f.read()
        self.assertTrue(mine.endswith(theirs),
                        "tunnel_link.py has drifted from Ekko's boat_link.py: re-copy it "
                        "(and update the commit in its header)")

    def test_the_numbers_the_boat_relies_on(self):
        self.assertEqual((tl.PAYLOAD_BOAT, tl.PAYLOAD_POSITIONS, tl.PAYLOAD_LIGHTS,
                          tl.PAYLOAD_GNSS, tl.PAYLOAD_TEST), (32768, 32772, 32773, 32774, 33022))
        # = RoboNation's BeaconState and rxl_codec.BEACON_*
        self.assertEqual([tl.CODE[n] for n in ("UNKNOWN", "OFF", "FLASHING_RED",
                                               "FLASHING_GREEN", "FLASHING_BLUE", "SOLID_BLUE")],
                         [0, 1, 2, 3, 4, 5])

    def test_describe_still_names_the_test_frame_the_radio_tab_sends(self):
        self.assertEqual(tl.describe(tl.PAYLOAD_TEST, tl.pack_test("test 3 from crusader")),
                         ("TEST", "test 3 from crusader"))


class EkkoMapPlan(unittest.TestCase):
    def setUp(self):
        self.em = ekko_map.EkkoMap()

    def plan_after(self, positions, lights, **kw):
        feed(self.em, positions, lights, **kw)
        return self.em.plan()

    def test_v3_positions_and_lights_become_the_passage_plan(self):
        pos, lights = field()
        plan = self.plan_after(pos, lights)
        self.assertEqual(plan["msg"], "SAFE_PASSAGE")
        self.assertEqual(len(plan["buoys"]), 5)
        by_id = {b["id"]: b for b in plan["buoys"]}
        self.assertEqual(sorted(by_id), [1, 2, 3, 4, 5])              # the SLOTS
        self.assertEqual(by_id[2]["beacon"], 2)                        # red
        self.assertEqual(by_id[3]["beacon"], 3)                        # green
        self.assertEqual(by_id[5]["beacon"], 1)                        # off = an obstacle
        self.assertAlmostEqual(by_id[2]["lat"], RED[0], places=7)
        self.assertAlmostEqual(by_id[2]["lon"], RED[1], places=7)

    def test_entry_is_the_flashing_blue_and_exit_the_solid_blue(self):
        pos, lights = field()
        plan = self.plan_after(pos, lights)
        self.assertAlmostEqual(plan["entry"][0], ENTRY[0], places=7)
        self.assertAlmostEqual(plan["entry"][1], ENTRY[1], places=7)
        self.assertAlmostEqual(plan["exit"][0], EXIT_[0], places=7)
        self.assertAlmostEqual(plan["exit"][1], EXIT_[1], places=7)

    def test_an_absent_entry_or_exit_is_the_plans_unknown_zero_zero(self):
        pos, lights = field()
        del pos[4], lights[4]                                           # no SOLID_BLUE yet
        plan = self.plan_after(pos, lights)
        self.assertEqual(plan["exit"], (0.0, 0.0))
        self.assertNotEqual(plan["entry"], (0.0, 0.0))

    def test_two_entries_the_lowest_slot_wins(self):
        pos, lights = field()
        pos[6] = (16, 1.2, 103.0)
        lights[6] = "FLASHING_BLUE"
        plan = self.plan_after(pos, lights)
        self.assertAlmostEqual(plan["entry"][0], ENTRY[0], places=7)

    def test_a_buoy_whose_light_is_unknown_is_not_in_the_plan(self):
        pos, lights = field()
        lights[3] = "UNKNOWN"
        ids = [b["id"] for b in self.plan_after(pos, lights)["buoys"]]
        self.assertEqual(ids, [1, 2, 4, 5])

    def test_a_light_with_no_position_is_not_a_buoy(self):
        pos, lights = field()
        lights[9] = "FLASHING_RED"                                      # heard, never placed
        ids = [b["id"] for b in self.plan_after(pos, lights)["buoys"]]
        self.assertNotIn(9, ids)

    def test_a_buoy_missing_from_lights_is_dropped(self):
        pos, lights = field()
        feed(self.em, pos, lights)
        del lights[2]                                                    # Ekko merged it away
        self.em.hear(tl.PAYLOAD_LIGHTS, tl.pack_lights(lights, epoch=7), 101.0)
        self.assertNotIn(2, [b["id"] for b in self.em.plan()["buoys"]])

    def test_no_usable_buoy_is_no_plan_not_an_entry_at_zero_zero(self):
        self.assertIsNone(self.em.plan())                                # nothing heard
        pos, lights = field()
        feed(self.em, pos, {s: "UNKNOWN" for s in lights})
        self.assertIsNone(self.em.plan())                                # nothing decided
        feed(self.em, {}, {}, epoch=8)                                   # a Clear
        self.assertIsNone(self.em.plan())

    def test_a_new_map_number_discards_the_old_map(self):
        pos, lights = field()
        feed(self.em, pos, lights, epoch=7)
        self.assertEqual(self.em.epoch(), 7)
        # map 8: a single buoy, and the lights alone arrive first
        self.em.hear(tl.PAYLOAD_LIGHTS, tl.pack_lights({1: "SOLID_BLUE"}, epoch=8), 102.0)
        self.assertEqual(self.em.epoch(), 8)
        self.assertIsNone(self.em.plan())             # slot 1 has no position IN MAP 8
        feed(self.em, {1: (3, 1.5, 103.5)}, {1: "SOLID_BLUE"}, epoch=8)
        plan = self.em.plan()
        self.assertEqual([b["id"] for b in plan["buoys"]], [1])
        self.assertEqual(plan["entry"], (0.0, 0.0))                      # old entry did not survive
        self.assertAlmostEqual(plan["exit"][0], 1.5, places=7)

    def test_more_than_ten_buoys_are_not_capped(self):
        pos = {s: (100 + s, 1.28 + s * 1e-5, 103.85) for s in range(1, 16)}
        lights = {s: "FLASHING_RED" for s in pos}
        plan = self.plan_after(pos, lights)
        self.assertEqual(len(plan["buoys"]), 15)                         # two POSITIONS packets
        self.assertFalse(plan["truncated"])

    def test_lights_alone_do_not_hold_a_plan_up_the_boat_waits_for_positions(self):
        pos, lights = field()
        self.em.hear(tl.PAYLOAD_LIGHTS, tl.pack_lights(lights, epoch=7), 100.0)
        self.assertIsNone(self.em.plan())

    def test_foreign_and_malformed_payloads_do_not_raise(self):
        for ptype, payload in ((tl.PAYLOAD_LIGHTS, b""), (tl.PAYLOAD_POSITIONS, b"\x07"),
                               (tl.PAYLOAD_POSITIONS, b"\x07\x05\x01"), (32770, b"\x01\x02\x03"),
                               (tl.PAYLOAD_GNSS, tl.pack_gnss(4, 20, 0.02, 0.03)),
                               (tl.PAYLOAD_TEST, b"hello")):
            self.assertIn(self.em.hear(ptype, payload, 100.0), (None, "positions", "lights"))
        self.assertTrue(self.em.heard())


class SelfAckClock(unittest.TestCase):
    def test_fresh_within_five_seconds_of_a_lights_frame_only(self):
        em = ekko_map.EkkoMap()
        self.assertFalse(em.lights_fresh(100.0))                         # never heard
        pos, lights = field()
        feed(em, pos, lights, now=100.0)
        self.assertTrue(em.lights_fresh(100.0))
        self.assertTrue(em.lights_fresh(104.9))
        self.assertTrue(em.lights_fresh(105.0))
        self.assertFalse(em.lights_fresh(105.1))

    def test_positions_test_and_gnss_frames_do_not_make_lights_fresh(self):
        em = ekko_map.EkkoMap()
        for ptype, payload in ((tl.PAYLOAD_POSITIONS, tl.pack_positions([(1, 5, 1.2, 103.8)], 7)[0]),
                               (tl.PAYLOAD_TEST, b"hi"),
                               (tl.PAYLOAD_GNSS, tl.pack_gnss(4, 20, 0.02, 0.03))):
            em.hear(ptype, payload, 100.0)
        self.assertTrue(em.heard())
        self.assertFalse(em.lights_fresh(100.5))

    def test_a_newer_lights_frame_renews_it(self):
        em = ekko_map.EkkoMap()
        pos, lights = field()
        feed(em, pos, lights, now=100.0)
        feed(em, pos, lights, now=104.0)
        self.assertTrue(em.lights_fresh(108.9))
        self.assertFalse(em.lights_fresh(109.1))


class BoatPacket(unittest.TestCase):
    def test_what_ekko_waits_for_the_position_the_map_and_the_slots_held(self):
        em = ekko_map.EkkoMap()
        pos, lights = field()
        feed(em, pos, lights, epoch=7)
        got = tl.unpack_boat(em.boat_payload(1.2806, 103.8557))
        self.assertAlmostEqual(got["lat"], 1.2806, places=7)
        self.assertAlmostEqual(got["lon"], 103.8557, places=7)
        self.assertEqual(got["epoch"], 7)
        self.assertEqual(got["acked"], {1, 2, 3, 4, 5})
        self.assertEqual(got["activity"], 0)                             # "unknown", not a guess
        self.assertEqual(got["target_slot"], 0)

    def test_it_is_long_enough_for_ekkos_unguarded_rx_loop(self):
        em = ekko_map.EkkoMap()
        self.assertGreaterEqual(len(em.boat_payload(1.0, 103.0)), 14)

    def test_before_any_map_it_acks_nothing_and_names_no_map(self):
        got = tl.unpack_boat(ekko_map.EkkoMap().boat_payload(1.0, 103.0))
        self.assertEqual((got["acked"], got["epoch"]), (set(), None))

    def test_after_a_new_map_it_acks_only_the_new_maps_slots(self):
        em = ekko_map.EkkoMap()
        pos, lights = field()
        feed(em, pos, lights, epoch=7)
        feed(em, {1: (3, 1.5, 103.5)}, {1: "SOLID_BLUE"}, epoch=8)
        got = tl.unpack_boat(em.boat_payload(1.0, 103.0))
        self.assertEqual((got["epoch"], got["acked"]), (8, {1}))

    def test_ekkos_sender_accepts_it_as_the_acknowledgement(self):
        s = tl.Sender(first_epoch=7)
        s.feed([(11, 1.0, 103.0, "FLASHING_RED"), (12, 1.1, 103.1, "SOLID_BLUE")], [], 0.0, "map-a")
        rx = ekko_map.EkkoMap()
        for ptype, payload in s.due(0.0):
            rx.hear(ptype, payload, 0.0)
        got = tl.unpack_boat(rx.boat_payload(1.0, 103.0))
        s.boat_heard(got["acked"], 10.0, got["epoch"])
        self.assertEqual(s.acked, {1, 2})


try:
    from crusader_link import rxl_codec
    from pymavlink import mavutil                                        # noqa: F401
    HAVE_DIALECT = True
except Exception:                                                        # noqa: BLE001
    HAVE_DIALECT = False


@unittest.skipUnless(HAVE_DIALECT, "needs pymavlink")
class CodecTunnelBranch(unittest.TestCase):
    def wire(self, ptype, payload, src=200):
        import robotx as rxlink
        mav = rxlink.MAVLink(None, srcSystem=src, srcComponent=191)
        m = rxlink.MAVLink_tunnel_message(0, 0, ptype, len(payload), tl.pad(payload))
        return rxlink.MAVLink(None).decode(bytearray(m.pack(mav)))

    def test_lights_from_sysid_200_arrive_whole(self):
        pos, lights = field()
        payload = tl.pack_lights(lights, [2, 3], epoch=9)
        d = rxl_codec.decode(self.wire(tl.PAYLOAD_LIGHTS, payload))
        self.assertEqual((d["msg"], d["src"], d["payload_type"]),
                         ("TUNNEL", 200, tl.PAYLOAD_LIGHTS))
        self.assertEqual(tl.unpack_lights(d["payload"])["confirmed"], [2, 3])

    def test_end_to_end_bytes_to_plan(self):
        pos, lights = field()
        em = ekko_map.EkkoMap()
        for ptype, payload in [(tl.PAYLOAD_POSITIONS, p) for p in tl.pack_positions(
                [(s, b, la, lo) for s, (b, la, lo) in sorted(pos.items())], 3)] + [
                (tl.PAYLOAD_LIGHTS, tl.pack_lights(lights, epoch=3))]:
            d = rxl_codec.decode(self.wire(ptype, payload))
            em.hear(d["payload_type"], d["payload"], 1.0)
        self.assertEqual(len(em.plan()["buoys"]), 5)

    def test_a_trailing_zero_boat_packet_keeps_its_length(self):
        boat = tl.pack_boat(1.0, 103.0, 0, 0, acked=(), epoch=None)      # ends in zero bytes
        d = rxl_codec.decode(self.wire(tl.PAYLOAD_BOAT, boat, src=42))
        self.assertEqual(d["payload"], boat)

    def test_describe_names_every_v3_kind(self):
        for ptype, payload, name in (
                (tl.PAYLOAD_POSITIONS, tl.pack_positions([(1, 5, 1.2, 103.8)], 7)[0], "POSITIONS"),
                (tl.PAYLOAD_LIGHTS, tl.pack_lights({1: "OFF"}, epoch=7), "LIGHTS"),
                (tl.PAYLOAD_GNSS, tl.pack_gnss(4, 20, 0.02, 0.03), "GNSS"),
                (tl.PAYLOAD_BOAT, tl.pack_boat(1.2, 103.8, 0, epoch=7), "BOAT")):
            self.assertEqual(rxl_codec.describe(self.wire(ptype, payload))[0], name)


# ------------------------------------------------------------ the node's own rules

try:
    import rclpy                                                         # noqa: F401
    from crusader_link import rxl_link_node as node_mod
    HAVE_ROS = True
except Exception:                                                        # noqa: BLE001
    HAVE_ROS = False


class _Log:
    def __init__(self):
        self.lines = []

    def __getattr__(self, level):
        return lambda msg, *a, **k: self.lines.append((level, msg))


class _Clock:
    t = 0.0


def make_node(ekko, pose_t=None, peer=True):
    """A stand-in `self` carrying the node's REAL methods."""
    from crusader_common.stream_cache import StreamCache
    N = node_mod.RxlLinkNode
    n = types.SimpleNamespace(_ekko=ekko, _pose=StreamCache(1.0), _seen={},
                              sent=[], gates=[], plans=[], log=_Log(), peer=peer)
    n._changed = lambda key, value: N._changed(n, key, value)
    n.get_logger = lambda: n.log
    n._uav_sysid = lambda: 200
    n._send_tunnel = lambda dst, ptype, payload: (
        (n.sent.append((dst, ptype, payload)) or None) if n.peer else node_mod.NO_PEER)
    n._transmit = lambda dst, send: None if n.peer else node_mod.NO_PEER
    n._note_boat_sending = lambda s, why: N._note_boat_sending(n, s, why)
    n._publish_gate = lambda d, how="": n.gates.append((d, how))
    n._publish_plan = lambda d, source: n.plans.append((d, source))
    n._send_boat = lambda: N._send_boat(n)
    n._on_gate_reached = lambda msg: N._on_gate_reached(n, msg)
    n._on_ekko = lambda d: N._on_ekko(n, d)
    return n


@unittest.skipUnless(HAVE_ROS, "needs rclpy + crusader_msgs (run in the crsd-sim container)")
class NodeRules(unittest.TestCase):
    def setUp(self):
        self.clock = _Clock()
        self._real = node_mod.time.monotonic
        node_mod.time.monotonic = lambda: self.clock.t

    def tearDown(self):
        node_mod.time.monotonic = self._real

    def lights_frame(self, n, payload=None, ptype=tl.PAYLOAD_LIGHTS):
        pos, lights = field()
        if payload is None:
            payload = tl.pack_lights(lights, epoch=7)
        n._on_ekko({"payload_type": ptype, "payload": payload})

    # --- the BOAT packet -------------------------------------------------
    def test_nothing_is_sent_before_ekko_has_been_heard(self):
        n = make_node(ekko_map.EkkoMap())
        n._pose.set((1.28, 103.85), self.clock.t)
        n._send_boat()
        self.assertEqual(n.sent, [])

    def test_a_fresh_pose_and_a_heard_ekko_send_one_boat_packet_to_everyone(self):
        em = ekko_map.EkkoMap()
        pos, lights = field()
        feed(em, pos, lights, now=self.clock.t)
        n = make_node(em)
        n._pose.set((1.28060, 103.85570), self.clock.t)
        n._send_boat()
        self.assertEqual(len(n.sent), 1)
        dst, ptype, payload = n.sent[0]
        self.assertEqual((dst, ptype), (0, tl.PAYLOAD_BOAT))
        got = tl.unpack_boat(payload)
        self.assertAlmostEqual(got["lat"], 1.28060, places=7)
        self.assertEqual((got["epoch"], len(got["acked"])), (7, 5))

    def test_a_stale_or_missing_pose_sends_nothing_never_the_last_known_one(self):
        em = ekko_map.EkkoMap()
        pos, lights = field()
        feed(em, pos, lights, now=self.clock.t)
        n = make_node(em)
        n._send_boat()                                                   # no pose at all
        self.assertEqual(n.sent, [])
        n._pose.set((1.28060, 103.85570), self.clock.t)
        self.clock.t += 0.9
        n._send_boat()
        self.assertEqual(len(n.sent), 1)
        self.clock.t += 0.2                                              # pose now 1.1 s old
        n._send_boat()
        self.assertEqual(len(n.sent), 1)
        self.assertTrue(any(lvl.startswith("warn") for lvl, _ in n.log.lines))

    def test_the_log_says_when_it_starts_and_stops_not_every_second(self):
        em = ekko_map.EkkoMap()
        pos, lights = field()
        feed(em, pos, lights, now=self.clock.t)
        n = make_node(em)
        for _ in range(5):
            n._pose.set((1.28, 103.85), self.clock.t)
            n._send_boat()
            self.clock.t += 0.5
        self.assertEqual(len(n.log.lines), 1)

    # --- the self-ack ----------------------------------------------------
    def reached(self, n, seq=4):
        n._on_gate_reached(types.SimpleNamespace(data=seq))

    def test_gate_reached_is_answered_locally_while_lights_are_fresh(self):
        em = ekko_map.EkkoMap()
        pos, lights = field()
        feed(em, pos, lights, now=100.0)
        n = make_node(em)
        self.clock.t = 104.9
        self.reached(n, 4)
        self.assertEqual(len(n.gates), 1)
        self.assertEqual(n.gates[0][0], {"gate_seq": 4, "red_id": 255, "green_id": 255})

    def test_gate_reached_is_not_answered_once_the_lights_are_over_five_seconds_old(self):
        em = ekko_map.EkkoMap()
        pos, lights = field()
        feed(em, pos, lights, now=100.0)
        n = make_node(em)
        self.clock.t = 105.1
        self.reached(n, 4)
        self.assertEqual(n.gates, [])

    def test_gate_reached_with_no_ekko_at_all_is_left_to_the_native_handshake(self):
        n = make_node(ekko_map.EkkoMap())
        self.reached(n, 1)
        self.assertEqual(n.gates, [])

    def test_the_self_ack_does_not_depend_on_the_native_request_getting_out(self):
        em = ekko_map.EkkoMap()
        pos, lights = field()
        feed(em, pos, lights, now=100.0)
        n = make_node(em, peer=False)
        self.clock.t = 101.0
        self.reached(n, 2)
        self.assertEqual(len(n.gates), 1)

    # --- which frames publish a plan ---------------------------------------
    def test_every_lights_frame_publishes_a_plan_but_positions_do_not(self):
        n = make_node(ekko_map.EkkoMap())
        pos, lights = field()
        for p in tl.pack_positions([(s, b, la, lo) for s, (b, la, lo) in sorted(pos.items())], 7):
            n._on_ekko({"payload_type": tl.PAYLOAD_POSITIONS, "payload": p})
        self.assertEqual(n.plans, [])
        for _ in range(3):
            self.lights_frame(n)
            self.clock.t += 1.0
        self.assertEqual(len(n.plans), 3)
        self.assertEqual(n.plans[-1][1], node_mod.SRC_EKKO)
        self.assertEqual(len(n.plans[-1][0]["buoys"]), 5)

    def test_a_map_with_no_usable_buoy_publishes_nothing_and_says_so_once(self):
        n = make_node(ekko_map.EkkoMap())
        for _ in range(3):
            self.lights_frame(n, tl.pack_lights({}, epoch=8))
        self.assertEqual(n.plans, [])
        warns = [m for lvl, m in n.log.lines if lvl.startswith("warn")]
        self.assertEqual(len(warns), 1)


if __name__ == "__main__":
    unittest.main()
