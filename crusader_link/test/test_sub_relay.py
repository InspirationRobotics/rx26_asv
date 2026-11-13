"""Graey's leg through the boat: sub_relay's rules off the boat, the frames with pymavlink
where it is installed, and rfd_link_node over UDP loopbacks in the ROS container.

    python3 -m pytest crusader_link/test/test_sub_relay.py
"""
import os
import socket
import time
import unittest

from crusader_link.sub_relay import SubRelay, parse_target

GRAEY_CUBE, GRAEY_COMPANION, OCS, LAPTOP, EKKO = 3, 43, 250, 254, 200


class Target(unittest.TestCase):
    def test_off_unicast_and_broadcast(self):
        self.assertIsNone(parse_target(""))
        self.assertIsNone(parse_target("   "))
        self.assertEqual(parse_target("192.168.8.107:14573"), ("192.168.8.107", 14573))
        self.assertEqual(parse_target(" 192.168.8.255:14573 "), ("192.168.8.255", 14573))

    def test_a_typo_stops_the_node_with_the_reason(self):
        for bad in ("192.168.8.107", "ocs-laptop:14573", "192.168.8.107:0",
                    "192.168.8.107:70000", "224.0.0.1:14573", "0.0.0.0:14573", ":14573"):
            with self.assertRaises(ValueError, msg=bad):
                parse_target(bad)

    def test_never_a_robocommand_course_network(self):
        for third in (65, 66, 67, 68):
            with self.assertRaisesRegex(ValueError, "course network"):
                parse_target("192.168.%d.2:14573" % third)


class Rules(unittest.TestCase):
    def setUp(self):
        self.r = SubRelay([GRAEY_CUBE, GRAEY_COMPANION], GRAEY_COMPANION)

    def test_up_is_graeys_systems_and_nothing_else_on_the_mesh(self):
        self.assertTrue(self.r.up(GRAEY_COMPANION, 41, 1.0))
        self.assertTrue(self.r.up(GRAEY_CUBE, 21, 1.1))
        for other in (EKKO, 1, 2, 42, 51, OCS):
            self.assertFalse(self.r.up(other, 30, 1.2))
        self.assertEqual((self.r.up_frames, self.r.up_bytes), (2, 62))

    def test_heard_follows_the_last_relayed_frame(self):
        self.assertFalse(self.r.heard(0.0))
        self.r.up(GRAEY_COMPANION, 41, 10.0)
        self.assertTrue(self.r.heard(12.9))
        self.assertFalse(self.r.heard(13.1))

    def test_down_is_an_ocs_tunnel_for_graeys_companion_only(self):
        self.assertIsNone(self.r.down(OCS, "TUNNEL", GRAEY_COMPANION, 30, 0.0))
        self.assertIsNone(self.r.down(LAPTOP, "TUNNEL", GRAEY_COMPANION, 140, 0.0))
        self.assertEqual(self.r.down(OCS, "COMMAND_LONG", GRAEY_CUBE, 40, 0.0), "not a TUNNEL")
        self.assertEqual(self.r.down(255, "TUNNEL", GRAEY_COMPANION, 30, 0.0),
                         "source is not the OCS")
        for target in (0, EKKO, GRAEY_CUBE, 42):
            self.assertEqual(self.r.down(OCS, "TUNNEL", target, 30, 0.0),
                             "not addressed to Graey's companion")
        self.assertEqual((self.r.down_frames, self.r.down_bytes), (2, 170))
        self.assertEqual(sum(self.r.refused.values()), 6)

    def test_the_down_leg_cannot_flood_the_shared_air(self):
        r = SubRelay([GRAEY_COMPANION], GRAEY_COMPANION, bytes_per_s=100.0, burst_bytes=300.0)
        sent = sum(r.down(OCS, "TUNNEL", GRAEY_COMPANION, 100, 0.0) is None for _ in range(10))
        self.assertEqual(sent, 3)                           # the burst, then refusals
        self.assertEqual(r.refused, {"over the air budget": 7})
        self.assertIsNone(r.down(OCS, "TUNNEL", GRAEY_COMPANION, 100, 1.0))   # refilled 100 B
        self.assertIsNotNone(r.down(OCS, "TUNNEL", GRAEY_COMPANION, 100, 1.0))
        self.assertIsNone(r.down(OCS, "TUNNEL", GRAEY_COMPANION, 100, 60.0))  # never above burst
        self.assertEqual(sum(r.down(OCS, "TUNNEL", GRAEY_COMPANION, 100, 60.0) is None
                             for _ in range(5)), 2)

    def test_a_refused_frame_spends_no_budget(self):
        r = SubRelay([GRAEY_COMPANION], GRAEY_COMPANION, bytes_per_s=1.0, burst_bytes=100.0)
        for _ in range(50):
            r.down(255, "TUNNEL", GRAEY_COMPANION, 100, 0.0)
        self.assertIsNone(r.down(OCS, "TUNNEL", GRAEY_COMPANION, 100, 0.0))

    def test_misconfiguration_is_refused(self):
        with self.assertRaises(ValueError):
            SubRelay([GRAEY_CUBE], GRAEY_COMPANION)             # the companion is not Graey's
        with self.assertRaises(ValueError):
            SubRelay([GRAEY_COMPANION, OCS], GRAEY_COMPANION)   # a frame would loop


try:
    os.environ["MAVLINK20"] = "1"
    from pymavlink import mavutil
    HAVE_MAVLINK = True
except ImportError:
    HAVE_MAVLINK = False


def tunnel(src_system, src_component, target, ptype, payload):
    """One TUNNEL frame as that system would put it on the wire."""
    mav = mavutil.mavlink.MAVLink(None, srcSystem=src_system, srcComponent=src_component)
    msg = mav.tunnel_encode(target, 0, ptype, len(payload), payload + bytes(128 - len(payload)))
    return bytes(msg.pack(mav))


@unittest.skipUnless(HAVE_MAVLINK, "needs pymavlink")
class Frames(unittest.TestCase):
    def test_a_relayed_frame_is_the_same_bytes(self):
        # rfd_link_node relays msg.get_msgbuf(): it must be the frame as it arrived, or the
        # laptop's CRC check and its tunnel log would see something Graey never sent.
        raw = tunnel(GRAEY_COMPANION, 201, 0, 0x8010, bytes(range(24)))
        parser = mavutil.mavlink.MAVLink(None)
        parser.robust_parsing = True
        (msg,) = parser.parse_buffer(raw)
        self.assertEqual(bytes(msg.get_msgbuf()), raw)
        self.assertEqual((msg.get_srcSystem(), msg.get_type(), msg.target_system),
                         (GRAEY_COMPANION, "TUNNEL", 0))

    def test_run_start_from_the_ocs_passes_and_garbage_costs_one_datagram(self):
        # As rfd_link_node reads the laptop: one parser per datagram.
        def frames(datagram):
            parser = mavutil.mavlink.MAVLink(None)
            parser.robust_parsing = True
            return [m for m in parser.parse_buffer(datagram) or () if m.get_type() != "BAD_DATA"]

        relay = SubRelay([GRAEY_CUBE, GRAEY_COMPANION], GRAEY_COMPANION)
        raw = tunnel(OCS, 190, GRAEY_COMPANION, 0x8020, b"\x01\x02\x03")
        self.assertEqual(frames(b"\x00noise\xfd\x09"), [])     # a stray start byte: nothing, no raise
        for datagram in (raw, b"\x00noise" + raw, raw + raw):
            got = frames(datagram)
            self.assertEqual([bytes(m.get_msgbuf()) for m in got], [raw] * datagram.count(raw))
            for m in got:
                self.assertIsNone(relay.down(m.get_srcSystem(), m.get_type(), m.target_system,
                                             len(m.get_msgbuf()), 0.0))


try:
    import rclpy  # noqa: F401
    from crusader_msgs.msg import RadioFrame  # noqa: F401
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


@unittest.skipUnless(HAVE_ROS and HAVE_MAVLINK, "needs rclpy and crusader_msgs (the ROS container)")
class NodeOverLoopback(unittest.TestCase):
    """The real node: its radio is a udpin, a fake Graey keys it, a fake laptop is a UDP socket."""

    def test_graey_reaches_the_laptop_and_run_start_reaches_graey(self):
        os.environ.setdefault("ROS_DOMAIN_ID", "71")
        import rclpy
        from crusader_link.rfd_link_node import RfdLinkNode

        laptop = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        laptop.bind(("127.0.0.1", 0))
        laptop.settimeout(0.05)
        rclpy.init(args=["--ros-args", "-p", "rfd_endpoint:=udpin:127.0.0.1:14598",
                         "-p", "sub_relay_target:=127.0.0.1:%d" % laptop.getsockname()[1]])
        node = RfdLinkNode()
        execs = rclpy.executors.SingleThreadedExecutor()
        execs.add_node(node)
        graey = mavutil.mavlink_connection("udpout:127.0.0.1:14598", source_system=GRAEY_COMPANION,
                                           source_component=201)
        heartbeat = tunnel(GRAEY_COMPANION, 201, 0, 0x8010, bytes(range(24)))
        from_ekko = tunnel(EKKO, 191, 0, 33022, b"not Graey")
        run_start = tunnel(OCS, 190, GRAEY_COMPANION, 0x8020, b"\x01\x02\x03")
        for_ekko = tunnel(OCS, 190, EKKO, 0x8020, b"\x01\x02\x03")
        up, down, relay_addr = [], [], None
        try:
            end = time.time() + 8.0
            while time.time() < end and not (up and down):
                graey.write(from_ekko)                   # heard on the same radio: never relayed
                graey.write(heartbeat)
                execs.spin_once(timeout_sec=0.05)
                try:
                    data, relay_addr = laptop.recvfrom(4096)
                    up.append(data)
                except socket.timeout:
                    pass
                if relay_addr is not None:
                    laptop.sendto(for_ekko, relay_addr)  # refused: not Graey's companion
                    laptop.sendto(run_start, relay_addr)
                reply = graey.recv_match(type="TUNNEL", blocking=False)
                if reply is not None:
                    down.append(bytes(reply.get_msgbuf()))
            refused = dict(node._relay.refused)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
            laptop.close()
            graey.close()
        self.assertTrue(up, "nothing from Graey reached the laptop")
        self.assertEqual(set(up), {heartbeat}, "only Graey's frames, byte for byte")
        self.assertTrue(down, "the OCS's RunStart never reached Graey")
        self.assertEqual(set(down), {run_start}, "only what is addressed to Graey's companion")
        self.assertIn("not addressed to Graey's companion", refused)
