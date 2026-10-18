"""ocs_link off-boat: finding the OCS from its beacon, and nothing else changing.

    python3 -m unittest crusader_groundstation/test/test_ocs_link.py
    python3 -m unittest discover -s crusader_groundstation/test     (from rx26_asv)

No ROS, no OCS, no boat. Everything that opens a socket does so on loopback, on
ephemeral ports -- never 37564 or 37565, so it cannot collide with a running OCS
on the same machine. The pure halves (beacon parse, target choice, the course
network guard) take no socket at all.
"""
import ast
import contextlib
import importlib.util
import io
import os
import socket
import sys
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from crusader_groundstation import ocs_link as ol  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
NODE = os.path.join(REPO, "crusader_groundstation", "crusader_groundstation",
                    "ocs_client_node.py")
YAML = os.path.join(REPO, "crusader_bringup", "config", "crusader_params.yaml")
CHECK_CONFIG = os.path.join(REPO, "tools", "scripts", "check_config.py")


def beacon(team="ASTA", port=37564, v=1):
    return ('{"ocs_beacon":{"team":"%s","port":%d,"v":%d}}'
            % (team, port, v)).encode()


def wait_until(cond, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return cond()


def free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Acceptor:
    """A stand-in OCS: a loopback TCP listener that remembers who connected."""

    def __init__(self):
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(4)
        self.srv.settimeout(0.2)
        self.port = self.srv.getsockname()[1]
        self.conns = []
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                conn, _ = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.conns.append(conn)

    def drop_all(self):
        for c in self.conns:
            c.close()
        self.conns = []

    def close(self):
        self._stop.set()
        self.drop_all()
        self.srv.close()


def send_beacon(udp_port, data):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.sendto(data, ("127.0.0.1", udp_port))
    finally:
        s.close()


class ParseBeacon(unittest.TestCase):
    def test_a_v1_beacon_parses(self):
        self.assertEqual(ol.parse_beacon(beacon()),
                         {"team": "ASTA", "port": 37564, "v": 1})

    def test_it_reads_the_bytes_the_ocs_writes(self):
        # Byte-for-byte what OCS/rx_bridge/beacon.build("ASTA", 37564) emits.
        wire = b'{"ocs_beacon":{"team":"ASTA","port":37564,"v":1}}'
        self.assertEqual(ol.parse_beacon(wire)["port"], 37564)

    def test_anything_else_is_not_a_beacon(self):
        for data in (b"", b"junk", b"\xff\xfe", b"[]", b"{}", None, "text",
                     beacon(port=0), beacon(port=65536), beacon(v=2), beacon(team=""),
                     b'{"ocs_beacon":{"team":"ASTA","port":"37564","v":1}}',
                     b'{"ocs_beacon":{"team":"ASTA","port":true,"v":1}}',
                     b'{"ocs_beacon":{"team":"ASTA","port":37564}}',
                     b'{"ocs_beacon":{"team":"ASTA","port":37564,"v":1},"x":1}',
                     b'{"ocs_hello":{"team":"ASTA","port":37564,"v":1}}',
                     beacon() + b" " * 600):
            with self.subTest(data=data if data is None else data[:30]):
                self.assertIsNone(ol.parse_beacon(data))


class CourseNetworks(unittest.TestCase):
    def test_the_four_course_networks_and_only_those(self):
        for third in (65, 66, 67, 68):
            self.assertTrue(ol.on_course_network("192.168.%d.2" % third))
        for host in ("192.168.64.2", "192.168.69.2", "192.168.100.107",
                     "192.168.8.107", "127.0.0.1", "banana", ""):
            self.assertFalse(ol.on_course_network(host), host)

    def test_fallback_hosts_keep_only_usable_addresses_in_order(self):
        kept, problems = ol.check_fallback_hosts(
            ["192.168.100.107", "chaser.local", "192.168.65.184", "192.168.8.107",
             "192.168.100.107", " 10.0.0.5 "])
        self.assertEqual(kept, ["192.168.100.107", "192.168.8.107", "10.0.0.5"])
        self.assertEqual(len(problems), 2)
        self.assertIn("chaser.local", problems[0])
        self.assertIn("course network", problems[1])

    def test_a_bare_string_and_nothing_are_both_tolerated(self):
        self.assertEqual(ol.check_fallback_hosts("192.168.100.107"),
                         (["192.168.100.107"], []))
        self.assertEqual(ol.check_fallback_hosts(None), ([], []))
        self.assertEqual(ol.check_fallback_hosts([]), ([], []))


class BeaconFilter(unittest.TestCase):
    """BeaconListener.handle: the decision, with no socket."""

    def setUp(self):
        self.logs = []
        self.clock = FakeClock()
        self.chooser = ol.TargetChooser(["192.168.8.107"], 37564, clock=self.clock)
        self.listener = ol.BeaconListener(37565, "ASTA", self.chooser, self.logs.append)

    def test_our_teams_beacon_is_recorded_with_the_senders_ip_and_the_named_port(self):
        self.assertTrue(self.listener.handle(beacon(port=40000), "192.168.100.107"))
        self.assertEqual(self.chooser.choose(),
                         ("192.168.100.107", 40000, "beacon from 192.168.100.107"))
        self.assertEqual((self.listener.heard, self.listener.ignored), (1, 0))

    def test_another_teams_beacon_is_ignored_and_said_so_once(self):
        for _ in range(5):
            self.assertFalse(self.listener.handle(beacon(team="XYZ"), "192.168.100.50"))
        self.assertEqual(self.chooser.choose()[2], "fallback 192.168.8.107")
        self.assertEqual(self.listener.ignored, 5)
        self.assertEqual(len([m for m in self.logs if "'XYZ'" in m]), 1)

    def test_a_beacon_from_a_course_network_is_refused(self):
        for _ in range(3):
            self.assertFalse(self.listener.handle(beacon(), "192.168.65.184"))
        self.assertEqual(self.chooser.choose()[2], "fallback 192.168.8.107")
        self.assertEqual(len([m for m in self.logs if "course network" in m]), 1)

    def test_junk_is_ignored_silently(self):
        self.assertFalse(self.listener.handle(b"\x00\x01", "192.168.100.9"))
        self.assertEqual(self.logs, [])
        self.assertEqual(self.listener.ignored, 1)


class ChooseTarget(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()

    def chooser(self, fallback=("192.168.100.107", "192.168.8.107")):
        return ol.TargetChooser(list(fallback), 37564, clock=self.clock)

    def test_a_fresh_beacon_beats_the_fallback_list(self):
        c = self.chooser()
        c.hear("192.168.1.75", 37564)
        self.assertEqual(c.choose(), ("192.168.1.75", 37564, "beacon from 192.168.1.75"))

    def test_a_stale_beacon_is_ignored(self):
        c = self.chooser()
        c.hear("192.168.1.75", 37564)
        self.clock.t += ol.BEACON_FRESH_S + 0.1
        self.assertEqual(c.choose()[2], "fallback 192.168.100.107")

    def test_a_beacon_exactly_at_the_limit_still_counts(self):
        c = self.chooser()
        c.hear("192.168.1.75", 37564)
        self.clock.t += ol.BEACON_FRESH_S
        self.assertEqual(c.choose()[2], "beacon from 192.168.1.75")

    def test_the_newest_beacon_wins(self):
        c = self.chooser()
        c.hear("192.168.1.75", 37564)
        self.clock.t += 1
        c.hear("192.168.100.107", 40000)
        self.assertEqual(c.choose()[:2], ("192.168.100.107", 40000))

    def test_the_fallback_list_is_round_robin(self):
        c = self.chooser(["10.0.0.1", "10.0.0.2", "10.0.0.3"])
        got = [c.choose()[0] for _ in range(7)]
        self.assertEqual(got, ["10.0.0.1", "10.0.0.2", "10.0.0.3", "10.0.0.1",
                               "10.0.0.2", "10.0.0.3", "10.0.0.1"])
        self.assertEqual(c.choose()[1], 37564)             # the configured port

    def test_a_fallback_host_that_answered_is_tried_first_next_time(self):
        c = self.chooser(["10.0.0.1", "10.0.0.2", "10.0.0.3"])
        self.assertEqual(c.choose()[0], "10.0.0.1")
        self.assertEqual(c.choose()[0], "10.0.0.2")
        c.note_connected("10.0.0.2")
        self.assertEqual(c.choose()[0], "10.0.0.2")
        self.assertEqual(c.choose()[0], "10.0.0.3")

    def test_a_beacon_heard_while_waiting_between_fallbacks_takes_over(self):
        c = self.chooser()
        self.assertEqual(c.choose()[2], "fallback 192.168.100.107")
        c.hear("192.168.8.107", 37564)
        self.assertEqual(c.choose()[2], "beacon from 192.168.8.107")

    def test_nothing_to_try_is_none_not_a_guess(self):
        self.assertIsNone(self.chooser([]).choose())


class ExplicitHostIsUnchanged(unittest.TestCase):
    def setUp(self):
        self.srv = Acceptor()
        self.addCleanup(self.srv.close)
        self.logs = []

    def link(self, **kw):
        link = ol.OcsLink("127.0.0.1", self.srv.port, log=self.logs.append,
                          retry_s=0.1, **kw)
        self.addCleanup(link.stop)
        return link

    def test_it_dials_the_given_address_and_logs_what_it_always_did(self):
        link = self.link()
        link.start()
        self.assertTrue(wait_until(lambda: link.connected))
        self.assertIn("OCS link: connected to 127.0.0.1:%d" % self.srv.port, self.logs)
        self.assertFalse([m for m in self.logs if "target" in m])
        self.assertEqual(link.describe(), "127.0.0.1:%d" % self.srv.port)

    def test_a_discovery_port_is_ignored_so_nothing_listens(self):
        port = free_udp_port()
        link = self.link(team="ASTA", discovery_port=port, fallback_hosts=["10.0.0.9"])
        link.start()
        self.assertTrue(wait_until(lambda: link.connected))
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(probe.close)
        probe.bind(("0.0.0.0", port))               # would raise if anything held it

    def test_a_beacon_does_not_move_a_fixed_link(self):
        other = Acceptor()
        self.addCleanup(other.close)
        port = free_udp_port()
        link = self.link(team="ASTA", discovery_port=port)
        link.start()
        self.assertTrue(wait_until(lambda: link.connected))
        send_beacon(port, beacon(port=other.port))
        time.sleep(0.4)
        self.assertEqual(other.conns, [])

    def test_the_constructor_still_takes_exactly_what_it_used_to(self):
        link = ol.OcsLink("127.0.0.1", 1, on_command=lambda c: None,
                          log=lambda m: None, retry_s=1.0)
        self.assertEqual((link.host, link.port, link.retry_s), ("127.0.0.1", 1, 1.0))


class AutoOverLoopback(unittest.TestCase):
    """The real listener, the real link thread, two stand-in OCS servers."""

    def setUp(self):
        self.a, self.b = Acceptor(), Acceptor()
        self.addCleanup(self.a.close)
        self.addCleanup(self.b.close)
        self.udp = free_udp_port()
        self.logs = []

    def link(self, fallback=(), **kw):
        link = ol.OcsLink("auto", 1, team="ASTA", discovery_port=self.udp,
                          fallback_hosts=list(fallback), log=self.logs.append,
                          retry_s=0.1, **kw)
        self.addCleanup(link.stop)
        link.start()
        self.assertTrue(wait_until(lambda: link._listener.listening))
        return link

    def test_it_dials_the_beacons_sender_at_the_port_the_beacon_names(self):
        link = self.link()
        time.sleep(0.3)
        self.assertFalse(link.connected)                   # nothing to dial yet
        send_beacon(self.udp, beacon(port=self.a.port))
        self.assertTrue(wait_until(lambda: len(self.a.conns) == 1))
        self.assertTrue(wait_until(lambda: link.connected))
        self.assertEqual([m for m in self.logs if "target ->" in m],
                         ["OCS link: target -> 127.0.0.1:%d (beacon from 127.0.0.1)"
                          % self.a.port])

    def test_it_stays_on_its_host_while_connected_and_moves_only_after_a_drop(self):
        link = self.link()
        send_beacon(self.udp, beacon(port=self.a.port))
        self.assertTrue(wait_until(lambda: len(self.a.conns) == 1))
        self.assertTrue(wait_until(lambda: link.connected))

        # A second OCS starts beaconing. The boat is connected, so: no flapping.
        for _ in range(5):
            send_beacon(self.udp, beacon(port=self.b.port))
            time.sleep(0.1)
        self.assertEqual(self.b.conns, [])
        self.assertTrue(link.connected)
        self.assertEqual(len(self.a.conns), 1)

        # The first OCS goes away; only now does the boat choose again.
        self.a.drop_all()
        self.assertTrue(wait_until(lambda: len(self.b.conns) == 1))
        self.assertEqual([m for m in self.logs if "target ->" in m],
                         ["OCS link: target -> 127.0.0.1:%d (beacon from 127.0.0.1)" % self.a.port,
                          "OCS link: target -> 127.0.0.1:%d (beacon from 127.0.0.1)" % self.b.port])

    def test_another_teams_beacon_never_connects(self):
        link = self.link()
        send_beacon(self.udp, beacon(team="XYZ", port=self.a.port))
        time.sleep(0.5)
        self.assertFalse(link.connected)
        self.assertEqual(self.a.conns, [])

    def test_with_no_beacon_it_falls_back_and_says_why(self):
        link = ol.OcsLink("auto", self.a.port, team="ASTA", discovery_port=0,
                          fallback_hosts=["127.0.0.1"], log=self.logs.append,
                          retry_s=0.1)
        self.addCleanup(link.stop)
        link.start()
        self.assertTrue(wait_until(lambda: link.connected))
        self.assertIn("OCS link: target -> 127.0.0.1:%d (fallback 127.0.0.1)"
                      % self.a.port, self.logs)

    def test_a_dead_fallback_host_is_followed_by_the_next_one(self):
        # The stand-in OCS listens on 127.0.0.1 only, so the first dial is refused.
        link = ol.OcsLink("auto", self.a.port, team="ASTA", discovery_port=0,
                          fallback_hosts=["127.0.0.2", "127.0.0.1"], log=self.logs.append,
                          retry_s=0.1)
        self.addCleanup(link.stop)
        link.start()
        self.assertTrue(wait_until(lambda: link.connected, 8.0))
        targets = [m for m in self.logs if "target ->" in m]
        self.assertEqual(targets[0].split("(")[1], "fallback 127.0.0.2)")
        self.assertEqual(targets[-1].split("(")[1], "fallback 127.0.0.1)")

    def test_nowhere_to_look_waits_and_says_so_once(self):
        link = ol.OcsLink("auto", 1, team="ASTA", discovery_port=0,
                          fallback_hosts=[], log=self.logs.append, retry_s=0.05)
        self.addCleanup(link.stop)
        link.start()
        time.sleep(0.4)
        self.assertFalse(link.connected)
        self.assertEqual(len([m for m in self.logs if "waiting for a beacon" in m]), 1)

    def test_a_course_network_fallback_is_dropped_with_a_log_line(self):
        link = ol.OcsLink("auto", 1, team="ASTA", discovery_port=0,
                          fallback_hosts=["192.168.65.2", "192.168.100.107"],
                          log=self.logs.append)
        self.assertIn("192.168.100.107", link.describe())
        self.assertNotIn("192.168.65.2", link.describe())
        self.assertEqual(len([m for m in self.logs if "course network" in m]), 1)

    def test_a_taken_discovery_port_costs_the_listener_not_the_link(self):
        squatter = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(squatter.close)
        squatter.bind(("0.0.0.0", self.udp))
        link = ol.OcsLink("auto", self.a.port, team="ASTA", discovery_port=self.udp,
                          fallback_hosts=["127.0.0.1"], log=self.logs.append,
                          retry_s=0.1)
        self.addCleanup(link.stop)
        link.start()
        self.assertTrue(wait_until(lambda: link.connected))
        self.assertTrue([m for m in self.logs if "cannot listen for beacons" in m])
        self.assertIn("NOT RUNNING", link.describe())

    def test_auto_with_a_listener_needs_the_team_code(self):
        with self.assertRaises(ValueError):
            ol.OcsLink("auto", 37564, discovery_port=37565)
        ol.OcsLink("auto", 37564, discovery_port=0, fallback_hosts=["10.0.0.1"])

    def test_auto_is_case_and_space_tolerant(self):
        link = ol.OcsLink("  AUTO ", 37564, team="ASTA", discovery_port=0,
                          fallback_hosts=["10.0.0.1"])
        self.assertTrue(link.describe().startswith("auto"))


class ShippedConfig(unittest.TestCase):
    """The node and the params file must agree, or the node dies at startup.

    ocs_client_node cannot be imported without rclpy, so its PARAM_SPEC is read
    from the source: a YAML key with no entry there is a ValueError from
    declare_from_config, which is exactly what a pull without a rebuild produces.
    """

    @classmethod
    def setUpClass(cls):
        try:
            import yaml
        except ImportError:
            raise unittest.SkipTest("PyYAML not installed")
        with open(NODE, encoding="utf-8") as fh:
            tree = ast.parse(fh.read())
        cls.spec = None
        for node in tree.body:
            if (isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
                    and node.targets[0].id == "PARAM_SPEC"):
                cls.spec = {k.value for k in node.value.keys}
        with open(YAML, encoding="utf-8") as fh:
            cls.params = yaml.safe_load(fh)["ocs_client"]["ros__parameters"]

    def test_every_yaml_key_has_a_declared_posture_and_the_reverse(self):
        self.assertEqual(sorted(set(self.params) - self.spec), [])
        self.assertEqual(sorted(self.spec - set(self.params)), [])

    def test_the_shipped_values_are_the_auto_discovery_setup(self):
        p = self.params
        self.assertEqual(p["ocs_host"], "auto")
        self.assertEqual(p["ocs_discovery_port"], 37565)
        self.assertEqual(p["ocs_fallback_hosts"], ["192.168.100.107", "192.168.8.107"])
        self.assertEqual(ol.check_fallback_hosts(p["ocs_fallback_hosts"])[1], [])
        self.assertEqual(ol.DEFAULT_FALLBACK_HOSTS, tuple(p["ocs_fallback_hosts"]))
        self.assertEqual(ol.DEFAULT_DISCOVERY_PORT, p["ocs_discovery_port"])

    def test_the_beacon_team_is_the_nodes_team_id(self):
        self.assertEqual(self.params["team_id"], "ASTA")

    def test_nothing_else_in_the_repo_names_the_discovery_port(self):
        hits = []
        for root, dirs, files in os.walk(REPO):
            dirs[:] = [d for d in dirs if d not in ("__pycache__", ".git")]
            for name in files:
                if not name.endswith((".py", ".yaml", ".yml", ".sh", ".service",
                                      ".cfg", ".xml")):
                    continue
                path = os.path.join(root, name)
                if os.path.samefile(path, __file__):
                    continue
                with open(path, encoding="utf-8", errors="replace") as fh:
                    if "37565" in fh.read():
                        hits.append(os.path.relpath(path, REPO))
        self.assertEqual(sorted(hits), sorted([
            os.path.join("crusader_bringup", "config", "crusader_params.yaml"),
            os.path.join("crusader_groundstation", "crusader_groundstation",
                         "ocs_link.py"),
        ]))


class CheckConfigRule(unittest.TestCase):
    """tools/scripts/check_config.py check_ocs_client, against synthetic params."""

    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("check_config_under_test",
                                                      CHECK_CONFIG)
        cls.mod = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(cls.mod)
        except ImportError as exc:
            raise unittest.SkipTest("check_config needs %s" % exc)

    def run_check(self, **params):
        base = {"ocs_host": "auto", "ocs_port": 37564, "ocs_discovery_port": 37565,
                "ocs_fallback_hosts": ["192.168.100.107", "192.168.8.107"]}
        base.update(params)
        self.mod.failures[:] = []
        with contextlib.redirect_stdout(io.StringIO()):
            self.mod.check_ocs_client({"ocs_client": {"ros__parameters": base}})
        return [detail for _check, detail in self.mod.failures]

    def test_the_shipped_shape_passes(self):
        self.assertEqual(self.run_check(), [])

    def test_a_fixed_address_passes_and_ignores_the_discovery_keys(self):
        self.assertEqual(self.run_check(ocs_host="192.168.8.107"), [])

    def test_a_course_address_is_refused_wherever_it_appears(self):
        self.assertTrue(self.run_check(ocs_host="192.168.65.2"))
        self.assertTrue(self.run_check(ocs_fallback_hosts=["192.168.100.107",
                                                           "192.168.67.9"]))

    def test_a_hostname_fallback_is_refused(self):
        self.assertTrue(self.run_check(ocs_fallback_hosts=["chaser.local"]))

    def test_bad_discovery_ports(self):
        for port in (-1, 80, 14551, 37564, "37565", True):
            with self.subTest(port=port):
                self.assertTrue(self.run_check(ocs_discovery_port=port))
        self.assertEqual(self.run_check(ocs_discovery_port=0), [])

    def test_a_link_with_nowhere_to_look_is_refused(self):
        self.assertTrue(self.run_check(ocs_discovery_port=0, ocs_fallback_hosts=[]))
        self.assertTrue(self.run_check(ocs_fallback_hosts=[]))

    def test_a_repeated_fallback_is_refused(self):
        self.assertTrue(self.run_check(ocs_fallback_hosts=["10.0.0.1", "10.0.0.1"]))


if __name__ == "__main__":
    unittest.main()
