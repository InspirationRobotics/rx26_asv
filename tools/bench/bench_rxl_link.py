#!/usr/bin/env python3
"""bench_rxl_link — prove the UAV link end to end, with no UAV and no radio.

    ros2 run crusader_link rxl_link_node --ros-args --params-file <cfg>   # term 1
    python3 tools/bench/bench_rxl_link.py                                 # term 2

Stands in for the aircraft: opens a udpout to the port rxl_link_node is bound
to, sends real RXL frames encoded by the real dialect, and then checks what came
out the ROS side.

WHY IT COMPARES FIELD BY FIELD. The failure this exists to catch is not "no
message arrived" -- that is obvious. It is the MAVLink field-order trap: fields
travel sorted by descending type width, not declaration order, and pymavlink
returns `fieldtypes` and `array_lengths` in different orders again. Both
mistakes yield byte offsets that decode to numbers that still look like
coordinates. So the values sent here are deliberately ASYMMETRIC -- entry and
exit differ in every digit, the buoys have distinct positions and distinct
colours -- because symmetric test data is exactly what a transposed decode
survives.

It also proves the reverse path, which nothing else does: publishing
/crsd/gate_reached must put an RXL_USV_REACHED_GATE back on the wire, carrying
the same gate_seq. Without that the handshake is one-way and the boat would wait
forever for a pair it never asked for.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

from crusader_msgs.msg import GatePair, PassagePlan

from crusader_link import rxl_codec

# Asymmetric on purpose. Every one of these differs from every other in a way a
# transposed or shifted decode cannot reproduce.
ENTRY = (1.28120, 103.855700)
EXIT = (1.28204, 103.855755)
BUOYS = [
    # id, lat, lon, beacon
    (0, 1.281401, 103.855699, rxl_codec.BEACON_FLASHING_BLUE),
    (1, 1.281563, 103.855753, rxl_codec.BEACON_FLASHING_RED),
    (2, 1.281564, 103.855645, rxl_codec.BEACON_FLASHING_GREEN),
    (3, 1.281626, 103.855825, rxl_codec.BEACON_OFF),
    (4, 1.282049, 103.855710, rxl_codec.BEACON_STEADY_BLUE),
]
GATE_SEQ = 3
RED_ID, GREEN_ID = 1, 2


class Bench(Node):
    def __init__(self, cfg):
        super().__init__("bench_rxl_link")
        self.plan = None
        self.gate = None
        self.create_subscription(PassagePlan, cfg.plan_topic, self._plan, 10)
        self.create_subscription(GatePair, cfg.gate_topic, self._gate, 10)
        self.reached = self.create_publisher(UInt8, cfg.gate_reached_topic, 10)

    def _plan(self, m):
        self.plan = m

    def _gate(self, m):
        self.gate = m


def spin_until(node, pred, timeout_s=8.0):
    """Spin until pred() or the timeout. Returns whether pred came true."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if pred():
            return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--endpoint", default="udpout:127.0.0.1:14555",
                    help="where rxl_link_node is listening (its rxl_endpoint)")
    ap.add_argument("--plan-topic", dest="plan_topic", default="crsd/passage_plan")
    ap.add_argument("--gate-topic", dest="gate_topic", default="crsd/next_gate")
    ap.add_argument("--gate-reached-topic", dest="gate_reached_topic",
                    default="crsd/gate_reached")
    cfg = ap.parse_args()

    fails = []

    def chk(name, ok, detail=""):
        print("  [%s] %s%s" % ("ok" if ok else "FAIL", name,
                               "" if ok else "  <- " + detail))
        if not ok:
            fails.append(name)

    rclpy.init()
    node = Bench(cfg)

    # The aircraft's side of the link. source_system must differ from the
    # boat's or an inspector files both vehicles under one tree.
    uav = rxl_codec.connect(cfg.endpoint, source_system=200)
    print("bench UAV -> %s\n" % cfg.endpoint)

    # ------------------------------------------------------- SAFE_PASSAGE
    lat = [rxl_codec.e7(b[1]) for b in BUOYS] + [0] * (10 - len(BUOYS))
    lon = [rxl_codec.e7(b[2]) for b in BUOYS] + [0] * (10 - len(BUOYS))
    col = [b[3] for b in BUOYS] + [0] * (10 - len(BUOYS))
    uav.mav.rxl_safe_passage_send(
        int(time.time() * 1e3) & 0xFFFFFFFF,
        rxl_codec.e7(ENTRY[0]), rxl_codec.e7(ENTRY[1]),
        rxl_codec.e7(EXIT[0]), rxl_codec.e7(EXIT[1]),
        len(BUOYS), lat, lon, col)

    got = spin_until(node, lambda: node.plan is not None)
    chk("a PassagePlan reached ROS", got,
        "is rxl_link_node running, and bound to this port?")
    if got:
        p = node.plan
        chk("entry latitude survives", abs(p.entry_latitude - ENTRY[0]) < 1e-7,
            "got %.7f want %.7f" % (p.entry_latitude, ENTRY[0]))
        chk("entry longitude survives", abs(p.entry_longitude - ENTRY[1]) < 1e-7,
            "got %.7f want %.7f" % (p.entry_longitude, ENTRY[1]))
        chk("exit is not the entry", abs(p.exit_latitude - EXIT[0]) < 1e-7,
            "got %.7f want %.7f" % (p.exit_latitude, EXIT[0]))
        chk("exit longitude survives", abs(p.exit_longitude - EXIT[1]) < 1e-7,
            "got %.7f want %.7f" % (p.exit_longitude, EXIT[1]))
        chk("buoy count survives", len(p.buoys) == len(BUOYS),
            "got %d want %d" % (len(p.buoys), len(BUOYS)))

        for want in BUOYS:
            wid, wlat, wlon, wcol = want
            hit = [b for b in p.buoys if b.id == wid]
            if not hit:
                chk("buoy %d present" % wid, False, "missing")
                continue
            b = hit[0]
            chk("buoy %d position" % wid,
                abs(b.latitude - wlat) < 1e-7 and abs(b.longitude - wlon) < 1e-7,
                "got %.6f,%.6f want %.6f,%.6f" % (b.latitude, b.longitude, wlat, wlon))
            chk("buoy %d beacon" % wid, b.beacon == wcol,
                "got %d want %d" % (b.beacon, wcol))

        chk("plan_version starts at 1", p.plan_version == 1,
            "got %d" % p.plan_version)
        # The stamp is RECEIPT time, so it must be close to now, not whatever
        # clock the sender had.
        age = abs(time.time() - (p.header.stamp.sec + p.header.stamp.nanosec / 1e9))
        chk("stamp is receipt time, not the sender's", age < 5.0, "%.1fs off" % age)

    # -------------------------------------------------- a SECOND plan bumps
    node.plan = None
    uav.mav.rxl_safe_passage_send(
        int(time.time() * 1e3) & 0xFFFFFFFF,
        rxl_codec.e7(ENTRY[0]), rxl_codec.e7(ENTRY[1]),
        rxl_codec.e7(EXIT[0]), rxl_codec.e7(EXIT[1]),
        len(BUOYS), lat, lon, col)
    if spin_until(node, lambda: node.plan is not None):
        chk("a second plan bumps plan_version", node.plan.plan_version == 2,
            "got %d — Disruptive re-tasking hangs off this" % node.plan.plan_version)
    else:
        chk("a second plan bumps plan_version", False, "no second plan arrived")

    # ------------------------------------------------------ NEXT_BUOY_SET
    uav.mav.rxl_next_buoy_set_send(
        int(time.time() * 1e3) & 0xFFFFFFFF, GATE_SEQ, [RED_ID, GREEN_ID])
    got = spin_until(node, lambda: node.gate is not None)
    chk("a GatePair reached ROS", got)
    if got:
        g = node.gate
        chk("gate_seq survives", g.gate_seq == GATE_SEQ, "got %d" % g.gate_seq)
        chk("index 0 became red_id", g.red_id == RED_ID, "got %d" % g.red_id)
        chk("index 1 became green_id", g.green_id == GREEN_ID, "got %d" % g.green_id)

    # the end-of-passage sentinel
    node.gate = None
    uav.mav.rxl_next_buoy_set_send(
        int(time.time() * 1e3) & 0xFFFFFFFF, GATE_SEQ + 1,
        [rxl_codec.NO_BUOY, rxl_codec.NO_BUOY])
    if spin_until(node, lambda: node.gate is not None):
        g = node.gate
        chk("the 255/255 end-of-passage pair arrives intact",
            g.red_id == rxl_codec.NO_BUOY and g.green_id == rxl_codec.NO_BUOY,
            "got %d/%d" % (g.red_id, g.green_id))
    else:
        chk("the 255/255 end-of-passage pair arrives intact", False, "nothing arrived")

    # ------------------------------------------------- the REVERSE direction
    # Nothing else proves this leg. If it is broken the boat clears a gate,
    # the aircraft never hears, and the mission stalls looking healthy.
    uav.mav.rxl_usv_reached_gate_send(0, 0)     # prime: give the node an address
    time.sleep(0.3)
    m = UInt8()
    m.data = 7
    node.reached.publish(m)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.05)

    heard = None
    end = time.monotonic() + 3.0
    while time.monotonic() < end and heard is None:
        got = uav.recv_match(blocking=False)
        if got is not None:
            d = rxl_codec.decode(got)
            if d and d["msg"] == "USV_REACHED_GATE" and d["gate_seq"] == 7:
                heard = d
        time.sleep(0.02)
    chk("gate_reached(7) came back on the wire", heard is not None,
        "the boat asked for a pair and the aircraft never heard it")

    node.destroy_node()
    rclpy.try_shutdown()
    print("\n%s" % ("PASS" if not fails else "FAIL: %d of the above" % len(fails)))
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
