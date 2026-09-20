#!/usr/bin/env python3
"""bench_gate_pairs — the confirmation handshake, without ROS or a radio peer.

    python3 tools/bench/bench_gate_pairs.py

Exercises UavLink's half of the protocol only; the boat's half -- pairing the
buoys and ordering the gates -- is nav::planPassage and is covered off-ROS by
crusader_bt/test/test_nav_math.cpp.

WHAT THE PROTOCOL IS NOW. The aircraft transmits ten buoys with their colours
and nothing else. The boat pairs them, picks the order, drives a gate, and then
asks "is the rest of the field still where you said it was?". The aircraft
answers with the field as it stands plus an acknowledgement of that sequence
number.

THE THREE RULES THAT ARE EASY TO BREAK, and why each is here:

  * CONFIRMING IS AN OPERATOR ACTION. The whole point is that a human may be
    about to change the field, so poll() must NOT answer on its own. Scripts
    opt into auto_confirm.

  * A RE-ASK IS ANSWERED IMMEDIATELY. That is the lost-reply case on a lossy
    radio and it must not wait on the operator a second time, or every dropped
    packet becomes a 20-second stall at a gate.

  * THE FIELD GOES OUT BEFORE THE ACK. The boat treats the ack as permission to
    drive on. An ack that overtook the new positions would let it leave on the
    old ones -- which is exactly the failure the confirmation exists to prevent.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from uav_link import UavLink                                   # noqa: E402
from crusader_link import rxl_codec                            # noqa: E402

RED = rxl_codec.BEACON_FLASHING_RED
GREEN = rxl_codec.BEACON_FLASHING_GREEN
ENTRY = rxl_codec.BEACON_FLASHING_BLUE
EXIT = rxl_codec.BEACON_STEADY_BLUE
OFF = rxl_codec.BEACON_OFF

# Two pairs and four unlit blacks: the mission as it stands.
FIELD = [(0, 20.0, 0.0, ENTRY),
         (1, 38.0, 6.0, RED), (2, 38.0, -6.0, GREEN),
         (3, 45.0, 14.0, OFF), (4, 50.0, -16.0, OFF),
         (5, 62.0, 7.0, RED), (6, 62.0, -5.0, GREEN),
         (7, 72.0, 12.0, OFF), (8, 80.0, -9.0, OFF),
         (9, 92.0, 0.0, EXIT)]
DEG = 111320.0
BUOYS = [(i, n / DEG, e / DEG, c) for i, n, e, c in FIELD]
ANY_ENTRY, ANY_EXIT = (1.0, 103.0), (1.001, 103.001)

# Nothing listens on this port and nothing needs to: a udpout socket never
# blocks, and every assertion here reads state rather than the wire.
ENDPOINT = "udpout:127.0.0.1:14599"


def link(auto=False, plan=True):
    lk = UavLink(endpoint=ENDPOINT, auto_confirm=auto)
    if plan:
        lk.send_plan(BUOYS, ANY_ENTRY, ANY_EXIT, quiet=True)
    return lk


def recoloured():
    """The same ten buoys with the two gate-1 buoys swapped."""
    out = []
    for i, la, lo, c in BUOYS:
        if i == 1:
            c = GREEN
        elif i == 2:
            c = RED
        out.append((i, la, lo, c))
    return out


def main():
    fails = []

    def check(name, ok, detail=""):
        print("  [%s] %s%s" % ("ok" if ok else "FAIL", name,
                               "" if ok else "  <- " + str(detail)))
        if not ok:
            fails.append(name)

    # ------------------------------------- confirming is an operator action
    lk = link()
    lk._request(1)                                     # noqa: SLF001
    st = lk.status()
    check("a request is recorded as pending", st["pending"] == 1, st["pending"])
    check("...and is NOT answered on its own", st["confirmed"] == 0, st["confirmed"])
    sent_before = st["sent"]

    msg = lk.confirm()
    st = lk.status()
    check("confirm() answers it", st["confirmed"] == 1 and st["pending"] is None,
          (st["confirmed"], st["pending"]))
    check("...and retransmits the field with it", st["sent"] == sent_before + 1,
          (sent_before, st["sent"]))
    check("confirm() says which gate", "1" in msg, msg)

    check("confirming twice has nothing to answer",
          "nothing" in lk.confirm().lower(), lk.confirm())

    # ------------------------------------------- a re-ask must not wait again
    lk._request(1)                                     # noqa: SLF001
    st = lk.status()
    check("a re-ask of a confirmed gate is answered at once",
          st["pending"] is None, st["pending"])
    check("...and does not double-count", st["confirmed"] == 1, st["confirmed"])
    check("...and says so in the log",
          any("re-asked" in s for s in st["log"]), st["log"][-2:])

    # A NEW sequence number is a new question and must wait for the operator.
    lk._request(2)                                     # noqa: SLF001
    check("a new gate goes pending", lk.status()["pending"] == 2)

    # ------------------------------------------ the field changes on confirm
    lk = link()
    lk._request(1)                                     # noqa: SLF001
    lk.confirm(recoloured(), ANY_ENTRY, ANY_EXIT)
    plan = lk._plan[0]                                 # noqa: SLF001
    colours = {i: c for i, _la, _lo, c in plan}
    check("confirming with a new field transmits the new colours",
          colours[1] == GREEN and colours[2] == RED,
          (colours[1], colours[2]))

    # ----------------------------------------------- auto_confirm, for scripts
    lk = link(auto=True)
    lk._request(1)                                     # noqa: SLF001
    st = lk.status()
    check("auto_confirm answers without an operator",
          st["confirmed"] == 1 and st["pending"] is None,
          (st["confirmed"], st["pending"]))

    # ------------------------------------------------------------- rewind
    lk.rewind()
    st = lk.status()
    check("rewind forgets the confirmations", st["confirmed"] == 0)
    lk = link()
    lk._request(1)                                     # noqa: SLF001
    lk.confirm()
    lk.rewind()
    lk._request(1)                                     # noqa: SLF001
    check("...so the same seq is a fresh question afterwards",
          lk.status()["pending"] == 1, lk.status()["pending"])

    # -------------------------------------- nothing assigns a gate any more
    check("status carries no gate list", "gates" not in lk.status(),
          sorted(lk.status()))
    check("UavLink cannot assign gates", not hasattr(lk, "set_gates"))

    print("\n%s" % ("PASS" if not fails else "FAIL: %d of the above" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
