#!/usr/bin/env python3
"""bench_gate_pairs — a gate pair must mean red-to-starboard whichever way it was authored.

    python3 tools/bench/bench_gate_pairs.py

No ROS, no node, no radio on the other end: this only exercises UavLink's own
bookkeeping, so it runs anywhere in a second.

WHAT IT EXISTS TO CATCH, and why nothing else caught it. The boat derives a
gate's crossing heading from `bearing(green -> red) - 90` (nav::gateWaypoints).
That is a pure function of the pair -- no boat heading, no entry, no exit --
which is what makes it survive an unresolved GPS yaw, and equally what makes an
INVERTED pair come out as a heading 180 degrees wrong rather than as an error.

2026-09-13, SITL: the page authored pairs in click order, the operator clicked
green-then-red, and the aircraft said "red 3, green 1" about a green 3 and a
red 1. The boat crossed the first gate correctly, turned around, and drove back
through it with RED TO PORT. Nothing in any log was above [INFO].

So the field here is the one that was on screen when that happened, and the
headings are checked against the entry->exit course: a gate crossed more than
90 degrees off the run of the passage is the signature of the bug.
"""
import math
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

# id, east, north, beacon -- the field that produced the reversal.
FIELD = [(0, -31.66, 9.86, ENTRY), (1, -8.03, 11.40, RED), (2, -14.83, 33.18, RED),
         (3, -16.53, 19.59, GREEN), (4, -23.78, 28.55, GREEN), (5, 6.02, 50.94, EXIT),
         (6, -3.86, 47.70, OFF), (7, 6.02, 20.36, OFF), (8, -25.95, 43.06, OFF),
         (9, -17.61, 12.95, OFF)]
POS = {i: (e, n) for i, e, n, _c in FIELD}

# Offsets straight into degrees. The link only re-reads the COLOURS, so the
# conversion needs to be reversible, not accurate.
DEG = 111320.0
BUOYS = [(i, n / DEG, e / DEG, c) for i, e, n, c in FIELD]
ANY_ENTRY, ANY_EXIT = (1.0, 103.0), (1.001, 103.001)

# Nothing listens on this port and nothing needs to: a udpout socket never
# blocks, and every assertion here reads state rather than the wire.
ENDPOINT = "udpout:127.0.0.1:14599"


def crossing_heading(red_id, green_id):
    """nav::gateWaypoints' heading, in degrees, 0..360."""
    (re_, rn), (ge, gn) = POS[red_id], POS[green_id]
    return (math.degrees(math.atan2(re_ - ge, rn - gn)) - 90.0) % 360.0


def course():
    (ee, en), (xe, xn) = POS[0], POS[5]
    return math.degrees(math.atan2(xe - ee, xn - en)) % 360.0


def link(plan=True):
    lk = UavLink(endpoint=ENDPOINT)
    if plan:
        lk.send_plan(BUOYS, ANY_ENTRY, ANY_EXIT, quiet=True)
    return lk


def main():
    fails = []

    def check(name, ok, detail=""):
        print("  [%s] %s%s" % ("ok" if ok else "FAIL", name,
                               "" if ok else "  <- " + str(detail)))
        if not ok:
            fails.append(name)

    print("entry -> exit course: %.1f deg\n" % course())

    # ---------------------------------------------- the order it happened in
    # Gates authored BEFORE Transmit, which is what the page encourages and is
    # the case that needs the deferred check: at set_gates time there is no
    # plan to read the colours from.
    lk = link(plan=False)
    lk.set_gates([(3, 1), (4, 2)])
    check("gates before a plan are left alone", lk.status()["gates"] == [(3, 1), (4, 2)],
          lk.status()["gates"])
    lk.send_plan(BUOYS, ANY_ENTRY, ANY_EXIT, quiet=True)
    check("transmitting the plan fixes them", lk.status()["gates"] == [(1, 3), (2, 4)],
          lk.status()["gates"])

    # ------------------------------------------------------ the other order
    lk = link()
    lk.set_gates([(3, 1), (4, 2)])
    check("gates after a plan are fixed at once", lk.status()["gates"] == [(1, 3), (2, 4)],
          lk.status()["gates"])

    # ---------------------------------------------------- already correct
    lk = link()
    lk.set_gates([(1, 3), (2, 4)])
    check("a correct pair is untouched", lk.status()["gates"] == [(1, 3), (2, 4)],
          lk.status()["gates"])

    # ------------------------------------- what it must NOT silently repair
    # Two reds is a mistake the operator has to see, not one to guess at:
    # swapping it would be inventing a colour, dropping it would quietly
    # shorten the passage.
    lk = link()
    lk.set_gates([(1, 2)])
    check("two reds pass through unaltered", lk.status()["gates"] == [(1, 2)],
          lk.status()["gates"])
    check("...and are complained about",
          any("not a red-green pair" in s for s in lk.status()["log"]))

    lk = link()
    lk.set_gates([(3, 99)])
    check("an unknown id passes through unaltered", lk.status()["gates"] == [(3, 99)],
          lk.status()["gates"])

    # ---------------------------------- the retransmit must not rewind a run
    # send_plan runs at 0.2 Hz for the whole passage and re-checks the pairs
    # each time. If that reset what has been served, every retransmit would
    # answer the next request with gate 1 and the boat would circle the field.
    lk = link()
    lk.set_gates([(3, 1), (4, 2)])
    lk._answer(1)                                     # noqa: SLF001
    lk._answer(2)                                     # noqa: SLF001
    served = lk.status()["served"]
    before = len(lk.status()["log"])
    for _ in range(5):
        lk.send_plan(BUOYS, ANY_ENTRY, ANY_EXIT, quiet=True)
    check("a retransmit does not rewind what was served",
          lk.status()["served"] == served, lk.status()["served"])
    check("a retransmit says nothing once the pairs are right",
          len(lk.status()["log"]) == before,
          lk.status()["log"][before:])

    # ------------------------- a corrected pair withdraws its cached answer
    # The idempotent re-ask is about a lost REPLY, not about frozen content. A
    # Disruptive recolour can make the pair we already answered the wrong way
    # round, and the boat re-asks that same seq after PlanChanged halts the leg.
    lk = link()
    lk.set_gates([(1, 3)])
    lk._answer(1)                                     # noqa: SLF001
    check("an answer is cached", lk.status()["served"] == 1)
    recoloured = [(i, la, lo, (GREEN if c == RED else RED if c == GREEN else c))
                  for i, la, lo, c in BUOYS]
    lk.send_plan(recoloured, ANY_ENTRY, ANY_EXIT, quiet=True)
    check("a recolour flips the pair", lk.status()["gates"] == [(3, 1)],
          lk.status()["gates"])
    check("...and withdraws the stale answer",
          any("answer withdrawn" in s for s in lk.status()["log"]),
          lk.status()["log"][-3:])

    # And the opposite: a plain retransmission must withdraw nothing, or every
    # re-ask on a lossy link would be answered afresh and could skip a gate.
    lk = link()
    lk.set_gates([(1, 3)])
    lk._answer(1)                                     # noqa: SLF001
    lk.send_plan(BUOYS, ANY_ENTRY, ANY_EXIT, quiet=True)
    lk._answer(1)                                     # noqa: SLF001
    check("a retransmit keeps the answer",
          any("re-asked -> same answer" in s for s in lk.status()["log"]),
          lk.status()["log"][-3:])

    # --------------------------------- and the heading it all comes down to
    lk = link()
    lk.set_gates([(3, 1), (4, 2)])
    crs = course()
    for n, (r, g) in enumerate(lk.status()["gates"], 1):
        h = crossing_heading(r, g)
        off = abs((h - crs + 180) % 360 - 180)
        check("gate %d crosses %.0f deg, %.0f off the course" % (n, h, off), off <= 90,
              "reversed" if off > 90 else "")

    print("\n%s" % ("PASS" if not fails else "FAIL: %d of the above" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
