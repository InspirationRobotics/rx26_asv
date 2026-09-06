#!/usr/bin/env python3
"""bench_proximity — prove the OBSTACLE_DISTANCE sector maths without a boat.

    python3 tools/bench/bench_proximity.py --selftest    # exits nonzero on failure
    python3 tools/bench/bench_proximity.py --scene ahead # draw one scene

Stdlib only, no ROS, no numpy — so it runs in CI beside check_config.py and on a
laptop with nothing installed.

WHY THIS EXISTS. proximity_core converts REP-103 body coordinates (x forward,
y LEFT, angles counter-clockwise) into MAVLink BODY_FRD sectors (index 0 at the
nose, increasing CLOCKWISE). Get the sign wrong and the world is mirrored
left-for-right: the boat dodges to port when it should dodge to starboard. On a
symmetric test object — a single buoy dead ahead, which is exactly what anyone
reaches for first — a mirrored map looks perfect. That is why this is a test and
not a bench observation.

It has already earned its place: the first version painted a sector whose bearing
lay OUTSIDE the object (a buoy at 45 deg reported an obstacle at 40 deg), and
could paint zero sectors for a narrow object sitting between two sector bearings.
"""
import argparse
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "crusader_perception"))
from crusader_perception import proximity_core as pc      # noqa: E402

NR = pc.NO_READING


def polar(bearing_cw_deg, range_m, extent=0.3):
    """A cluster at a clockwise bearing, expressed in REP-103 body coords.

    Written the long way round on purpose: the test must construct its inputs
    in the INPUT convention, or it would just be asserting that a function is
    its own inverse.
    """
    a = math.radians(-bearing_cw_deg)          # cw -> ccw
    return (range_m * math.cos(a), range_m * math.sin(a), extent)


SCENES = {
    "empty":     [],
    "ahead":     [polar(0, 10.0)],
    "stbd_bow":  [polar(45, 5.0)],
    "port_bow":  [polar(315, 5.0)],
    "gate":      [polar(350, 8.0), polar(10, 8.0)],
    "astern":    [polar(180, 10.0)],
    "wide":      [polar(0, 5.0, extent=3.0)],
    "two_depths": [polar(0, 10.0), polar(0, 4.0)],
}


def draw(sectors, increment_deg=5.0):
    """One line per filled sector, plus a coarse compass rose."""
    filled = [(i, d) for i, d in enumerate(sectors) if d != NR]
    if not filled:
        print("    (nothing in view — all 72 sectors NO_READING)")
        return
    for i, d in filled:
        b = (i * increment_deg) % 360.0
        side = "ahead" if b < 15 or b > 345 else ("stbd" if b < 180 else "port")
        print(f"    sector {i:2d}  {b:5.1f} deg cw  {d/100.0:5.2f} m  {side}")


def selftest():
    fails = []

    def chk(name, got, want):
        if got != want:
            fails.append(f"{name}: got {got!r} want {want!r}")
        print(f"  [{'ok' if got == want else 'FAIL'}] {name}")

    # --- the conversion itself ---
    chk("nose is 0 deg", round(pc.bearing_cw_deg(10, 0), 1), 0.0)
    chk("port beam is 270 (NOT 90)", round(pc.bearing_cw_deg(0, 10), 1), 270.0)
    chk("stbd beam is 90", round(pc.bearing_cw_deg(0, -10), 1), 90.0)
    chk("astern is 180", round(pc.bearing_cw_deg(-10, 0), 1), 180.0)
    chk("port bow is 315", round(pc.bearing_cw_deg(10, 10), 1), 315.0)

    # --- round trip through polar() catches a mirrored convention ---
    for b in (0, 45, 90, 135, 225, 315):
        x, y, _ = polar(b, 7.0)
        chk(f"round trip {b} deg", round(pc.bearing_cw_deg(x, y), 1), float(b))

    # --- placement ---
    s = pc.build_sectors(SCENES["ahead"])
    chk("ahead lands in sector 0", s[0], 1000)
    chk("ahead leaves the beam unseen", s[18], NR)
    chk("stbd bow reports 45", pc.summarise(pc.build_sectors(SCENES["stbd_bow"]))[2], 45.0)
    chk("port bow reports 315", pc.summarise(pc.build_sectors(SCENES["port_bow"]))[2], 315.0)

    # a narrow object between two sector bearings must still paint one
    s = pc.build_sectors([polar(42.5, 5.0, extent=0.1)])
    chk("42.5 deg object is not dropped", pc.summarise(s)[0] >= 1, True)

    # --- the two sentinels are not interchangeable ---
    s = pc.build_sectors(SCENES["empty"])
    chk("empty scene is all NO_READING", set(s), {NR})
    chk("empty scene contains no 0 (= touching)", 0 in s, False)

    # --- hull occlusion ---
    chk("astern is suppressed", pc.summarise(pc.build_sectors(SCENES["astern"]))[0], 0)

    # --- range gates discard, never clamp ---
    chk("beyond max is discarded", pc.summarise(pc.build_sectors([polar(0, 50.0)]))[0], 0)
    chk("inside min is discarded", pc.summarise(pc.build_sectors([polar(0, 0.2)]))[0], 0)

    # --- nearest wins where objects share a sector ---
    chk("nearest wins", pc.build_sectors(SCENES["two_depths"])[0], 400)

    # --- angular extent ---
    narrow = pc.summarise(pc.build_sectors([polar(0, 5.0, 0.3)]))[0]
    wide = pc.summarise(pc.build_sectors(SCENES["wide"]))[0]
    chk("a wide object paints more sectors", wide > narrow, True)

    chk("always 72 sectors", len(pc.build_sectors([])), 72)

    print()
    if fails:
        print(f"FAIL — {len(fails)} check(s):")
        for f in fails:
            print("  " + f)
        return 1
    print("PASS")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--scene", choices=sorted(SCENES))
    a = ap.parse_args()
    if a.scene:
        print(f"  scene: {a.scene}")
        draw(pc.build_sectors(SCENES[a.scene]))
        return 0
    return selftest()


if __name__ == "__main__":
    sys.exit(main())
