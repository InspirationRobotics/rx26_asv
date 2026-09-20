#!/usr/bin/env python3
"""bench_radio -- the Radio tab's record, rate estimates and TUNNEL decoding.

No ROS, no boat, no radio:

    python3 tools/bench/bench_radio.py

Drives crusader_groundstation.radio_core with its own clock, so a minute of
link takes no time, and crusader_link.tunnel_link with payloads packed the way
Ekko's telemetry_bridge packs them.

The NATIVE side of the link (RXL_SAFE_PASSAGE and friends) is covered by
`python3 -m crusader_link.rxl_codec --selftest`, which needs the generated
dialect; this bench needs nothing but the standard library.
"""
import os
import sys

REPO = os.path.join(os.path.dirname(__file__), "..", "..")
sys.path.insert(0, os.path.join(REPO, "crusader_groundstation"))
sys.path.insert(0, os.path.join(REPO, "crusader_link"))

from crusader_link import tunnel_link as tl                 # noqa: E402
from crusader_groundstation import radio_core as rc         # noqa: E402


def check(name, ok, detail=""):
    print("  [%s] %s%s" % ("ok" if ok else "FAIL", name,
                           "  -- " + str(detail) if detail and not ok else ""))
    return bool(ok)


class Clock:
    """A monotonic clock the bench moves by hand."""

    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def pack_map(n_buoys=3, confirmed=(1, 2)):
    buoys = [(i + 1, 32.9 + i * 1e-4, -117.0, "FLASHING_RED")
             for i in range(n_buoys)]
    out = bytearray([len(buoys)] + [int(i) & 0xFF for i in confirmed]
                    + [0] * (2 - len(confirmed)))
    for bid, lat, lon, label in buoys:
        out += tl._BUOY.pack(bid, int(round(lat * 1e7)), int(round(lon * 1e7)),
                             tl.CODE[label])
    return bytes(out)


def heard(log, name, ptype=0, summary="", src=200):
    log.add(rc.RX, src, 0, 42, name, ptype, summary, 30)


def main():
    r = []

    print("\ntunnel_link.describe -- the aircraft's format, read by the boat")
    name, s = tl.describe(tl.PAYLOAD_BOAT, tl.pack_boat(32.9238, -117.0386, 3, 4))
    r.append(check("boat packet named BOAT", name == "BOAT", name))
    r.append(check("  ...says what the aircraft thinks the boat is doing",
                   "transiting" in s and "target B4" in s, s))
    name, s = tl.describe(tl.PAYLOAD_BUOYS, pack_map())
    r.append(check("buoy map named BUOY_MAP", name == "BUOY_MAP", name))
    r.append(check("  ...a confirmed gate reads red then green",
                   "3 buoys" in s and "red B1" in s and "green B2" in s, s))
    _, s = tl.describe(tl.PAYLOAD_BUOYS, pack_map(confirmed=(5,)))
    r.append(check("  ...a single confirmed id reads as the exit", "exit B5" in s, s))
    _, s = tl.describe(tl.PAYLOAD_BUOYS, pack_map(confirmed=()))
    r.append(check("  ...no confirmation reads as nothing",
                   "confirmed nothing" in s, s))
    name, s = tl.describe(tl.PAYLOAD_TEST, tl.pack_test("test 4 from ekko"))
    r.append(check("test frame decodes to its text",
                   name == "TEST" and s == "test 4 from ekko", (name, s)))
    r.append(check("pack_test cuts to one TUNNEL payload",
                   len(tl.pack_test("x" * 400)) == tl.MAX_PAYLOAD))
    name, s = tl.describe(0x8019, bytes(6))
    r.append(check("an unknown TUNNEL type is named by its number",
                   name == "TUNNEL_0x8019" and "6 bytes" in s, (name, s)))
    try:
        name, s = tl.describe(tl.PAYLOAD_BOAT, b"\x01\x02")
        r.append(check("a truncated packet is reported, not raised",
                       name == "TUNNEL_0x8000" and "does not decode" in s, (name, s)))
    except Exception as e:                      # noqa: BLE001
        r.append(check("a truncated packet is reported, not raised", False, e))

    print("\nRadioLog: records")
    clock = Clock()
    log = rc.RadioLog(capacity=5, clock=clock, wall=lambda: 0.0)
    recs, newest, dropped = log.read()
    r.append(check("empty log reads empty",
                   recs == [] and newest == 0 and dropped == 0))
    log.add(rc.TX, 42, 191, 1, "USV_REACHED_GATE", 0, "gate 3", 24)
    heard(log, "BUOY_MAP", tl.PAYLOAD_BUOYS, "10 buoys, confirmed nothing")
    recs, newest, _ = log.read()
    r.append(check("records come back oldest first",
                   [x["name"] for x in recs] == ["USV_REACHED_GATE", "BUOY_MAP"],
                   recs))
    r.append(check("TX is named by where it went",
                   recs[0]["who"] == "Ekko", recs[0]["who"]))
    r.append(check("RX is named by who sent it",
                   recs[1]["who"] == "Ekko (bridge)", recs[1]["who"]))
    r.append(check("the cursor returns only newer records",
                   log.read(since_seq=newest)[0] == []))
    for _ in range(8):
        heard(log, "HEARTBEAT", 0, "SURFACE_BOAT", src=255)
    recs, newest, dropped = log.read()
    r.append(check("the ring keeps capacity and counts what fell off",
                   len(recs) == 5 and dropped == 5 and newest == 10,
                   (len(recs), dropped, newest)))
    systems = {s["sys"]: s for s in log.systems()}
    r.append(check("one entry per peer, sent and heard counted separately",
                   systems[1]["tx"] == 1 and systems[200]["rx"] == 1,
                   systems))
    r.append(check("the link node's own id is named",
                   rc.name_of(42) == "Crusader (link node)"))
    log.add(rc.TX, 42, 191, 0, "TEST", tl.PAYLOAD_TEST, "hi", 34)
    r.append(check("a broadcast is named as one",
                   log.read()[0][-1]["who"] == "broadcast"))
    clock.t += 7
    r.append(check("ages come from the clock",
                   abs({s["sys"]: s for s in log.systems()}[200]["heard_s"] - 7)
                   < 1e-9))

    print("\nRadioLog: one rate per format, so a quiet link is not a wrong format")
    clock = Clock()
    log = rc.RadioLog(clock=clock)
    streams = {s["name"]: s for s in log.streams()}
    r.append(check("both formats are scored", sorted(streams) ==
                   ["BUOY_MAP", "SAFE_PASSAGE"], sorted(streams)))
    r.append(check("never heard: no rate, no percentage",
                   streams["BUOY_MAP"]["rate_hz"] is None
                   and streams["BUOY_MAP"]["pct"] is None))
    for _ in range(60):
        heard(log, "BUOY_MAP", tl.PAYLOAD_BUOYS, "10 buoys")
        clock.t += 1.0
    streams = {s["name"]: s for s in log.streams()}
    r.append(check("a steady 1 Hz buoy map scores ~100%",
                   streams["BUOY_MAP"]["pct"] > 99, streams["BUOY_MAP"]))
    r.append(check("  ...and the other format still reads as never heard",
                   streams["SAFE_PASSAGE"]["rate_hz"] is None,
                   streams["SAFE_PASSAGE"]))
    r.append(check("  ...longest silence is about a second",
                   abs(streams["BUOY_MAP"]["longest_gap_s"] - 1.0) < 1e-6,
                   streams["BUOY_MAP"]))
    clock = Clock()
    log = rc.RadioLog(clock=clock)
    for i in range(60):
        if not 20 <= i < 30:
            heard(log, "BUOY_MAP", tl.PAYLOAD_BUOYS, "10 buoys")
        clock.t += 1.0
    st = {s["name"]: s for s in log.streams()}["BUOY_MAP"]
    r.append(check("ten missing packets lower the estimate", 80 < st["pct"] < 90, st))
    r.append(check("  ...and show as an eleven-second silence",
                   abs(st["longest_gap_s"] - 11.0) < 1e-6, st))
    clock = Clock()
    log = rc.RadioLog(clock=clock)
    for _ in range(5):
        heard(log, "SAFE_PASSAGE", 0, "3 buoys, entry/exit set", src=1)
        clock.t += 5.0
    st = {s["name"]: s for s in log.streams()}["SAFE_PASSAGE"]
    r.append(check("a 0.2 Hz passage plan scores ~100% against its own rate",
                   st["pct"] > 99, st))
    r.append(check("a link heard for 25 s is not scored against a minute",
                   st["rate_hz"] > 0.19, st))
    clock.t += 120
    st = {s["name"]: s for s in log.streams()}["SAFE_PASSAGE"]
    r.append(check("a sender gone quiet for two minutes scores 0%",
                   st["pct"] == 0.0 and st["heard"] == 0, st))
    r.append(check("  ...and its silence fills the whole window",
                   abs(st["longest_gap_s"] - rc.RATE_WINDOW_S) < 1e-6, st))
    log.clear()
    recs, newest, _ = log.read()
    r.append(check("clear empties the log and keeps the sequence",
                   recs == [] and newest == 5 and log.systems() == []
                   and log.streams()[0]["rate_hz"] is None, (recs, newest)))

    print("\n%d/%d" % (sum(r), len(r)))
    return 0 if all(r) else 1


if __name__ == "__main__":
    raise SystemExit(main())
